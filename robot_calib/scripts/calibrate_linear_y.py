#!/usr/bin/env python
# --coding:utf-8 --

import rospy
from geometry_msgs.msg import Twist, Point
from math import copysign, sqrt, pow
from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client
from robot_calib.cfg import CalibrateLinearConfig
import tf

class CalibrateLinear():
    def __init__(self):
        # Give the node a name
        # 节点名字
        rospy.init_node('calibrate_linear', anonymous=False)
        
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        
        # How fast will we check the odometry values?
		# 获取里程计数据的频率
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)
        
        # Set the distance to travel
		# 设置校准的相关参数
        self.test_distance = rospy.get_param('~test_distance', 1.0) # meters
        self.speed = rospy.get_param('~speed', 0.15) # meters per second
        self.tolerance = rospy.get_param('~tolerance', 0.01) # meters
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)
        
        # Publisher to control the robot's speed
		# 发布cmd_vel话题
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # Fire up the dynamic_reconfigure server
        dyn_server = Server(CalibrateLinearConfig, self.dynamic_reconfigure_callback)
        
        # Connect to the dynamic_reconfigure server
        dyn_client = dynamic_reconfigure.client.Client("calibrate_linear", timeout=60)
 
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
		# 获取机器人坐标系名字
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
		# 获取里程计坐标系名字
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
		# 初始化tf变换数据的收听功能
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))   
            # self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(60.0))
                self.base_frame = '/base_footprint'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception") 

        rospy.loginfo("Bring up rqt_reconfigure to control the test.") 
  
        self.position = Point()
        
        # Get the starting position from the tf transform between the odom and base frames
        # 设置当前位置为机器人的起点
        self.position = self.get_position()
        x_start = self.position.x
        y_start = self.position.y
            
        move_cmd = Twist()
		
        # 以一定频率开始循环    
        while not rospy.is_shutdown():
            # Stop the robot by default
            move_cmd = Twist()
            
            if self.start_test:
                # Get the current position from the tf transform between the odom and base frames
                self.position = self.get_position()
                
                # Compute the Euclidean distance from the target point
                distance = sqrt(pow((self.position.x - x_start), 2) +
                                pow((self.position.y - y_start), 2))
                                
                # Correct the estimated distance by the correction factor
                distance *= self.odom_linear_scale_correction
                
                # How close are we?
                error =  distance - self.test_distance
                
                # Are we close enough?
                if not self.start_test or abs(error) <  self.tolerance:
                    self.start_test = False
                    params = {'start_test': False}
                    rospy.loginfo(params)
                    dyn_client.update_configuration(params)
                else:
                    # If not, move in the appropriate direction
					move_cmd.linear.y = copysign(self.speed, -1 * error)
            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
                
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        # Stop the robot
        self.cmd_vel.publish(Twist())
        
    def dynamic_reconfigure_callback(self, config, level):
        self.test_distance = config['test_distance']
        self.speed = config['speed']
        self.tolerance = config['tolerance']
        self.odom_linear_scale_correction = config['odom_linear_scale_correction']
        self.start_test = config['start_test']
        
        return config
        
    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans)
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        CalibrateLinear()
        rospy.spin()
    except:
        rospy.loginfo("Calibration terminated.")
