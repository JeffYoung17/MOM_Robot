#!/usr/bin/env python
# --coding:utf-8 --

import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client
from robot_calib.cfg import CalibrateAngularConfig
import tf
from math import radians, copysign
from transform_utils import quat_to_angle, normalize_angle

class CalibrateAngular():
    def __init__(self):
        # Give the node a name
		# 节点名字
        rospy.init_node('calibrate_angular', anonymous=False)
        
        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)
        
        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)
        
        # The test angle is 360 degrees
		# 设置角度校准的相关参数
        self.test_angle = radians(rospy.get_param('~test_angle', 360.0))
        self.speed = rospy.get_param('~speed', 1.0) # radians per second
        self.tolerance = radians(rospy.get_param('tolerance', 1)) # degrees converted to radians
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)
        
        # Publisher to control the robot's speed
		# 发布速度指令到cmd_vel话题
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # Fire up the dynamic_reconfigure server
        dyn_server = Server(CalibrateAngularConfig, self.dynamic_reconfigure_callback)
        
        # Connect to the dynamic_reconfigure server
        dyn_client = dynamic_reconfigure.client.Client("calibrate_angular", timeout=60)
        
        # The base frame is usually base_link or base_footprint
		# 设置机器人坐标系的名字
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
		# 设置里程计坐标系的名字
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # Initialize the tf listener
		# 初始化tf变换数据的收听功能
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
            
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
        
        reverse = 1
        
        while not rospy.is_shutdown():
			# 如果测试可以开始
            if self.start_test:
                # Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()
                
                last_angle = self.odom_angle
                turn_angle = 0
                self.test_angle *= reverse
                error = self.test_angle - turn_angle
                                
                # Alternate directions between tests
                reverse = -reverse
                
				# 只要测试处于开始状态并且角度差值大于容忍值
                while abs(error) > self.tolerance and self.start_test:
                    if rospy.is_shutdown():
                        return
                    
                    # Rotate the robot to reduce the error
					# 旋转机器人减少角度差值
                    move_cmd = Twist()
                    move_cmd.angular.z = copysign(self.speed, error)
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                 
                    # Get the current rotation angle from tf                   
                    # 从tf变换中读取并设置当前的角度/姿态
                    self.odom_angle = self.get_odom_angle()
                    
                    # Compute how far we have gone since the last measurement
                    # 计算从上一时刻到当前转过的角度
                    delta_angle = self.odom_angular_scale_correction * normalize_angle(self.odom_angle - last_angle)
                    
                    # Add to our total angle so far
                    turn_angle += delta_angle

                    # Compute the new error
                    error = self.test_angle - turn_angle

                    # Store the current angle for the next comparison
                    last_angle = self.odom_angle
                                    
                # Stop the robot
				# 退出while循环后, 就停止机器人
                self.cmd_vel.publish(Twist())
                
                # Update the status flag
                self.start_test = False
                params = {'start_test': False}
                dyn_client.update_configuration(params)
                
            rospy.sleep(0.5)
                    
        # Stop the robot
        self.cmd_vel.publish(Twist())
        
    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        
        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))
            
    def dynamic_reconfigure_callback(self, config, level):
        self.test_angle =  radians(config['test_angle'])
        self.speed = config['speed']
        self.tolerance = radians(config['tolerance'])
        self.odom_angular_scale_correction = config['odom_angular_scale_correction']
        self.start_test = config['start_test']
        
        return config
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        CalibrateAngular()
    except:
        rospy.loginfo("Calibration terminated.")
