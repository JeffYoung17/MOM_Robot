#include <ros/ros.h>
#include "robot_bringup/base_node.hpp"

odom_data_t odom_data = {
	.x = 0.0,
	.y = 0.0,
	.theta = 0.0,
	.vx = 0.0,
	.vy = 0.0,
	.v_theta = 0.0
};

imu_data_t imu_data = {
	.yaw = 0.0,
	.vel_angular_z = 0.0
};

ros::Time odom_current_time, odom_last_time;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_node");
	ros::Time::init();
    ros::Rate loop_rate(50);

	odom_current_time = ros::Time::now();
	odom_last_time = ros::Time::now();

    ros::NodeHandle nh;
    ros::Subscriber encoder_sub = nh.subscribe("encoder", 50, encoder_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu/data", 50, imu_callback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    tf::TransformBroadcaster odom_broadcaster;

    // publish topic of odom and broadcast tf of base_footprint frame to odom frame
    while(ros::ok())
    {
        odom_current_time = ros::Time::now();
        
		// broadcast transform from odom frame to base_footprint frame
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = odom_current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        geometry_msgs::Quaternion odom_quat;
        odom_quat = tf::createQuaternionMsgFromYaw(odom_data.theta);
        odom_trans.transform.translation.x = odom_data.x;
        odom_trans.transform.translation.y = odom_data.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat; 
        odom_broadcaster.sendTransform(odom_trans);

        // publish topic: odom, use the msg from odom_data
        nav_msgs::Odometry odom_topic_msg;
        odom_topic_msg.header.stamp = odom_current_time;
        odom_topic_msg.header.frame_id = "odom";
        odom_topic_msg.child_frame_id = "base_footprint";
        odom_topic_msg.pose.pose.position.x = odom_data.x;
        odom_topic_msg.pose.pose.position.y = odom_data.y;
        odom_topic_msg.pose.pose.position.z = 0.0;
        odom_topic_msg.twist.twist.linear.x = odom_data.vx;
        odom_topic_msg.twist.twist.linear.y = odom_data.vy;
        odom_topic_msg.twist.twist.angular.z = odom_data.v_theta;
        odom_pub.publish(odom_topic_msg);
		
		ros::spinOnce();
		
		loop_rate.sleep();
		
    }//end of while

    return 0;
}//end of main


/*
 * update v_theta
**/
void imu_callback(const sensor_msgs::Imu& imu_msg) 
{
    imu_data.vel_angular_z = imu_msg.angular_velocity.z;
}


void encoder_callback(const robot_msgs::Encoder& encoder_msg)
{
    ros::Time temp_cur_time;
    // unit is m/s
    odom_data.vx = (+encoder_msg.fl_speed_rpm + encoder_msg.bl_speed_rpm - encoder_msg.br_speed_rpm - encoder_msg.fr_speed_rpm)\
        * 478.0f / 60.0f / 19.0f /4.0f /1000.0f; 
    odom_data.vy = (-encoder_msg.fl_speed_rpm + encoder_msg.bl_speed_rpm + encoder_msg.br_speed_rpm - encoder_msg.fr_speed_rpm)\
        * 478.0f / 60.0f / 19.0f /4.0f /1000.0f; 
        
    // unit is rad/s, if you want deg/s, just divide by 57.3
    // odom_data.v_theta = -(encoder_msg.fr_speed_rpm + encoder_msg.fl_speed_rpm + encoder_msg.bl_speed_rpm + encoder_msg.br_speed_rpm) * 478.0f / 60.0f / 19.0f / 4.0f / ((386+540)/2.0f);
		
	odom_data.v_theta = imu_data.vel_angular_z;
    
    temp_cur_time = ros::Time::now();
    double dt = (temp_cur_time - odom_last_time).toSec();
    double delta_x = (odom_data.vx * cos(odom_data.theta) - odom_data.vy * sin(odom_data.theta)) * dt;
    double delta_y = (odom_data.vx * sin(odom_data.theta) + odom_data.vy * cos(odom_data.theta)) * dt;
    double delta_theta = odom_data.v_theta * dt;

    odom_data.x += delta_x;
    odom_data.y += delta_y;
    odom_data.theta += delta_theta;
    ROS_INFO("theta is %5.1f", odom_data.theta);
    odom_last_time = temp_cur_time;
}






    



    
    

