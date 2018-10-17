#ifndef __BASE_NODE_H__
#define __BASE_NODE_H__

#include <ros/time.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "robot_msgs/Encoder.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef  struct{
	double x;
	double y;
	double theta;
	double vx;
	double vy;
	double v_theta;
}odom_data_t;

typedef struct{
	float yaw;
	float vel_angular_z;
}imu_data_t;

void imu_callback(const sensor_msgs::Imu& imu_msg);
void encoder_callback(const robot_msgs::Encoder& encoder_msg);

#endif
