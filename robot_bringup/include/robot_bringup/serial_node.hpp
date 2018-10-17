#ifndef __SERIAL_NODE_H__
#define __SERIAL_NODE_H__

#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <string>
#include <cstring>
#include <math.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

#define TX_MSG_LEN  (12)
#define RX_MSG_LEN  (31)

/*
 * 0x0d -> '\r'
 * 0x0a -> '\n'
**/
#define DATA_HEADER_0   (0x55)
#define DATA_HEADER_1   (0Xaa)
#define DATA_TAIL_0   (0x0d)
#define DATA_TAIL_1   (0X0a)

typedef  struct{
    int16_t fr_speed_rpm;
    int16_t fl_speed_rpm;
    int16_t bl_speed_rpm;
    int16_t br_speed_rpm;
	int16_t acc_linear_x;
	int16_t acc_linear_y;
	int16_t acc_linear_z;
	int16_t vel_angular_x;
	int16_t vel_angular_y;
	int16_t vel_angular_z;
	int16_t ist_mag_x;
	int16_t ist_mag_y;
	int16_t ist_mag_z;
}rx_msg_t;

typedef  struct{
	float acc_lin_x;
	float acc_lin_y;
	float acc_lin_z;
	float vel_ang_x;
	float vel_ang_y;
	float vel_ang_z;
	float mag_x;
	float mag_y;
	float mag_z;
}imu_msg_t;


string readLine(void);
void read_serial(void);
void write_serial(const geometry_msgs::Twist& cmd_input);

#endif
