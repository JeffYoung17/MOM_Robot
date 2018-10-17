#include <ros/ros.h>
#include "robot_bringup/serial_node.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "robot_msgs/Encoder.h"

using namespace std;
using namespace boost::asio;

//serial port param
io_service iosev;
serial_port sp(iosev, "/dev/mecanum_base");

rx_msg_t rx_msg = {
    .fr_speed_rpm = 0,
    .fl_speed_rpm = 0,
    .bl_speed_rpm = 0,
    .br_speed_rpm = 0,
	.acc_linear_x = 0,
	.acc_linear_y = 0,
	.acc_linear_z = 0,
	.vel_angular_x = 0,
	.vel_angular_y = 0,
	.vel_angular_z = 0,
	.ist_mag_x = 0,
	.ist_mag_y = 0,
	.ist_mag_z = 0
};

imu_msg_t imu_msg = {
	.acc_lin_x = 0.0,
	.acc_lin_y = 0.0,
	.acc_lin_z = 0.0,
	.vel_ang_x = 0.0,
	.vel_ang_y = 0.0,
	.vel_ang_z = 0.0,
	.mag_x = 0.0,
	.mag_y = 0.0,
	.mag_z = 0.0
};


ros::Time imu_current_time;

int main(int argc, char** argv)
{
    try
    {
		sp.set_option(serial_port::baud_rate(115200));
		sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp.set_option(serial_port::parity(serial_port::parity::none));
        sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp.set_option(serial_port::character_size(8));
		ROS_INFO("Serial Port initialized successful.");
		
        ros::init(argc, argv, "serial_node");
        ros::Time::init();
        ros::Rate loop_rate(50);
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe("cmd_vel", 50, write_serial);
		ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 50);
		ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 50);
		ros::Publisher encoder_pub = nh.advertise<robot_msgs::Encoder>("encoder", 50);
        ROS_INFO("ROS Node initialized successful.");
        
        while(ros::ok()){
			
            // update rx_msg(include imu and encoder data)
			read_serial();
            
            // publish topic of imu/raw_data, imu/mag, encoder
			sensor_msgs::Imu imu_topic_msg;
			sensor_msgs::MagneticField mag_topic_msg;
			robot_msgs::Encoder encoder_topic_msg;
            imu_current_time = ros::Time::now();

			imu_topic_msg.header.stamp = imu_current_time;
			imu_topic_msg.header.frame_id = "imu_link";
			imu_topic_msg.angular_velocity.x = imu_msg.vel_ang_x;
			imu_topic_msg.angular_velocity.y = imu_msg.vel_ang_y;
			imu_topic_msg.angular_velocity.z = imu_msg.vel_ang_z;
			imu_topic_msg.linear_acceleration.x = imu_msg.acc_lin_x;
			imu_topic_msg.linear_acceleration.y = imu_msg.acc_lin_y;
			imu_topic_msg.linear_acceleration.z = imu_msg.acc_lin_z;

			mag_topic_msg.header.stamp = imu_current_time;
			mag_topic_msg.header.frame_id = "imu_link";
			mag_topic_msg.magnetic_field.x = imu_msg.mag_x;
			mag_topic_msg.magnetic_field.y = imu_msg.mag_y;
			mag_topic_msg.magnetic_field.z = imu_msg.mag_z;

			encoder_topic_msg.fr_speed_rpm = rx_msg.fr_speed_rpm;
			encoder_topic_msg.fl_speed_rpm = rx_msg.fl_speed_rpm;
			encoder_topic_msg.bl_speed_rpm = rx_msg.bl_speed_rpm;
			encoder_topic_msg.br_speed_rpm = rx_msg.br_speed_rpm;

			imu_pub.publish(imu_topic_msg);
			mag_pub.publish(mag_topic_msg);
			encoder_pub.publish(encoder_topic_msg);

			// process cmd_vel callback
			ros::spinOnce();
            loop_rate.sleep();
        }//end of while

        iosev.run();

    }//end of try

    catch(exception& err){
        ROS_ERROR("Exception Error: %s", err.what());
        return -1;
    }

    return 0;
}//end of main

/*
    transmit velocity command to mcu
*/
void write_serial(const geometry_msgs::Twist& cmd_input)
{
    uint8_t send_data[TX_MSG_LEN] = {0};
    int16_t check_data = 0;
    float raw_data[3] = {0.0};

    raw_data[0] = 1000 * cmd_input.linear.x;
    raw_data[1] = 1000 * cmd_input.linear.y;
    raw_data[2] = 1000 * cmd_input.angular.z;

    check_data = (int16_t)(raw_data[0] + raw_data[1] + raw_data[2]);

    send_data[0] = DATA_HEADER_0;
    send_data[1] = DATA_HEADER_1;
    send_data[2] = ((int16_t)raw_data[0]) & 0x00ff;
    send_data[3] = (((int16_t)raw_data[0]) & 0xff00) >> 8;
    send_data[4] = ((int16_t)raw_data[1]) & 0x00ff;
    send_data[5] = (((int16_t)raw_data[1]) & 0xff00) >> 8;
    send_data[6] = ((int16_t)raw_data[2]) & 0x00ff;
    send_data[7] = (((int16_t)raw_data[2]) & 0xff00) >> 8;
    send_data[8] = ((int16_t)check_data) & 0x00ff;
    send_data[9] = (((int16_t)check_data) & 0xff00) >> 8;
    send_data[10] = DATA_TAIL_0;
    send_data[11] = DATA_TAIL_1;
	//ROS_INFO("tx_buf: %d %d %d %d %d %d %d %d", send_data[2], send_data[3], send_data[4],
		//send_data[5], send_data[6], send_data[7], send_data[8], send_data[9]);

	boost::asio::write(sp, buffer(send_data));
	//ROS_INFO("velocity_cmd is sending");
}

/*
    update data of wheel_odom
    data type is double
*/
void read_serial(void)
{
    uint8_t odom_buf[RX_MSG_LEN] = {0};
	uint8_t check_num = 0;
    
	string str;
	str = readLine();

	if(str.size() != RX_MSG_LEN)
	{
		// 因为接受到的字符串buf长度并不等于要求数据的长度,所以直接返回,跳出函数
		return ;
	}

	char* pdata = new char[str.size()+1];
    for(int i=0; i<RX_MSG_LEN; i++)
    {
        pdata[i] = str[i];
    }
    for(int i=0; i<RX_MSG_LEN; i++)
    {
        odom_buf[i] = pdata[i];

    }
	delete[] pdata;
    /*
	ROS_INFO("odom_buf is:\t %d %d %d %d %d %d %d %d",
        odom_buf[2], odom_buf[3], odom_buf[4], odom_buf[5],
		odom_buf[6], odom_buf[7], odom_buf[8], odom_buf[9]);   
    */
	check_num = odom_buf[2];
    for(int i = 3; i < RX_MSG_LEN - 3; i++)
    {
        check_num = check_num ^ odom_buf[i];
    }

    if(check_num == odom_buf[28])
    {
        //ROS_INFO("rx data:\t %d %d %d %d %d %d %d %d %d %d",
        //    odom_buf[2], odom_buf[3], odom_buf[4], odom_buf[5], odom_buf[6],
        //    odom_buf[7], odom_buf[8], odom_buf[9], odom_buf[10], odom_buf[11]);
        ROS_INFO("ojbk, bingo! check num is %d",check_num);
        
        rx_msg.fr_speed_rpm = (((int16_t)odom_buf[3]) << 8) | (int16_t)odom_buf[2];
        rx_msg.fl_speed_rpm = (((int16_t)odom_buf[5]) << 8) | (int16_t)odom_buf[4];
        rx_msg.bl_speed_rpm = (((int16_t)odom_buf[7]) << 8) | (int16_t)odom_buf[6];
        rx_msg.br_speed_rpm = (((int16_t)odom_buf[9]) << 8) | (int16_t)odom_buf[8];
		rx_msg.vel_angular_x = (((int16_t)odom_buf[11]) << 8) | (int16_t)odom_buf[10];
		rx_msg.vel_angular_y = (((int16_t)odom_buf[13]) << 8) | (int16_t)odom_buf[12];
		rx_msg.vel_angular_z = (((int16_t)odom_buf[15]) << 8) | (int16_t)odom_buf[14];
		rx_msg.acc_linear_x = (((int16_t)odom_buf[17]) << 8) | (int16_t)odom_buf[16];
		rx_msg.acc_linear_y = (((int16_t)odom_buf[19]) << 8) | (int16_t)odom_buf[18];
		rx_msg.acc_linear_z = (((int16_t)odom_buf[21]) << 8) | (int16_t)odom_buf[20];
		rx_msg.ist_mag_x = (((int16_t)odom_buf[23]) << 8) | (int16_t)odom_buf[22];
		rx_msg.ist_mag_y = (((int16_t)odom_buf[25]) << 8) | (int16_t)odom_buf[24];
		rx_msg.ist_mag_z = (((int16_t)odom_buf[27]) << 8) | (int16_t)odom_buf[26];
		
		// real num of imu raw data
		imu_msg.vel_ang_x = rx_msg.vel_angular_x / 16.384f / 57.3f;
		imu_msg.vel_ang_y = rx_msg.vel_angular_y / 16.384f / 57.3f;
		imu_msg.vel_ang_z = rx_msg.vel_angular_z / 16.384f / 57.3f;
		imu_msg.acc_lin_x = rx_msg.vel_angular_x / 4096.0f;
		imu_msg.acc_lin_y = rx_msg.vel_angular_y / 4096.0f;
		imu_msg.acc_lin_z = rx_msg.vel_angular_z / 4096.0f;
		imu_msg.mag_x = rx_msg.ist_mag_x / 5120000.0f;
		imu_msg.mag_y = rx_msg.ist_mag_y / 5120000.0f;
		imu_msg.mag_z = rx_msg.ist_mag_z / 3276800.0f;
        
    } //end of if

    else{
        ROS_INFO("fuck you, data is not right!");
    }

} //end of function

/**
 * 接收到'\n'的时候：
 * 首先判断result的size，若为0，即'\n'是第一个接收到的，则累加
 * 若size不为0，且最后一个为'\r'，则加上'\n'，返回结果(表示收到消息尾)，放到后面处理成数组做校验判断
*/
string readLine()
{
	//Reading data char by char, code is optimized for simplicity, not speed
	char single_byte;
	string result;
	for(;;)
	{
		boost::asio::read(sp, buffer(&single_byte, 1));
        //ROS_INFO("%d", single_byte);
		switch(single_byte)
		{
			case '\n':
				if(result.size() != 0 && result[result.size() - 1] == '\r'){
					result += single_byte;
					return result;
				}
				else if(result.size() == 0){
					result += single_byte;
				}

				else{
					result += single_byte;
				}

			default:
				result += single_byte;
		}
	}
}
