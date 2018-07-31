#pragma once

#include <ros/ros.h>
#include <map>
#include "../include/mavlink/v1.0/common/mavlink.h"
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <serial_comm/velocity.h>

#include "simple_comm/simple_serial_port.h"

class Communication
{
public:
	Communication(ros::NodeHandle &n)
	{
		imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1000);
		tf_sub = n.subscribe("tf", 1000, &Communication::Callback, this);
		offboard_mode_pub = n.advertise<std_msgs::Int32>("offboard_mode_msg",1000);
		velocity_sub = n.subscribe("path_planning", 1000, &Communication::Callback_v, this);
	}
	
	void start()
	{
		while (ros::ok())
		{
			Process(nullptr);
			ros::spinOnce();
		}

	}
	static uint8_t serial_port_receive_buffer[1024];
	static uint8_t serial_port_send_buffer[1024];

private:
	std::map<uint32_t, mavlink_highres_imu_t> highres_imu_map;
	ros::Publisher imu_pub;
	ros::Subscriber tf_sub;
	ros::Publisher offboard_mode_pub;
	ros::Subscriber velocity_sub;
	void Callback(const tf::tfMessage::ConstPtr &_msg);
	void Process(const tf::tfMessage::ConstPtr *_msg);
	void Callback_v(const serial_comm::velocity::ConstPtr &_msg);
	void send_tf_msg(const tf::tfMessage::ConstPtr *tf_msg);
	void receive_mavlink_send_imu();
	void print_local_position(mavlink_local_position_ned_t& local_position);
	SimpleSerialPort *serial_port;
	
	float delta_x= 0;
	float delta_y= 0;
	float delta_z= 0;
	float delta_yaw= 0;
	float source_x = 0;
	float source_y = 0;
	float source_z = 0;
	float source_yaw = 0;
	float local_x;
	float local_y;
	float local_z;
	float local_yaw;
};
