#pragma once

#include "ros/ros.h"
#include <map>
#include "../include/mavlink/v1.0/common/mavlink.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "tf/tfMessage.h"

class Communication
{
public:
	Communication(ros::NodeHandle &n)
	{
		imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1000);
		tf_sub = n.subscribe("tf", 1000, &Communication::Callback, this);
	}
	
	void start()
	{
		while (ros::ok())
		{
			Process(nullptr);
			ros::spinOnce();
		}

	}

private:
	std::map<uint32_t, mavlink_highres_imu_t> highres_imu_map;
	ros::Publisher imu_pub;
	ros::Subscriber tf_sub;
	void Callback(const tf::tfMessage::ConstPtr &_msg);
	void Process(const tf::tfMessage::ConstPtr *_msg);
	

};