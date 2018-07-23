#pragma once
#include <ros/ros.h>
#include <std_msgs/Int32.h>
//#include <tf/tfMessage.h>
#include <iostream>
using namespace std;

class KyControl{
private:
		ros::Publisher keyboard_control_pub;
		ros::Subscriber keyboard_control_sub;
		void Callback(const std_msgs::Int32::ConstPtr& msg);
		bool is_offboard=false;
		int scanKeyboard();
public:
		KyControl(ros::NodeHandle & n){
			keyboard_control_pub = n.advertise<std_msgs::Int32>("keyboard_control_msg",1000);
			keyboard_control_sub = n.subscribe("offboard_mode_msg",1000,&KyControl::Callback,this);
		}
		void run();
};
