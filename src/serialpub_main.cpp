/**
* serialpub_main.cpp
* @author Zhang Xiang
* @description 
* @created Sun Sep 30 2018 09:55:21 GMT+0800 (CST)
* @license MIT
* @copyright All rights reserved, 2018
* @last-modified Sun Sep 30 2018 10:10:52 GMT+0800 (CST)
*/

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <gflags/gflags.h>
#include "handler/rosnodehandler.h"
#include "io/serialwrapper.h"

DEFINE_string(device_name, "",
              "Serial device of your flight controller");

int main(int argc, char **argv)
{
	using namespace std;
#ifdef GFLAGS_NAMESPACE
    GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
#else
    gflags::ParseCommandLineFlags(&argc, &argv, true);
#endif
	if(FLAGS_device_name.empty())
	{
		ROS_ERROR("Device name should not be empty");
		exit(EXIT_FAILURE);
	}
	ros::init(argc, argv, "serial_pub");
	serialWrapper sp(FLAGS_device_name);
	rosNodeHandler node(&sp);
	node.run();	
	return 0;
}


