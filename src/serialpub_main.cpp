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
#include "handler/rosnodehandler.h"
#include "io/serialwrapper.h"

using namespace std;

int main(int argc, char **argv)
{
	std::string devName;
	if (argc != 1 && argc != 2)
	{
		ROS_ERROR("Incorrect command. Arguments should contain only serial device name");
	}
	else
	{
		if (argc == 2)
		{
			devName = std::string(argv[1]);
		}
	}
	ros::init(argc, argv, "serial_pub");
	serialWrapper sp(devName);
	rosNodeHandler node(&sp);
	node.run();	
	return 0;
}


