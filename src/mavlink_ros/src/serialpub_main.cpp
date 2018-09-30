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
	ros::init(argc, argv, "mavros_link");
	serialWrapper sp(devName);
	rosNodeHandler node(&sp);
	node.run();	
	return 0;
}


