#include "ros/ros.h"

// serial port
#include "simple_comm/simple_serial_port.h"
#include <iostream>
#include <string>

#include "communication.h"

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
			devName = "/dev/" + std::string(argv[1]);
			SimpleSerialPort::enableSerial();
		}
	}
	ros::init(argc, argv, "tf_subscriber");
	ros::NodeHandle n;
	// initialize serial port
	SimpleSerialPort *serial_port = SimpleSerialPort::getInstance();
	if (serial_port)
	{
		serial_port->openPort(devName.c_str(), 921600);
		if (!serial_port->isOpen())
		{
			ROS_ERROR("Serial port open failed");
			return -1;
		}
		else
		{
			ROS_INFO("Serial port open successful");
		}
	}
	else
	{
		ROS_WARN("Serial device not specified, only output data to stdout. If you want to output data to serial, please add device name");
	}
	Communication mainComm(n);
	mainComm.start();
	return 0;
}


