/**
* udpnode_main.cpp
* @author Zhang Xiang
* @description 
* @created Sun Sep 30 2018 10:09:53 GMT+0800 (CST)
* @license MIT
* @copyright All rights reserved, 2018
* @last-modified Sun Sep 30 2018 11:19:34 GMT+0800 (CST)
*/

#include "ros/ros.h"
#include <iostream>
#include <string>
#include "handler/udpnethandler.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "udp_node");
    udpNetHandler handler(14556);
    handler.run();
	return 0;
}