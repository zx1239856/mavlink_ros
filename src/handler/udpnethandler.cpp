/**
* udpnethandler.cpp
* @author Zhang Xiang
* @description 
* @created Sun Sep 30 2018 10:36:31 GMT+0800 (CST)
* @license MIT
* @copyright All rights reserved, 2018
* @last-modified Sun Sep 30 2018 11:33:28 GMT+0800 (CST)
*/

#include "udpnethandler.h"
#include "../io/udpwrapper.h"
#include "../include/mavlink/v1.0/common/mavlink.h"
#include "rosconstant.h"

uint8_t udpNetHandler::udp_receive_buffer[1024];

udpNetHandler::udpNetHandler(ushort port) : netHandler(new udpWrapper(port))
{
}

void udpNetHandler::sendData(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
    std::vector<uint8_t> data = msg->data;
    (*_io) << data;
}

void udpNetHandler::run()
{
    initializeHandler();
    ROS_INFO("PX4 UDP forwarding enabled...");
    while (ros::ok())
    {
        // read from udp
        uint received_udp_cnt = _io->read(udp_receive_buffer,1024);
        mavlink_message_t mavlink_msg;
        mavlink_status_t status;
        for (int i = 0; i < received_udp_cnt; ++i)
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, udp_receive_buffer[i], &mavlink_msg, &status))
            {
                std_msgs::UInt8MultiArray mmsg;
                mmsg.data.resize(i+1);
                for(int j=0;j<=i;++j)
                {
                    mmsg.data[j] = udp_receive_buffer[j];
                }
                _pub.publish(mmsg);
                break;
            }
        }
        // ok, now it will automatically send data to udp
        ros::spinOnce();
    }
}

void udpNetHandler::initializeHandler()
{
    using namespace handler;
    _pub = _node.advertise<std_msgs::UInt8MultiArray>(mavWriteTopic, 1 << 10);
    _sub = _node.subscribe(mavReadTopic, 1 << 10, &udpNetHandler::sendData, this);
}