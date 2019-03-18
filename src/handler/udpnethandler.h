/**
* udpnethandler.h
* @author Zhang Xiang
* @description 
* @created Sun Sep 30 2018 10:32:16 GMT+0800 (CST)
* @license MIT
* @copyright All rights reserved, 2018
* @last-modified Sun Sep 30 2018 11:00:27 GMT+0800 (CST)
*/

#pragma once
#include "nethandler.h"
#include <std_msgs/UInt8MultiArray.h>

class udpNetHandler : public netHandler
{
  private:
    static uint8_t udp_receive_buffer[1024];
    void initializeHandler();
    void sendData(const std_msgs::UInt8MultiArray::ConstPtr &msg);
  public: 
    udpNetHandler(ushort localPort);
    void run() override;
};