/**
* nethandler.h
* @author Zhang Xiang
* @description 
* @created Sun Sep 30 2018 10:17:45 GMT+0800 (CST)
* @license MIT
* @copyright All rights reserved, 2018
* @last-modified Sun Sep 30 2018 10:53:02 GMT+0800 (CST)
*/

#pragma once
#include "ros/ros.h"
#include "../io/iowrapper.h"

// Subscribe /mavReadRaw and send it to UDP
// receive UDP and publish it to /mavWriteRaw
class netHandler
{
  protected:
    ioWrapper *_io;
    ros::NodeHandle _node;
    ros::Subscriber _sub;
    ros::Publisher _pub;
  public:
    netHandler(ioWrapper *wrapper):_io(wrapper){}
    virtual void run() = 0;
    virtual ~netHandler() = 0;
};