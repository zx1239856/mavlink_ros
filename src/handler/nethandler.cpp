/**
* nethandler.cpp
* @author Zhang Xiang
* @description 
* @created Sun Sep 30 2018 10:45:16 GMT+0800 (CST)
* @license MIT
* @copyright All rights reserved, 2018
* @last-modified Sun Sep 30 2018 10:52:46 GMT+0800 (CST)
*/

#include "nethandler.h"
#include <std_msgs/UInt8MultiArray.h>
#include "rosconstant.h"

netHandler::~netHandler()
{
    if(_io)delete _io;
}