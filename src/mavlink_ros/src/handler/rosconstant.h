/**
* rosconstant.h
* @author Zhang Xiang
* @description 
* @created Sun Sep 30 2018 01:00:42 GMT+0800 (CST)
* @license MIT
* @copyright All rights reserved, 2018
* @last-modified Sun Sep 30 2018 10:00:17 GMT+0800 (CST)
*/

#pragma once

namespace handler
{
    const char tfTopic[]= "tf";
    const char imuTopic[] = "imu";
    const char imuFrameId[] = "imu_link";
    const char mavWriteTopic[] = "mavWriteRaw";
    const char mavReadTopic[] = "mavReadRaw"; 
    const char offBoardMsgTopic[] = "offboard_mode_msg";
}