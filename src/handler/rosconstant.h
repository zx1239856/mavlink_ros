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
    constexpr double visionEstimationPeriodSec = 0.03; 
    constexpr char mapFrameId[] = "map";
    constexpr char robotFrameId[] = "base_link";
    constexpr char imuTopic[] = "imu";
    constexpr char imuFrameId[] = "imu_link";
    constexpr char mavWriteTopic[] = "mavWriteRaw";
    constexpr char mavReadTopic[] = "mavReadRaw"; 
    constexpr char offBoardMsgTopic[] = "offboard_mode_msg";
}