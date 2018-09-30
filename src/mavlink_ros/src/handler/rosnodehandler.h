#pragma once
#include "rosconstant.h"
#include <ros/ros.h>
#include <map>
#include "../include/mavlink/v1.0/common/mavlink.h"
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

#include "../io/serialwrapper.h"

class rosNodeHandler
{
    private:
    ros::NodeHandle _node;
    ros::Subscriber _mavSub;
    ros::Subscriber _tfSub;
    ros::Publisher _mavPub;
    ros::Publisher _imuPub;
    ros::Publisher _offboardModePub;
	static uint8_t serial_port_receive_buffer[1024];
	static uint8_t serial_port_send_buffer[1024];
    static uint8_t publish_buffer[1024];
    std::map<uint32_t, mavlink_highres_imu_t> highres_imu_map;
    serialWrapper *serial = nullptr;
    // handlers
    void tfCallback(const tf::tfMessage::ConstPtr &msg);
    void mavCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
    protected:
    void initializeHandler();
    // read from serial and process
    void processSerial();
    public:
    rosNodeHandler(serialWrapper *sp); 
    rosNodeHandler(const rosNodeHandler&)=delete;
    rosNodeHandler& operator=(const rosNodeHandler&)=delete;
    void run();
};