#include "rosnodehandler.h"
#include "../utils/coordinate.h"
#include <exception>

uint8_t rosNodeHandler::serial_port_receive_buffer[1024];
uint8_t rosNodeHandler::serial_port_send_buffer[1024];
uint8_t rosNodeHandler::publish_buffer[1024];

rosNodeHandler::rosNodeHandler(serialWrapper *sp) : serial(sp)
{
}

void rosNodeHandler::initializeHandler()
{
    using namespace handler;
    _tfSub = _node.subscribe(tfTopic, 1 << 10, &rosNodeHandler::tfCallback, this);
    _mavSub = _node.subscribe(mavWriteTopic, 1 << 10, &rosNodeHandler::mavCallback, this);
    _imuPub = _node.advertise<sensor_msgs::Imu>(imuTopic, 1 << 10);
    _mavPub = _node.advertise<std_msgs::UInt8MultiArray>(mavReadTopic, 1 << 10);
    _offboardModePub = _node.advertise<std_msgs::Int32>(offBoardMsgTopic, 1 << 10);
}

void rosNodeHandler::run()
{
    initializeHandler();
    if (!serial)
    {
        ROS_ERROR("Serial port not specified. Exit.");
        exit(EXIT_FAILURE);
    }
    try
    {
        if (!serial->openPort())
        {
            ROS_ERROR("Serial port open failed. Exit");
            exit(EXIT_FAILURE);
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Serial port open failed. %s", e.what());
        exit(EXIT_FAILURE);
    }
    ROS_INFO("Start streaming serial info...");
    bool flag = true;
    ros::Rate loopRate(100);
    while (ros::ok() && flag)
    {
        try
        {
            processSerial();
        }
        catch (std::exception &e)
        {
            ROS_INFO("%s", e.what());
            flag = false;
        }
        ros::spinOnce();
        loopRate.sleep();
    }
}

// main-stuff, process serial, and pub to mavReadRaw
void rosNodeHandler::processSerial()
{
    using namespace std;
    using namespace handler;
    // init imu_topic_msg
    sensor_msgs::Imu imu_topic_msg;
    imu_topic_msg.header.stamp = ros::Time::now();
    imu_topic_msg.header.frame_id = string(imuFrameId);
    //uint64_t currTime;

    Coordinate coor(0, 0, 0, 0);

    size_t readLen = serial->read(serial_port_receive_buffer, 1024);
    for (int i = 0; i < readLen; ++i)
    {
        mavlink_message_t mavlink_msg;
        mavlink_status_t status;

        if (mavlink_parse_char(MAVLINK_COMM_0, serial_port_receive_buffer[i], &mavlink_msg, &status) == 0)
        {
            continue;
        }
        unsigned int len = mavlink_msg_to_send_buffer(publish_buffer, &mavlink_msg);
        std_msgs::UInt8MultiArray mmsg;
        mmsg.data.resize(len);
        for (int i = 0; i < len; ++i)
        {
            mmsg.data[i] = publish_buffer[i];
        }
        _mavPub.publish(mmsg);
        switch (mavlink_msg.msgid)
        {
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            mavlink_local_position_ned_t local_position;
            mavlink_msg_local_position_ned_decode(&mavlink_msg, &local_position);
            break;
        case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
            mavlink_vision_position_estimate_t vision_position;
            mavlink_msg_vision_position_estimate_decode(&mavlink_msg, &vision_position);
            printf("Vision_position_estimate: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f\n",
                   vision_position.x, vision_position.y, vision_position.z, vision_position.roll, vision_position.pitch, vision_position.yaw);
            break;
        case MAVLINK_MSG_ID_HIGHRES_IMU:
            mavlink_highres_imu_t imu_data;
            mavlink_msg_highres_imu_decode(&mavlink_msg, &imu_data);
            if (highres_imu_map.size() == 20)
            {
                highres_imu_map.erase(highres_imu_map.begin());
            }
            highres_imu_map[imu_data.time_usec / 1000] = imu_data;
            break;
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            mavlink_attitude_quaternion_t att;
            mavlink_msg_attitude_quaternion_decode(&mavlink_msg, &att);
            // set position
            coor.setRotation((double)att.q1, (double)att.q2, (double)att.q3, (double)att.q4);

            if (!highres_imu_map.empty())
            {
                auto pos = highres_imu_map.rbegin();
                // set angular position and acceleration
                coor.setGyro(pos->second.xgyro, pos->second.ygyro, pos->second.zgyro);
                coor.setAcc(pos->second.xacc, pos->second.yacc, pos->second.zacc);
            }
            coor.setCoordMode(Coordinate::nedBody);
            coor.assign(imu_topic_msg);
            // construct ok, now pub it
            _imuPub.publish(imu_topic_msg);
            break;
        default:
            // do nothing here
            break;
        }
    }
}

// grab data from mavMsg and send to serial
void rosNodeHandler::mavCallback(const std_msgs::UInt8MultiArray::ConstPtr &mavMsg)
{
    auto data = mavMsg->data;
    for (int i = 0; i < data.size(); ++i)
    {
        mavlink_message_t mavlink_msg;
        mavlink_status_t status;
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &mavlink_msg, &status) == 0)
        {
            continue;
        }
        (*serial) << data;
        break;
    }
}

void rosNodeHandler::tfCallback(const tf::tfMessage::ConstPtr &tf_msg)
{
    using namespace std;
    if (tf_msg)
    {
        auto transforms = (*tf_msg).transforms;
        auto data_frame = string("undefined");
        auto data = (*tf_msg).transforms[1].transform;

        for (auto it = transforms.begin(); it != transforms.end(); it++)
        {
            //cout << "frame id of tf_msg : " << (*it).child_frame_id << endl;
            if ((*it).child_frame_id == string("laser"))
            {
                data = (*it).transform;
                data_frame = string("laser");
            }
        }

        if (data_frame != string("laser"))
        {
            return;
        }

        double w = data.rotation.w;
        double x = data.rotation.x;
        double y = data.rotation.y;
        double z = data.rotation.z;

        std::vector<std::vector<double>> rot;
        rot[0][0] = 1 - 2 * y * y - 2 * z * z;
        rot[0][1] = 2 * (x * y - w * z);
        rot[0][2] = 2 * (x * z + w * y);
        rot[1][0] = 2 * (x * y + w * z);
        rot[1][1] = 1 - 2 * x * x - 2 * z * z;
        rot[1][2] = 2 * (y * z - w * x);
        rot[2][0] = 2 * (x * z - w * y);
        rot[2][1] = 2 * (y * z + w * x);
        rot[2][2] = 1 - 2 * x * x - 2 * y * y;
        //cout << "rot matrix" << rot << endl;
        std::vector<std::vector<double>> result;
        result[0][0] = rot[1][0];
        result[0][1] = -rot[1][1];
        result[0][2] = -rot[1][2];
        result[1][0] = rot[0][0];
        result[1][1] = -rot[0][1];
        result[1][2] = -rot[0][2];
        result[2][0] = -rot[2][0];
        result[2][1] = rot[2][1];
        result[2][2] = rot[2][2];

        double roll = atan2(result[2][1], result[2][2]);
        double pitch = atan2(-result[2][0], sqrt(result[2][1] * result[2][1] + result[2][2] * result[2][2]));
        double yaw = atan2(result[1][0], result[0][0]);
        ROS_INFO("Attempting to send data to serial: x=%lf, y=%lf, z=%lf, (DEG)roll=%lf, pitch=%lf, yaw=%lf\n\n", data.translation.y, data.translation.x, -data.translation.z, roll, pitch, yaw);
        // ROS-ENU -> PX4-NED
        mavlink_message_t msg;
        mavlink_msg_vision_position_estimate_pack(1, 200, &msg, ros::Time::now().toNSec(),
                                                  data.translation.y, data.translation.x, -data.translation.z, roll, pitch, yaw);
        unsigned int send_length = mavlink_msg_to_send_buffer(serial_port_send_buffer, &msg);
        serial->send(serial_port_send_buffer, send_length);
        cout << "estimate send" << endl;
    }
}