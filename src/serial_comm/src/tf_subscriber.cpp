#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "tf/tfMessage.h"
// serial port
#include "../include/mavlink/v1.0/common/mavlink.h"
#include "simple_comm/simple_serial_port.h"
#include "coordinate.h"
#include <cmath>
#include <iostream>
#include <string>

#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <map>
using namespace std;

ros::Publisher *publisher = nullptr;
/*
geometry_msgs/TransformStamped[] transforms
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string child_frame_id
  geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion rotation
      float64 x
      float64 y
      float64 z
      float64 w
 */

std::map<uint32_t, mavlink_highres_imu_t> highres_imu_map;

void Process(const tf::tfMessage::ConstPtr *_msg)
{
	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp = ros::Time::now();
	imu_msg.header.frame_id = string("laser");
	uint64_t currTime;
	//ROS_INFO("I heard: %s", msg->transforms.size());c
	//cout<<"0\n";
	//cout<<msg->transforms[0];
	//cout<<"1\n";
	// static vars used to avoid initilization repetitively
	static uint8_t serial_port_receive_buffer[1024];
	static uint8_t serial_port_send_buffer[1024];
	SimpleSerialPort *serial_port = SimpleSerialPort::getInstance();
	if (serial_port)
	{
		int received_count = serial_port->receiveBytes(serial_port_receive_buffer, 1024);
		if (received_count > 0)
		{
			mavlink_message_t msg;
			mavlink_status_t status;
			for (int i = 0; i < received_count; ++i)
			{
				if (mavlink_parse_char(MAVLINK_COMM_0, serial_port_receive_buffer[i], &msg, &status))
				{
					switch (msg.msgid)
					{
					case MAVLINK_MSG_ID_ATTITUDE:
						//we got a complete mavlink frame, continue to decode
						//"msg" is filled with mavlink frame
						//printf("received msg id = %d length=%d\n",msg.msgid,msg.len);
						mavlink_attitude_t attitude_s;
						mavlink_msg_attitude_decode(&msg, &attitude_s);
						break;
					case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
						//this is an attitude message

						//printf("attitude roll=%f pitch=%f yaw=%f\n",
						//		attitude_s.roll, attitude_s.pitch, attitude_s.yaw);
						mavlink_local_position_ned_t local_position;
						mavlink_msg_local_position_ned_decode(&msg, &local_position);
						//printf("local position x=%f  y=%f z=%f\n",
						//		local_position.x, local_position.y, local_position.z);
						break;
					case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
					{
						mavlink_vision_position_estimate_t vision_position;
						mavlink_msg_vision_position_estimate_decode(&msg, &vision_position);
						printf("params: t_x=%f  t_y=%f r_z=%f r_w=%f\n",
							   vision_position.x, vision_position.y, vision_position.pitch, vision_position.yaw);
					}
					break;
					case MAVLINK_MSG_ID_HIGHRES_IMU:
					{
						mavlink_highres_imu_t imu_data;
						mavlink_msg_highres_imu_decode(&msg, &imu_data);
						if (highres_imu_map.size() == 20)
						{
							highres_imu_map.erase(highres_imu_map.begin());
						}
						highres_imu_map[imu_data.time_usec / 1000] = imu_data;
					}
					break;
					case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
					{
						mavlink_attitude_quaternion_t att;
						mavlink_msg_attitude_quaternion_decode(&msg, &att);
						// set position
						if (!highres_imu_map.empty())
						{
							auto pos = highres_imu_map.rbegin();
							//imu_msg.header.seq = 1;
							//set angular velocity
							imu_msg.angular_velocity.x = pos->second.xgyro;
							imu_msg.angular_velocity.y = -pos->second.ygyro;
							imu_msg.angular_velocity.z = -pos->second.zgyro;
							//set acceleration
							imu_msg.linear_acceleration.x = pos->second.xacc;
							imu_msg.linear_acceleration.y = -pos->second.yacc;
							imu_msg.linear_acceleration.z = -pos->second.zacc;
							for (int i = 0; i < 9; i++)
							{
								imu_msg.orientation_covariance[i] = 0;
								imu_msg.angular_velocity_covariance[i] = 0;
								imu_msg.linear_acceleration_covariance[i] = 0;
							}
						}
						imu_msg.orientation.x = att.q2;
						imu_msg.orientation.y = -att.q3;
						imu_msg.orientation.z = -att.q4;
						imu_msg.orientation.w = att.q1;
						// construct ok, now pub it
						cout << imu_msg << endl;
						if (publisher)
							publisher->publish(imu_msg);
					}
					}
				}
			}
		}
		if (_msg)
		{
			auto data = (*_msg)->transforms[1].transform;
			Coordinate converter(data.translation.x, data.translation.y, data.translation.z, data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z);
			converter.setCoordMode(Coordinate::nedBody);
			converter.setRotMode(Coordinate::euler);
			auto rot = converter.getRotation();
			auto trans = converter.getTranslation();
			printf("NED coord:  x=%lf, y=%lf, yaw(deg)=%lf\n", trans[0], trans[1], rot[2] / M_PI * 180);
			// ROS-ENU -> PX4-NED
			if (serial_port)
			{
				mavlink_message_t msg;
				mavlink_msg_vision_position_estimate_pack(1, 200, &msg, ros::Time::now().toNSec(), trans[0],
														  trans[1], 0, 0, 0, rot[2]);
				unsigned int send_length = mavlink_msg_to_send_buffer(serial_port_send_buffer, &msg);
				serial_port->sendBytes(serial_port_send_buffer, send_length);
			}
		}
	}
}

void Callback(const tf::tfMessage::ConstPtr &_msg)
{
	Process(&_msg);
}

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
	ros::NodeHandle n2;
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
		ROS_INFO("Serial device not specified, only output data to stdout. If you want to output data to serial, please add device name");
	}
	ros::Publisher imu_pub = n2.advertise<sensor_msgs::Imu>("imu", 1000);
	publisher = &imu_pub;
	ros::Subscriber sub = n.subscribe("tf", 1000, Callback);
	while (ros::ok())
	{
		Process(nullptr);
	}
	ros::spin();
	return 0;
}
