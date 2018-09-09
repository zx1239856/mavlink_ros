#include "communication.h"
#include "../include/mavlink/v1.0/common/mavlink.h"
#include "coordinate.h"

#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <tf/tfMessage.h>

#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <cmath>

using namespace std;
using namespace Eigen;

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

uint8_t Communication::serial_port_receive_buffer[1024];
uint8_t Communication::serial_port_send_buffer[1024];
uint8_t Communication::udp_receive_buffer[1024];
uint8_t Communication::udp_send_buffer[1024];

void Communication::Callback(const tf::tfMessage::ConstPtr &_msg)
{
	Process(&_msg);
}

void Communication::Process(const tf::tfMessage::ConstPtr *tf_msg)
{
	// static vars used to avoid initilization repetitively
	serial_port = SimpleSerialPort::getInstance();
	if (!serial_port)
	{
		ROS_WARN("serial port open fail, returning");
		return;
	}
	// receive
	receive_mavlink_send_imu();
	// receive
	send_tf_msg(tf_msg);
}
void Communication::print_local_position(mavlink_local_position_ned_t &local_position)
{
	//ROS_INFO()
	cout << "local position x" << local_position.x << endl;
	cout << "local position y" << local_position.y << endl;
	cout << "local position z" << local_position.z << endl;
}
void Communication::receive_mavlink_send_imu()
{
	// init imu_topic_msg
	sensor_msgs::Imu imu_topic_msg;
	imu_topic_msg.header.stamp = ros::Time::now();
	imu_topic_msg.header.frame_id = string("laser");
	//uint64_t currTime;

	Coordinate coor(0, 0, 0, 0);
	// receive_msg from imu
	serial_port = SimpleSerialPort::getInstance();
	if (!serial_port)
	{
		ROS_WARN("serial port open fail, returning");
		return;
	}

    // UDP => serial
	int received_udp_cnt = udpHandler->receive(udp_receive_buffer, 1024);
	mavlink_message_t mavlink_msg;
	mavlink_status_t status;
	for (int i = 0; i < received_udp_cnt; ++i)
	{
		if(mavlink_parse_char(MAVLINK_COMM_0,udp_receive_buffer[i],&mavlink_msg,&status))
		{
			unsigned int len = mavlink_msg_to_send_buffer((unsigned char*)serial_port_send_buffer,&mavlink_msg);
			if(serial_port)
			{
				serial_port->sendBytes(serial_port_send_buffer,len);
			}
		}
	}

	// serial readout

	int received_count = serial_port->receiveBytes(Communication::serial_port_receive_buffer, 1024);
	//int received_count = serial_port->receiveBytes(serial_port_receive_buffer, 1024);

	for (int i = 0; i < received_count; ++i)
	{
		mavlink_message_t mavlink_msg;
		mavlink_status_t status;

		if (mavlink_parse_char(MAVLINK_COMM_0, Communication::serial_port_receive_buffer[i], &mavlink_msg, &status) == 0)
		//if (mavlink_parse_char(MAVLINK_COMM_0, Communication::serial_port_receive_buffer[i], &mavlink_msg, &status))
		{
			//ROS_INFO("mavline cannot parse char, receiver returning ...");
			continue;
		}

		unsigned int len = mavlink_msg_to_send_buffer(udp_send_buffer,&mavlink_msg);
		udpHandler->send(udp_send_buffer,len);

		//ROS_INFO("mavlink parsed char");
		switch (mavlink_msg.msgid)
		{
		case MAVLINK_MSG_ID_ATTITUDE:
			//we got a complete mavlink frame, continue to decode
			//"mavlink_msg" is filled with mavlink frame
			//cout<<"MAVLINK_MSG_ID_ATTITUDE"<<endl;
			printf("received mavlink_msg id = %d length=%d\n", mavlink_msg.msgid, mavlink_msg.len);
			mavlink_attitude_t attitude_s;
			mavlink_msg_attitude_decode(&mavlink_msg, &attitude_s);
			break;
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			//this is an attitude message
			//cout<<"MAVLINK_MSG_ID_LOCAL_POSITION_NED"<<endl;
			mavlink_local_position_ned_t local_position;
			mavlink_msg_local_position_ned_decode(&mavlink_msg, &local_position);
			local_x = local_position.x;
			local_y = local_position.y;
			local_z = local_position.z;
			print_local_position(local_position);
			break;
		case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
			mavlink_vision_position_estimate_t vision_position;
			mavlink_msg_vision_position_estimate_decode(&mavlink_msg, &vision_position);
			printf("Vision_position_estimate: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f\n",
				   vision_position.x, vision_position.y, vision_position.z, vision_position.roll, vision_position.pitch, vision_position.yaw);
			break;
		case MAVLINK_MSG_ID_HIGHRES_IMU:
			//cout<<"MAVLINK_MSG_ID_HIGHRES_IMU"<<endl;
			mavlink_highres_imu_t imu_data;
			mavlink_msg_highres_imu_decode(&mavlink_msg, &imu_data);
			if (highres_imu_map.size() == 20)
			{
				highres_imu_map.erase(highres_imu_map.begin());
			}
			highres_imu_map[imu_data.time_usec / 1000] = imu_data;
			break;
		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
			//	cout<<"MAVLINK_MSG_ID_ATTITUDE_QUATERNION"<<endl;
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
			imu_pub.publish(imu_topic_msg);
			break;
		case MAVLINK_MSG_ID_HEARTBEAT:
			//cout<<"MAVLINK_MSG_ID_HEARTBEAT"<<endl;
			mavlink_heartbeat_t packet;
			mavlink_msg_heartbeat_decode(&mavlink_msg, &packet);
			uint8_t custom_mode_main_mode = (packet.custom_mode >> 16) & 0xff;
			uint8_t custom_mode_sub_mode = (packet.custom_mode >> 24) & 0xff;
			static const int PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
			std_msgs::Int32 offboard_mode_msg;
			if (custom_mode_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD)
			{
				offboard_mode_msg.data = 1;
			}
			else
			{
				offboard_mode_msg.data = 0;
			}
			offboard_mode_pub.publish(offboard_mode_msg);
			break;
			/*default:
			break;*/
		}
	}
}

void Communication::send_tf_msg(const tf::tfMessage::ConstPtr *tf_msg)
{
	serial_port = SimpleSerialPort::getInstance();
	if (tf_msg)
	{
		auto transforms = (*tf_msg)->transforms;
		auto data_frame = string("undefined");
		auto data = (*tf_msg)->transforms[1].transform;

		for (auto it = transforms.begin(); it != transforms.end(); it++)
		{
			cout << "frame id of tf_msg : " << (*it).child_frame_id << endl;
			if ((*it).child_frame_id == string("laser"))
			{
				data = (*it).transform;
				data_frame = string("laser");
			}
		}
		/*if(unlikely((*tf_msg)->transforms[0].header.frame_id=="laser"))
		{
			data = (*tf_msg)->transforms[0].transform;
		}*/

		if (data_frame != string("laser"))
		{
			return;
		}

		double w = data.rotation.w;
		double x = data.rotation.x;
		double y = data.rotation.y;
		double z = data.rotation.z;

		MatrixXd rot(3, 3);
		rot(0, 0) = 1 - 2 * y * y - 2 * z * z;
		rot(0, 1) = 2 * (x * y - w * z);
		rot(0, 2) = 2 * (x * z + w * y);
		rot(1, 0) = 2 * (x * y + w * z);
		rot(1, 1) = 1 - 2 * x * x - 2 * z * z;
		rot(1, 2) = 2 * (y * z - w * x);
		rot(2, 0) = 2 * (x * z - w * y);
		rot(2, 1) = 2 * (y * z + w * x);
		rot(2, 2) = 1 - 2 * x * x - 2 * y * y;
		//cout << "rot matrix" << rot << endl;
		MatrixXd result(3, 3);
		result(0, 0) = rot(1, 0);
		result(0, 1) = -rot(1, 1);
		result(0, 2) = -rot(1, 2);
		result(1, 0) = rot(0, 0);
		result(1, 1) = -rot(0, 1);
		result(1, 2) = -rot(0, 2);
		result(2, 0) = -rot(2, 0);
		result(2, 1) = rot(2, 1);
		result(2, 2) = rot(2, 2);

		double roll = atan2(result(2, 1), result(2, 2));
		double pitch = atan2(-result(2, 0), sqrt(result(2, 1) * result(2, 1) + result(2, 2) * result(2, 2)));
		double yaw = atan2(result(1, 0), result(0, 0));
		local_yaw = yaw;
		// the following lines should be double checked !!!!!
		/*	Coordinate coor(data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z);
		coor.setCoordMode(Coordinate::nedLocal);
		double roll = coor.getRotation(Coordinate::euler)[0];
		double pitch = coor.getRotation(Coordinate::euler)[1];
		double yaw = coor.getRotation(Coordinate::euler)[2];*/
		//cout<<test_frame<<endl;
		//send target position
		if (serial_port)
		{
			if (abs(source_x + delta_x - local_x) < 0.02)
			{
				delta_x = 0;
			}
			if (abs(source_y + delta_y - local_y) < 0.02)
			{
				delta_y = 0;
			}
			if (abs(source_z + delta_z - local_z) < 0.02)
			{
				delta_z = 0;
			}
			if (abs(source_yaw + delta_yaw - local_yaw) < 0.02)
			{
				delta_yaw = 0;
			}
			mavlink_message_t msg2;
			const uint16_t TYPE_MASK_XYZ_YAW = 0b0000100111111000;
			mavlink_msg_set_position_target_local_ned_pack(1, 200, &msg2, ros::Time::now().toNSec(), 0, 0, 1, TYPE_MASK_XYZ_YAW,
														   local_x + delta_x, local_y + delta_y, local_z + delta_z, 0, 0, 0, 0, 0, 0, yaw + delta_yaw, 0);

			unsigned int send_length2 = mavlink_msg_to_send_buffer(Communication::serial_port_send_buffer, &msg2);
			serial_port->sendBytes(Communication::serial_port_send_buffer, send_length2);

			static tf::TransformBroadcaster br;
			tf::Transform transform;
			transform.setOrigin(tf::Vector3(local_y + delta_y, local_x + delta_x, local_z));
			transform.setRotation(tf::Quaternion(data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "target"));

			cout << "target send" << endl;
		}
		// send vision position estimate

		ROS_INFO("Attempting to send data to serial: x=%lf, y=%lf, z=%lf, (DEG)roll=%lf, pitch=%lf, yaw=%lf\n\n", data.translation.y, data.translation.x, -data.translation.z, roll, pitch, yaw);
		// ROS-ENU -> PX4-NED
		if (serial_port)
		{
			mavlink_message_t msg;
			mavlink_msg_vision_position_estimate_pack(1, 200, &msg, ros::Time::now().toNSec(),
													  data.translation.y, data.translation.x, -data.translation.z, roll, pitch, yaw);
			unsigned int send_length = mavlink_msg_to_send_buffer(Communication::serial_port_send_buffer, &msg);
			//unsigned int send_length = mavlink_msg_to_send_buffer(serial_port_send_buffer, &msg);
			serial_port->sendBytes(Communication::serial_port_send_buffer, send_length);
			//serial_port->sendBytes(serial_port_send_buffer, send_length);
			cout << "estimate send" << endl;
		}
	}
}

void Communication::Callback_v(const serial_comm::velocity::ConstPtr &_msg)
{
	if (_msg)
	{
		source_x = local_x;
		source_y = local_y;
		source_z = local_z;
		source_yaw = local_yaw;
		delta_x = _msg->v_x;
		delta_y = _msg->v_y;
		if (_msg->reverse)
			delta_yaw = 3.141592535;
		else
			delta_yaw = 0;
	}
}
/*
void Communication::Callback_ctrl(const std_msgs::Int32::ConstPtr &keyboard_msg){
	if(abs(source_x + delta_x - local_x) < 0.02){
		delta_x=0;
	}
	if(abs(source_y + delta_y - local_y) < 0.02){
		delta_y=0;
	}
	if(abs(source_z + delta_z - local_z) < 0.02){
		delta_z=0;
	}
	if(abs(source_yaw + delta_yaw - local_yaw) < 0.02){
		delta_yaw=0;
	}

	if(!keyboard_msg)  return;
	// there are msgs
	source_x = local_x;
	source_y = local_y;
	source_z = local_z;
	source_yaw = local_yaw;
	// QUESTION MARK????????? why does y increase when it heads forward????
	switch(keyboard_msg->data){
	// w forward
		case 0:
			delta_y=0.2;
			break;
	// s backward
		case 1:
			delta_y=-0.2;
			break;
	// a left
		case 2:
			delta_x=0.2;
			break;
	// d right
		case 3:
			delta_x=-0.2;
			break;
	// e up
		case 4:
			delta_z=-0.2;
			break;
	// r down
		case 5:
			delta_z=0.2;
			break;
	// o counterclockwise
		case 6:
			delta_yaw=0.2;
			break;
	// p clockwise
		case 7:	
			delta_yaw=-0.2;
			break;
	// x no move
		case 8:
			delta_x=0;
			delta_y=0;
			delta_z=0;
			delta_yaw=0;
			break;
	}

}*/
