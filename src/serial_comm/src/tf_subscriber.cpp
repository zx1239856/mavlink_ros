#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "tf/tfMessage.h"
// serial port
#include "../include/mavlink/v1.0/common/mavlink.h"
#include "simple_comm/simple_serial_port.h"
// matrix calculation
#include <cmath>
#include <iostream>
#include <string>
using namespace std;

ros::Publisher* publisher = nullptr;

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


void Callback(const tf::tfMessage::ConstPtr& _msg)
{
	sensor_msgs::Imu pubMsg;
  //ROS_INFO("I heard: %s", msg->transforms.size());c
	//cout<<"0\n";
	//cout<<msg->transforms[0];
	//cout<<"1\n";
  // static vars used to avoid initilization repetitively
  	static uint8_t serial_port_receive_buffer[1024];
		static uint8_t serial_port_send_buffer[1024];
    SimpleSerialPort* serial_port = SimpleSerialPort::getInstance();
		if(serial_port)
		{
			int received_count = serial_port->receiveBytes(serial_port_receive_buffer, 1024);
		if(received_count > 0)
		{
			mavlink_message_t msg;
			mavlink_status_t status;
			for (int i = 0; i < received_count; ++i)
			{
				if (mavlink_parse_char(MAVLINK_COMM_0, serial_port_receive_buffer[i], &msg, &status))
				{
					switch(msg.msgid)
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
									vision_position.x,vision_position.y,vision_position.pitch,vision_position.yaw);
						}
						break;
						case MAVLINK_MSG_ID_HIGHRES_IMU:
						{
							mavlink_highres_imu_t imu_data;
							mavlink_msg_highres_imu_decode(&msg,&imu_data);
							// NED => ENU here
							float xacc = imu_data.xacc;
							float yacc = -imu_data.yacc;
							float zacc = -imu_data.zacc;
							float x_ang = imu_data.xgyro; //roll
							float y_ang = -imu_data.ygyro;
							float z_ang = -imu_data.zgyro;
							/*double cy = cos(yaw * 0.5);
							double sy = sin(yaw * 0.5);
							double cr = cos(roll * 0.5);
							double sr = sin(roll * 0.5);
							double cp = cos(pitch * 0.5);
							double sp = sin(pitch * 0.5);
							double w = cy * cr * cp + sy * sr * sp;
							double x = cy * sr * cp - sy * cr * sp;
							double y = cy * cr * sp + sy * sr * cp;
							double z = sy * cr * cp - cy * sr * sp;*/
							printf("Imu data: usec=%lld, xacc=%lf, yacc=%lf, zacc=%lf\troll_spd, pitch_spd, yaw_spd=%lf, %lf, %lf\n",imu_data.time_usec,xacc,yacc,zacc,x_ang,y_ang,z_ang);

							// publish data 

    					pubMsg.header.seq = 1;
    					//pubMsg.header.stamp = 0;
    					pubMsg.header.frame_id = string("laser");
    					//set angular velocity
    					pubMsg.angular_velocity.x = x_ang;
    					pubMsg.angular_velocity.y = y_ang;
    					pubMsg.angular_velocity.z = z_ang;
    					//set acceleration
							pubMsg.linear_acceleration.x = xacc;
							pubMsg.linear_acceleration.y = yacc;
							pubMsg.linear_acceleration.z = zacc;

    					for(int i = 0; i < 9; i++){
    						pubMsg.orientation_covariance[i] = 0;
    						pubMsg.angular_velocity_covariance[i] = 0;
    						pubMsg.linear_acceleration_covariance[i] = 0;
    					}
						}
						case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
						{
							mavlink_attitude_quaternion_t attitude;
							mavlink_msg_attitude_quaternion_decode(&msg,&attitude);
							// set position

							// (w,x,y,z) <= (x,-y,-z,w) 
							pubMsg.orientation.x = -attitude.q3;
    					pubMsg.orientation.y = -attitude.q4;
    					pubMsg.orientation.z = attitude.q1;
   				  	pubMsg.orientation.w = attitude.q2;

							// construct ok, now pub it
    					cout<<pubMsg<<endl;
    					if(publisher)publisher->publish(pubMsg);
						}
					}
				}
			}
		}
		auto data = _msg->transforms[1].transform;

		/** 
		 * ENU in ROS => NED in PX4 coordinate frame conversion guidance 
		 * 
		 * (x,y,z) => (x,-y,-z)
		 * (w,x,y,z) => (x,-y,-z,w) 
		 * (roll,pitch,yaw) => (roll,-pitch,-yaw)
		 */
		/*** 
		 * quaternion to euler angle
		 * 
		 * here we only need yaw for the laser */
		//double roll,pitch,yaw;
		//roll = atan2(2*(data.rotation.w*data.rotation.x+data.rotation.y*data.rotation.z),(1-2*(data.rotation.x*data.rotation.x+data.rotation.y*data.rotation.y)));
		//pitch = asin(2*(data.rotation.w*data.rotation.y-data.rotation.x*data.rotation.z));
		double yaw = atan2(2*(data.rotation.w*data.rotation.z+data.rotation.x*data.rotation.y),(1-2*(data.rotation.z*data.rotation.z+data.rotation.y*data.rotation.y)));
		printf("NED coord:  x=%lf, y=%lf, yaw(deg)=%lf\n",data.translation.x,-data.translation.y,-yaw/M_PI*180);
		// ROS-ENU -> PX4-NED
		if(serial_port)
		{
			mavlink_message_t msg;
			mavlink_msg_vision_position_estimate_pack(1,200, &msg, ros::Time::now().toNSec(), data.translation.x,
			-data.translation.y, 0, 0, 0, -yaw);
			unsigned int send_length = mavlink_msg_to_send_buffer(serial_port_send_buffer,&msg);
			serial_port->sendBytes(serial_port_send_buffer, send_length);
		}
	}
}

int main(int argc, char **argv)
{
	std::string devName;
	if(argc!=1 && argc!=2)
	{
		ROS_ERROR("Incorrect command. Arguments should contain only serial device name");
	}
	else
	{
		if(argc==2)
		{
			devName = "/dev/"+std::string(argv[1]);
			SimpleSerialPort::enableSerial();
		}
	}
  ros::init(argc, argv, "tf_subscriber");
  ros::NodeHandle n;
	ros::NodeHandle n2;
  // initialize serial port
  SimpleSerialPort* serial_port = SimpleSerialPort::getInstance();
  if(serial_port)
	{
		serial_port->openPort(devName.c_str(), 921600);
  	if(!serial_port->isOpen())
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
  ros::spin();
  return 0;
}
