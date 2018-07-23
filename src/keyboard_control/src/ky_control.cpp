#include "ky_control.h"
#include <termio.h>

void KyControl::Callback(const std_msgs::Int32::ConstPtr& msg){
	cout << "get message "<<msg->data<<endl;
	if(msg->data == 1){
		if(!is_offboard){
			cout << "Under the Offboard Mode." << endl;
			is_offboard=true;
		}
	}
	else if(msg->data == 0){
		if(is_offboard){
			cout << "Exit the Offboard Mode." << endl;
			is_offboard=false;
		}
	}
}

int KyControl::scanKeyboard()
{
	int in;
	struct termios new_settings;
	struct termios stored_settings;
	tcgetattr(0,&stored_settings);
	new_settings = stored_settings;
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VTIME] = 0;
	tcgetattr(0,&stored_settings);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0,TCSANOW,&new_settings);
 
	in = getchar();
 
	tcsetattr(0,TCSANOW,&stored_settings);
	return in;
}

void KyControl::run(){
	std_msgs::Int32 _msg;
	while(true){
		ros::spinOnce();
		if(true){	
			switch((char)scanKeyboard()){
				case 'w':
					cout << "forward" <<endl;
					_msg.data=0;
					keyboard_control_pub.publish(_msg);
					break;
				case 's':
					cout << "backward" <<endl;
					_msg.data=1;
					keyboard_control_pub.publish(_msg);
					break;
				case 'a':
					cout << "left" <<endl;
					_msg.data=2;
					keyboard_control_pub.publish(_msg);
					break;
				case 'd':
					cout << "right" <<endl;
					_msg.data=3;
					keyboard_control_pub.publish(_msg);
					break;
				case 'e':
					cout << "up" <<endl;
					_msg.data=4;
					keyboard_control_pub.publish(_msg);
					break;
				case 'r':
					cout << "down" <<endl;
					_msg.data=5;
					keyboard_control_pub.publish(_msg);
					break;
				case 'o':
					cout << "anticlockwise" <<endl;
					_msg.data=6;
					keyboard_control_pub.publish(_msg);
					break;
				case 'p':
					cout << "clockwise" <<endl;
					_msg.data=7;
					keyboard_control_pub.publish(_msg);
					break;
				case 'x':
					cout << "no move" <<endl;
					_msg.data=8;
					keyboard_control_pub.publish(_msg);
					break;
				case 'v':
					return ;
			}
		
		}
	
	}
}



