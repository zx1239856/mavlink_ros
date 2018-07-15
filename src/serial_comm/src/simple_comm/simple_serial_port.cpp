/*
 * SimpleSerialPort.cpp
 *
 *  Created on: Sep 11, 2016
 *      Author: dozen
 */

#include "simple_serial_port.h"

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

using namespace std;

SimpleSerialPort::SimpleSerialPortGarbo SimpleSerialPort::garbo;
SimpleSerialPort* SimpleSerialPort::_instance = nullptr;
bool SimpleSerialPort::enabled = false;

bool SimpleSerialPort::enableSerial()
{
	if(enabled)return false;
	else
	{
		enabled=true;
		return enabled;
	}
}

SimpleSerialPort* SimpleSerialPort::getInstance()
{
	if(enabled && _instance==nullptr)_instance=new SimpleSerialPort();
	return _instance;
}

SimpleSerialPort::SimpleSerialPort() {
	// TODO Auto-generated constructor stub
	file_descriptor = -1;
	is_open = false;
}

SimpleSerialPort::~SimpleSerialPort() {
	// TODO Auto-generated destructor stub

	closePort();
}


bool SimpleSerialPort::openPort(const char *device_name, int baud_rate_int)
{
	if(is_open)
	{
		closePort();
	}
	/* Open File Descriptor */
	//int serial_port = open(portName.c_str(), O_RDWR| O_NONBLOCK | O_NDELAY );
	file_descriptor = open(device_name, O_RDWR| O_NDELAY | O_NOCTTY);

	/* Error Handling */
	if (file_descriptor < 0 )
	{
		return false;
	}

	/* *** Configure Port *** */
	struct termios tty;
	memset (&tty, 0, sizeof tty);

	/* Error Handling */
	if ( tcgetattr ( file_descriptor, &tty ) != 0 )
	{
		goto fail;
	}


	/* Set Baud Rate */
	speed_t baud_rate_speed;
	switch(baud_rate_int)
	{
	case 1500000:
		baud_rate_speed = B1500000;
		break;
	case 921600:
		baud_rate_speed = B921600;
		break;
	case 460800:
		baud_rate_speed = B460800;
		break;
	case 230400:
		baud_rate_speed = B230400;
		break;
	case 115200:
		baud_rate_speed = B115200;
		break;
	case 57600:
		baud_rate_speed = B57600;
		break;
	case 38400:
		baud_rate_speed = B38400;
		break;
	case 19200:
		baud_rate_speed = B19200;
		break;
	case 9600:
		baud_rate_speed = B9600;
		break;
	case 4800:
		baud_rate_speed = B4800;
		break;
	case 2400:
		baud_rate_speed = B2400;
		break;
	case 1800:
		baud_rate_speed = B1800;
		break;
	case 1200:
		baud_rate_speed = B1200;
		break;
	default:
		baud_rate_speed = B57600;
	}


	cfsetospeed (&tty, baud_rate_speed);
	cfsetispeed (&tty, baud_rate_speed);

	/* Setting other Port Stuff */
	tty.c_cflag     &=  ~PARENB;        // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;
	tty.c_cflag     &=  ~CRTSCTS;       // no flow control
	tty.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
	tty.c_oflag     =   0;                  // no remapping, no delays
	tty.c_cc[VMIN]      =   0;                  // read doesn't block
	tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout

	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
	tty.c_iflag 	= 0;
	//tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
	//tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	tty.c_lflag     = 0;
	tty.c_oflag     &=  ~OPOST;              // make raw

	/* Flush Port, then applies attributes */
	tcflush( file_descriptor, TCIFLUSH );

	if ( tcsetattr ( file_descriptor, TCSANOW, &tty ) != 0)
	{
		goto fail;
	}

	is_open = true;
	return true;


fail:
	close(file_descriptor);
	return false;

}

bool SimpleSerialPort::closePort()
{
	if(is_open)
	{
		close(file_descriptor);
		is_open = false;
	}
	return true;
}
bool SimpleSerialPort::isOpen()
{
	return is_open;
}


bool SimpleSerialPort::sendBytes(const uint8_t *data, uint32_t length)
{
	if(!is_open)
	{
		return false;
	}
	unsigned int bytes_sent = 0;
	while(bytes_sent < length)
	{
		unsigned int bytes_should_sent = length-bytes_sent;
		if(bytes_should_sent > MAX_SEND_BYTES_PER_TIME)
		{
			bytes_should_sent = MAX_SEND_BYTES_PER_TIME;
		}
		ssize_t bytes_written = write(file_descriptor, data, bytes_should_sent);
		if(bytes_written <= 0)
		{
			return false;
		}
		bytes_sent += bytes_written;
	}

	return true;
}
int SimpleSerialPort::receiveBytes(uint8_t *buffer, uint32_t max_length)
{
	if(!is_open)
	{
		return -1;
	}

	int n = read( file_descriptor, buffer , max_length);

	return n;
}

