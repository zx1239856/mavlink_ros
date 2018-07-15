/*
 * SimpleSerialPort.h
 *
 *  Created on: Sep 11, 2016
 *      Author: dozen
 */

#ifndef SIMPLE_SERIALPORT_H_
#define SIMPLE_SERIALPORT_H_

#include <stdint.h>


class SimpleSerialPort {
public:
	static const unsigned int MAX_SEND_BYTES_PER_TIME = 1024;
	// Garbo Class
	class SimpleSerialPortGarbo
	{
		public:
		~SimpleSerialPortGarbo()
		{
			if(_instance)delete _instance;
		}	
	};
private:
	static SimpleSerialPort* _instance;
	static SimpleSerialPortGarbo garbo;
	static bool enabled;
	int file_descriptor;
	bool is_open;
	SimpleSerialPort();
public:
	// disable copy for singleton
	SimpleSerialPort(const SimpleSerialPort&)=delete;
	SimpleSerialPort& operator=(SimpleSerialPort&&)=delete;
	// singleton creator
	static SimpleSerialPort* getInstance();
	static bool enableSerial();
	bool openPort(const char *device_name, int baud_rate_int);
	bool closePort();
	bool isOpen();
	bool sendBytes(const uint8_t *data, uint32_t length);
	int receiveBytes(uint8_t *buffer, uint32_t max_length);

public:
	virtual ~SimpleSerialPort();
};

#endif /* SIMPLE_SERIALPORT_H_ */
