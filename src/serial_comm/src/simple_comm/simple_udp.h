/*
 * SimpleUDP.h
 *
 *  Created on: Oct 21, 2016
 *      Author: dozen
 */

#ifndef CMAKE_LIBS_SIMPLE_UDP_H_
#define CMAKE_LIBS_SIMPLE_UDP_H_

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <stdint.h>


class SimpleUDP {
private:
	int sock_fd;
	struct sockaddr_in send_addr;
	struct sockaddr_in receive_addr;
	bool inited;

public:
	SimpleUDP();
	virtual ~SimpleUDP();
public:
	bool init(const char *send_ip, int send_port, const char *receive_ip, int receive_port);
	int send(const uint8_t *buf, uint32_t length);
	int receive(uint8_t *buf, uint32_t buffer_length);
	bool is_inited();
};



#endif /* CMAKE_LIBS_SIMPLE_UDP_H_ */
