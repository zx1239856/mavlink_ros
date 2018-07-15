/*
 * SimpleUDP.cpp
 *
 *  Created on: Oct 21, 2016
 *      Author: dozen
 */

#include "simple_udp.h"


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
#include <arpa/inet.h>
#include <linux/ip.h>

SimpleUDP::SimpleUDP() {
	// TODO Auto-generated constructor stub
	inited = false;
	sock_fd = -1;
}

SimpleUDP::~SimpleUDP() {
	// TODO Auto-generated destructor stub
}




bool SimpleUDP::init(const char *send_ip, int send_port, const char *receive_ip, int receive_port)
{
	sock_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	memset(&send_addr, 0, sizeof(send_addr));
	send_addr.sin_family = AF_INET;
	send_addr.sin_addr.s_addr = inet_addr(send_ip);
	send_addr.sin_port = htons(send_port);

	memset(&receive_addr, 0, sizeof(receive_addr));
	receive_addr.sin_family = AF_INET;
	receive_addr.sin_addr.s_addr = inet_addr(receive_ip);
	receive_addr.sin_port = htons(receive_port);

	//printf("%d \n", receive_addr.sin_addr.s_addr);
	if (-1 == bind(sock_fd,(struct sockaddr *)&receive_addr, sizeof(struct sockaddr)))
    {
		perror("error bind failed");
		close(sock_fd);
		return false;
    }

	/* Attempt to make it non blocking */
	if (fcntl(sock_fd, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock_fd);
		return false;
    }

	//unsigned char  service_type = 0xa0 | IPTOS_LOWDELAY ;
	unsigned char  service_type = 0xa0 ;
	if(setsockopt(sock_fd, SOL_IP/*IPPROTO_IP*/, IP_TOS, (void *)&service_type, sizeof(service_type)) < 0)
	        perror("setsockopt(IP_TOS) failed:");

	/*
	int priority = 6;
	if(setsockopt(sock_fd, SOL_SOCKET, SO_PRIORITY, &priority,
				sizeof(priority)) < 0){
	printf("Oh no\n");
	}
	*/

	inited = true;
	return true;
}

int SimpleUDP::send(const uint8_t *buf, uint32_t length)
{
	if(!inited) return -1;
	return sendto(sock_fd, buf, length, 0, (struct sockaddr*)&send_addr, sizeof(struct sockaddr_in));
}

int SimpleUDP::receive(uint8_t *buf, uint32_t buffer_length)
{
	if(!inited) return -1;
	struct sockaddr_in current_sender_addr;
	unsigned int current_sender_addr_length = sizeof(sockaddr);
	return recvfrom(sock_fd, (void *)buf, buffer_length, 0, (struct sockaddr *)&current_sender_addr, &current_sender_addr_length);
}
bool SimpleUDP::is_inited()
{
	return inited;
}
