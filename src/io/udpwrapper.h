/**
* udpwrapper.h
* @author Zhang Xiang
* @description 
* @created Sun Sep 30 2018 01:47:21 GMT+0800 (CST)
* @license MIT
* @copyright All rights reserved, 2018
* @last-modified Sun Sep 30 2018 10:14:21 GMT+0800 (CST)
*/

#pragma once
#include "iowrapper.h"
#include <boost/asio.hpp>

class udpWrapper : public ioWrapper
{
    typedef boost::asio::ip::udp udp;

  private:
    udp::endpoint _localEndpnt;
    udp::endpoint _remoteEndpnt;
    boost::asio::io_service _io;
    udp::socket _socket;

  public:
    udpWrapper(ushort localPort);
    virtual ioWrapper &operator<<(const std::vector<uint8_t> &writeBuffer)override;
    virtual ioWrapper &operator>>(std::vector<uint8_t> &readBuffer)override;
    virtual size_t send(uint8_t *buffer,size_t len)override;
    virtual size_t read(uint8_t *buffer,size_t len)override;
};