/**
* udpwrapper.cpp
* @author Zhang Xiang
* @description 
* @created Sun Sep 30 2018 01:54:20 GMT+0800 (CST)
* @license MIT
* @copyright All rights reserved, 2018
* @last-modified Sun Sep 30 2018 10:14:48 GMT+0800 (CST)
*/

#include "udpwrapper.h"

udpWrapper::udpWrapper(ushort localPort):
_localEndpnt(udp::v4(),localPort),
_socket(_io,_localEndpnt)
{

}

ioWrapper& udpWrapper::operator<<(const std::vector<uint8_t> &writeBuffer)
{
    _socket.send_to(boost::asio::buffer(writeBuffer),_remoteEndpnt);
    return *this;
}

ioWrapper& udpWrapper::operator>>(std::vector<uint8_t> &readBuffer)
{
    readBuffer.resize(1024);
    _socket.receive_from(boost::asio::buffer(readBuffer),_remoteEndpnt);
    return *this;
}

size_t udpWrapper::send(uint8_t *buffer,size_t len)
{
    auto res = _socket.send_to(boost::asio::buffer(buffer,len),_remoteEndpnt);
    return res;
}

size_t udpWrapper::read(uint8_t *buffer,size_t len)
{
    auto res = _socket.receive_from(boost::asio::buffer(buffer,len),_remoteEndpnt);
    return res;
}