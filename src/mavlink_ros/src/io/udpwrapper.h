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
    udpWrapper(ushort localPort,std::string remoteAddr,ushort remotePort);
    virtual ioWrapper &operator<<(const std::vector<uint8_t> &writeBuffer)override;
    virtual ioWrapper &operator>>(std::vector<uint8_t> &readBuffer)override;
    virtual size_t send(uint8_t *buffer,size_t len)override;
    virtual size_t read(uint8_t *buffer,size_t len)override;
};