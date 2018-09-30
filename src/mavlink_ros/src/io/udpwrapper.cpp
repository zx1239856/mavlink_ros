#include "udpwrapper.h"

udpWrapper::udpWrapper(ushort localPort,std::string remoteAddr,ushort remotePort):
_localEndpnt(udp::v4(),localPort),_remoteEndpnt(boost::asio::ip::address_v4::from_string(remoteAddr),remotePort),
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