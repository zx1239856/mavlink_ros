/**
* serialwrapper.cpp
* @author Zhang Xiang
* @description 
* @created Sun Sep 30 2018 04:06:42 GMT+0800 (CST)
* @license MIT
* @copyright All rights reserved, 2018
* @last-modified Sun Sep 30 2018 09:59:35 GMT+0800 (CST)
*/

#include "serialwrapper.h"

serialWrapper::serialWrapper(std::string dev, uint baudRate) : _devName(dev), _baudRate(baudRate), _port(_io)
{
}

serialWrapper::~serialWrapper()
{
    closePort();
}

bool serialWrapper::openPort()
{
    if (_port.is_open())
    {
        _port.close();
    }
    _port.open(_devName);
    if (_port.is_open())
    {
        _port.set_option(_baudRate);
        _port.set_option(_dataBits);
        _port.set_option(_flowCtl);
        _port.set_option(_parity);
        _port.set_option(_stopBits);
        return true;
    }
    else return false;
}

bool serialWrapper::closePort()
{
    _port.close();
}

bool serialWrapper::isOpen() const
{
    return _port.is_open();
}

ioWrapper &serialWrapper::operator<<(const std::vector<uint8_t> &writeBuffer)
{
    _port.write_some(boost::asio::buffer(writeBuffer));
    _io.run();
    return *this;
}

ioWrapper &serialWrapper::operator>>(std::vector<uint8_t> &readBuffer)
{
    readBuffer.resize(1024);
    _port.read_some(boost::asio::buffer(readBuffer));
    _io.run();
    return *this;
}

size_t serialWrapper::send(uint8_t *buffer, size_t len)
{
    auto res = _port.write_some(boost::asio::buffer(buffer, len));
    _io.run();
    return res;
}

size_t serialWrapper::read(uint8_t *buffer, size_t len)
{
    auto res = _port.read_some(boost::asio::buffer(buffer, len));
    _io.run();
    return res;
}