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