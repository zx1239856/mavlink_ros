#pragma once
#include "iowrapper.h"
#include <string>
#include <boost/asio.hpp>

class serialWrapper : public ioWrapper
{
    typedef boost::asio::serial_port serial;

  private:
    // serial port settings
    std::string _devName;
    serial::baud_rate _baudRate;
    serial::character_size _dataBits;
    serial::flow_control _flowCtl;
    serial::parity _parity;
    serial::stop_bits _stopBits;
    // actual serial
    boost::asio::io_service _io;
    serial _port;

  public:
    // constructor & destructor
    serialWrapper(std::string dev, uint baudRate = 921600);
    ~serialWrapper();
    // conf mutators
    void setDevName(std::string dev)
    {
        _devName = dev;
    }
    void setBaudRate(uint rate)
    {
        _baudRate = serial::baud_rate(rate);
    }
    void setDataBits(uint bits)
    {
        _dataBits = serial::character_size(bits);
    }
    void setFlowCtl(serial::flow_control::type type)
    {
        _flowCtl = serial::flow_control(type);
    }
    void setParity(serial::parity::type type)
    {
        _parity = serial::parity(type);
    }
    void setStopBits(serial::stop_bits::type type)
    {
        _stopBits = serial::stop_bits(type);
    }
    // io stuffs
    virtual ioWrapper &operator<<(const std::vector<uint8_t> &writeBuffer)override;
    virtual ioWrapper &operator>>(std::vector<uint8_t> &readBuffer)override;
    virtual size_t send(uint8_t *buffer,size_t len)override;
    virtual size_t read(uint8_t *buffer,size_t len)override;

    // status control
    bool openPort();
    bool closePort();
    bool isOpen()const;
};