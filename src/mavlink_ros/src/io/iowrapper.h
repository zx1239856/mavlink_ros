#pragma once
#include <vector>
#include <stdint.h>
#include <stdlib.h>

class ioWrapper
{
    public:
    virtual ioWrapper& operator<<(const std::vector<uint8_t>& writeBuffer)=0;
    virtual ioWrapper& operator>>(std::vector<uint8_t>& readBuffer)=0;
    virtual size_t send(uint8_t *arr,size_t len)=0;
    virtual size_t read(uint8_t *arr,size_t len)=0;
};