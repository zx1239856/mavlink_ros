/**
* iowrapper.h
* @author Zhang Xiang
* @description 
* @created Sun Sep 30 2018 02:59:10 GMT+0800 (CST)
* @license MIT
* @copyright All rights reserved, 2018
* @last-modified Sun Sep 30 2018 09:59:26 GMT+0800 (CST)
*/

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