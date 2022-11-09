#pragma once
#include <stdint.h>


class BaseClass
{
private:
    /* data */
public:
    BaseClass(/* args */)
    {

    }

    ~BaseClass()
    {

    }

    // Dummy interface functions to be overwritten in implementation
    virtual void begin(void) {};
    virtual void loop(uint32_t const now_ms) {};
};
