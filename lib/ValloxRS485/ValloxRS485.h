#pragma once
#include "BaseClass.h"


class ValloxRS485: public BaseClass
{
public:
    ValloxRS485();
    ~ValloxRS485();

    void begin(void);
    void loop(uint32_t now_ms);

private:
};
