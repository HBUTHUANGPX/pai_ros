#ifndef _LOWPASS_H_
#define _LOWPASS_H_
#include <ros/ros.h>

class lowpass_c
{
private:

public:
    lowpass_c()
    {
    }
    ~lowpass_c();
    float fresh(float now_data,float last_data,float _dt)
    {
        return last_data * ((1.0 - _dt) / 1.0) + now_data * (_dt/1.0);
    }
};

#endif