#include "../../include/interface/Livelybot_motor.h"
motor::~motor()
{
}
void motor::fresh_motor()
{
    motor_fb_space = _driver_pointer->get_motor_state(_ID);
    std::cout<<_ID<<"ok"<<std::endl;
}
