#include "pai_motor_control_tool.h"
// #include <boost/algorithm.hpp>
double clamp(double& value, double min, double max)
{
    value = std::max(min, std::min(max, value));
}
double computeVel(double current_position, double last_position, double last_velocity, double duration)
{
    return (current_position-last_position)/duration;
}
double computeTorque(double current_position, double current_velocity, ServoCmd& cmd)
{
    return cmd.posStiffness*(cmd.pos-current_position)+cmd.velStiffness*(cmd.vel-current_velocity)+cmd.torque;
}