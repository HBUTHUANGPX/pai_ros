#ifndef _PAI_CONTROL_TOOL_H_
#define _PAI_CONTROL_TOOL_H_
#define PMSM (0x0A)
#define BRAKE (0x00)
#define PosStopF (2.146E+9f)
#define VelStopF (16000.0f)
#include "ros/ros.h"
#include <boost/algorithm/algorithm.hpp>

typedef struct 
{
    uint8_t mode;
    double pos;
    double posStiffness;
    double vel;
    double velStiffness;
    double torque;
} ServoCmd;
double clamp(double& value, double min, double max);  // eg. clamp(1.5, -1, 1) = 1
double computeVel(double current_position, double last_position, double last_velocity, double duration);  // get current velocity
double computeTorque(double current_position, double current_velocity, ServoCmd& cmd);  // get torque

#endif