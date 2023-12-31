#ifndef _LIVELYBO_MOTOR_H
#define _LIVELYBO_MOTOR_H
#include <ros/ros.h>
#include "Livelybot_Driver.h"
#include "pai_msgs/MotorState.h"
#include "pai_msgs/MotorCmd.h"
#include "../messages/LowlevelState.h"
#include "lowpass.h"

class motor
{
private:
    ros::NodeHandle _nm;

    motor_fb_space_s motor_fb_space;
    ros::Publisher _motor_pub;

    // lowpass _lowpass;
    // std::vector<lowpass_c> _lowpass_v;
    lowpass_c lp;
    ros::Subscriber _motor_sub; // 接收控制的数据

public:
    int _ID;
    int _num;
    float cmd_t;
    pai_msgs::MotorState _mt_st;
    motor_fb_space_s motor_fb;
    Livelybot_Driver *_driver_pointer;
    std::string _robot_name, _motor_name;
    motor(ros::NodeHandle nm,
          std::string robot_name,
          std::string motor_name,
          Livelybot_Driver *driver,
          int ID, int num) : _nm(nm),
                             _robot_name(robot_name),
                             _motor_name(motor_name),
                             _driver_pointer(driver),
                             _ID(ID),
                             _num(num)
    {
        // _motor_sub = _nm.subscribe("/" + _robot_name + "_real" + motor_name + "_controller/command", 1, &motor::Callback, this);
        _motor_pub = _nm.advertise<pai_msgs::MotorState>("/" + _robot_name + "_real" + motor_name + "_controller/state", 1);
        // _lowpass_v.push_back(lowpass_c(0.001));
        memset(&motor_fb_space,0,sizeof(motor_fb_space));
    }
    // 接受来自于控制端的信号，在回调中处理，控制电机
    ~motor();
    // void Callback(const pai_msgs::MotorCmd &msg);
    void fresh_motor(MotorState *motorState,float c_t);
    void fresh_cmd_t(float cmd_t);
};

#endif