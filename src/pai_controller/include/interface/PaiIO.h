#ifndef _PAIIO_H_
#define _PAIIO_H_

#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>
#include "pai_msgs/MotorCmd.h"
#include "pai_msgs/MotorState.h"
#include "pai_msgs/HighState.h"
#include "pai_msgs/LowCmd.h"

#include "IOInterface.h"
#include "KeyBoard.h"
#include <csignal>
class PaiIO: public IOInterface
{
private:
    ros::NodeHandle _nm;
    ros::Subscriber _servo_sub[10], _state_sub;
    ros::Publisher _servo_pub[10];
    std::string _robot_name;
    pai_msgs::HighState _highState;
    pai_msgs::LowCmd _lowCmd;

public:
    PaiIO(std::string robot_name);
    ~PaiIO();
    void sendCmd(const LowlevelCmd *cmd);
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
    void recvState(LowlevelState *state);
    void initSend();
    void initRecv();
    void StateCallback(const gazebo_msgs::ModelStates &msg);
    void LhipCallback(const pai_msgs::MotorState &msg);
    void Lhip2Callback(const pai_msgs::MotorState &msg);
    void LthighCallback(const pai_msgs::MotorState &msg);
    void LcalfCallback(const pai_msgs::MotorState &msg);
    void LtoeCallback(const pai_msgs::MotorState &msg);
    void RhipCallback(const pai_msgs::MotorState &msg);
    void Rhip2Callback(const pai_msgs::MotorState &msg);
    void RthighCallback(const pai_msgs::MotorState &msg);
    void RcalfCallback(const pai_msgs::MotorState &msg);
    void RtoeCallback(const pai_msgs::MotorState &msg);
};

#endif