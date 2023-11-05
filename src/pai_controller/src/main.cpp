#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include <stdlib.h>
#include "../include/interface/PaiIO.h"
#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/FSM/FSM.h"
#include <ros/ros.h>
using namespace std;
// 一个简单的demo，实现机器人站起来/蹲下的操作
bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
    system("stty sane"); // Terminal back to normal
    exit(0);
}

int main(int argc, char **argv)
{
    IOInterface *ioInter;
    ros::init(argc, argv, "pai_control", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Rate rate(1000);
    double dt = 0.001;
    std::string robot_name = "pai";
    // std::cout << "robot name " << robot_name << std::endl;
    Biped biped;
    StateEstimate stateEstimate;
    biped.setBiped();
    ioInter = new PaiIO(robot_name,"/dev/spidev4.1");
    LowlevelCmd *cmd = new LowlevelCmd();
    LowlevelState *state = new LowlevelState();
    std::cout<<"init ok"<<std::endl;
    signal(SIGINT, ShutDown);
    ///////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////simple control////////////////////////////////////////
    
    for (size_t i = 0; i < 10; i++)
    {
        cmd->motorCmd[i].q = 0.0;
        cmd->motorCmd[i].dq = 0.0;
        cmd->motorCmd[i].Kp = 300.0;
        cmd->motorCmd[i].Kd = 0.1;
    }
    //

    cmd->motorCmd[9].q = 0.0;
    cmd->motorCmd[9].dq = 0.0;
    cmd->motorCmd[9].Kp = 25.0;
    cmd->motorCmd[9].Kd = 0.025;
    cmd->motorCmd[4].q = 0.0;
    cmd->motorCmd[4].dq = 0.0;
    cmd->motorCmd[4].Kp = 25.0;
    cmd->motorCmd[4].Kd = 0.025;
    cout << state->motorState[6].q << " " << cmd->motorCmd[6].q << endl;
    while (ros::ok())
    {
        // ioInter->sendRecv(cmd, state);
        // ioInter->
        rate.sleep();
        // cont++;
        // cout << state->motorState[6].q << " " << cmd->motorCmd[6].q << endl;
    }

    ////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////gazebo////////////////////////////////////////////////

    // LegController *legController = new LegController(biped);
    // StateEstimatorContainer *stateEstimator = new StateEstimatorContainer(state,
    //                                                                       legController->data,
    //                                                                       &stateEstimate);
    // stateEstimator->addEstimator<CheaterOrientationEstimator>();
    // stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
    // DesiredStateCommand *desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData *_controlData = new ControlFSMData;
    // _controlData->_biped = &biped;
    // _controlData->_stateEstimator = stateEstimator;
    // _controlData->_legController = legController;
    // _controlData->_desiredStateCommand = desiredStateCommand;
    // _controlData->_interface = ioInter;
    // _controlData->_lowCmd = cmd;
    // _controlData->_lowState = state;
    // FSM *_FSMController = new FSM(_controlData);
    // while(ros::ok())
    // {
    //     _FSMController->run();
    //     rate.sleep();
    // }
    // delete _controlData;
    return 0;
}
