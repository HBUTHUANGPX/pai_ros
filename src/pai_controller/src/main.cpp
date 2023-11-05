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
void set_motor(ros::NodeHandle nh)
{
    std::vector<std::string> _motor_names, side;
    _motor_names.push_back("_hip");
    _motor_names.push_back("_hip2");
    _motor_names.push_back("_thigh");
    _motor_names.push_back("_calf");
    _motor_names.push_back("_toe");
    side.push_back("L");
    side.push_back("R");
    int ID[5] = {5, 4, 3, 2, 1};
    int CAN[2] = {0x10,0x20};
    int num[5] = {0, 1, 2, 3, 4};
    int count_side = 0;
    int count_motor = 0;
    for (std::string _side : side)
    {
        for (std::string _motor_name : _motor_names)
        {
            std::cout << _side + _motor_name << std::endl;
            nh.setParam(_side + _motor_name + "_ID", CAN[count_side] | ID[count_motor]);
            nh.setParam(_side + _motor_name + "_num", num[count_motor] + count_side * 5);
            count_motor++;
        }
        count_motor = 0;
        count_side++;
    }
}
int main(int argc, char **argv)
{
    IOInterface *ioInter;
    ros::init(argc, argv, "pai_control", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    set_motor(nh);
    ros::Rate rate(1000);
    double dt = 0.001;
    std::string robot_name = "pai";
    // std::cout << "robot name " << robot_name << std::endl;
    Biped biped;
    StateEstimate stateEstimate;
    biped.setBiped();
    // PaiIO pai(robot_name, "/dev/spidev4.1");
    ioInter = new PaiIO(robot_name,"/dev/spidev4.1");

    LowlevelCmd *cmd = new LowlevelCmd();
    LowlevelState *state = new LowlevelState();
    std::cout << "init ok" << std::endl;
    signal(SIGINT, ShutDown);
    ///////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////simple control////////////////////////////////////////

    for (size_t i = 0; i < 10; i++)
    {
        cmd->motorCmd[i].q = 0.0;
        cmd->motorCmd[i].dq = 0.0;
        cmd->motorCmd[i].Kp = 0.0;
        cmd->motorCmd[i].Kd = 0.1;
    }
    //
    while (ros::ok())
    {
        // cout<<spi_flag<<endl;
        ioInter->sendRecv(cmd, state);
        // pai._send_recv._motors[0]._driver_pointer->set_motor_position(pai._send_recv._motors[0]._ID, 0, 0, 0, 0, 0);
        // pai._send_recv._motors[0].fresh_motor();
        // pai._send_recv._motors[0]._driver_pointer->set_motor_position(pai._send_recv._motors[0]._ID, 0);
        // printf("motor0x%02x pos:%d\n",
        //        pai._send_recv._motors[0]._ID,
        //        pai._send_recv._motors[0]._driver_pointer->get_motor_state(pai._send_recv._motors[0]._ID).position);
        // pai._send_recv._motors[0]._driver_pointer->set_motor_position(pai._send_recv._motors[0]._ID, 0);
        // pai._send_recv._motors[0]._driver_pointer->spi_send();
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
