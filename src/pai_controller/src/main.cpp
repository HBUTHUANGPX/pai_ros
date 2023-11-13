#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include <stdlib.h>
#include "../include/LivelyBot/USBCAN/LD_Socket_Can.h"
#include "../include/LivelyBot/USBCAN/LD_USBCAN.h"
#include "../include/interface/PaiIO.h"
#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/FSM/FSM.h"
#include <ros/ros.h>
#include <thread>
using namespace std;
// 一个简单的demo，实现机器人站起来/蹲下的操作
bool running = true;
// void Append(std::ostream *ostr, const char *format, ...)
// {
//     char buf[1024] = {};
//     va_list ap;

//     va_start(ap, format);
//     int count = ::vsnprintf(buf, sizeof(buf), format, ap);
//     va_end(ap);
//     ostr->write(buf, count);
// }
void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
    system("stty sane"); // Terminal back to normal
    exit(0);
}
#define CAN_TEST

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pai_control", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Rate rate(1000);
#ifdef CAN_TEST
    Socket_CAN a("/dev/fdcanusb", "vcan0");
    Socket_CAN *pointer = &a;
    struct canfd_frame recv_frame = {};
    struct canfd_frame send_frame = {};
    socklen_t len = sizeof(a.addr);
    signal(SIGINT, ShutDown);
    struct canfd_frame cfd;
    char buf[1024] = {};
    std::ostringstream fdcanusb_buf;
    LD_Motor_Ctl ctl(pointer);
    // while (ros::ok())
    std::thread recv(&LD_Motor_Ctl::loop_read, &ctl);
    while (0)
    {
        FD_ZERO(&a.rfds);
        FD_SET(a._socket, &a.rfds);
        FD_SET(a.fd, &a.rfds);
        ctl.set_position_32_kpkd(1, 5, 0, 0, 1, 0.51);
        ctl.set_position_32_kpkd(2, 5, 0, 0, 1, 0.51);
        ctl.set_position_32_kpkd(3, 5, 0, 0, 1, 0.51);
        rate.sleep();
    }

    while (1)
    {
        FD_ZERO(&a.rfds);
        FD_SET(a._socket, &a.rfds);
        FD_SET(a.fd, &a.rfds);
        const int ret = ::select(std::max(a.fd, a._socket) + 1, &a.rfds, nullptr, nullptr, nullptr);

        if (FD_ISSET(a.fd, &a.rfds))
        {
            read(a.fd, &buf, sizeof(buf));
            if (a.StartsWith(buf, "rcv "))
            {
                Tokenizer tokenizer(buf, " ");
                const auto rcv = tokenizer.next();
                const auto address = tokenizer.next();
                const auto data = tokenizer.next();
                // cout<<rcv<<","<<address<<endl;
                send_frame.len = a.ParseCanData(data, send_frame.data);

                // sendto(a.fd, &send_frame, sizeof(struct canfd_frame), 0, (struct sockaddr *)&a.addr, sizeof(a.addr));
                // continue;

                std::ostringstream cmd;
                // if (address == "8004")
                // {
                //     a.Append(&cmd, "can send %X ", 0x8006);
                // }
                // else
                // {
                // }
                a.Append(&cmd, "can send %X ", 0x8005);

                for (size_t i = 0; i < send_frame.len; i++)
                {
                    a.Append(&cmd, "%02X", static_cast<int>(send_frame.data[i]));
                }
                a.Append(&cmd, "\n");
                // cout << write(a.fd, cmd.str().c_str(), cmd.str().size()) << endl;
                if (write(a.fd, cmd.str().c_str(), cmd.str().size()) >= 0)
                {
                    // ROS_INFO_STREAM(cmd.str());
                }
                else
                {
                }
                // ROS_INFO_STREAM("2");
            }
            std::memset(&buf, 0, sizeof(buf));
        }
    }

#else
    IOInterface *ioInter;
    double dt = 0.001;
    std::string robot_name = "pai";
    // std::cout << "robot name " << robot_name << std::endl;
    Biped biped;
    StateEstimate stateEstimate;
    biped.setBiped();
    ioInter = new PaiIO(robot_name, "/dev/spidev4.1");
    LowlevelCmd *cmd = new LowlevelCmd();
    LowlevelState *state = new LowlevelState();
    std::cout << "init ok" << std::endl;
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
#endif
    return 0;
}
