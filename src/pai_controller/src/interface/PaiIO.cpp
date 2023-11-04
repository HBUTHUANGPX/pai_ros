#include "../../include/interface/PaiIO.h"
inline void RosShutDown(int sig)
{
    ROS_INFO("ROS interface shutting down!");
    ros::shutdown();
}
PaiIO::PaiIO(std::string robot_name, const std::string spi_name) : IOInterface(),
#if USE
                                                                   _send_recv(_nm, robot_name, spi_name)
#endif
{
    std::cout << "The control interface for ROS Gazebo simulation with cheat states from gazebo" << std::endl;
    _robot_name = robot_name;
    std::cout << "The robot name: " << robot_name << std::endl;
#if USE
#else
    // start subscriber
    initRecv();
    ROS_INFO("initRecv");
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(3000); // wait for subscribers start
    // initialize publisher
    initSend();
    ROS_INFO("initSend");
#endif
    signal(SIGINT, RosShutDown);

    cmdPanel = new KeyBoard();
}
PaiIO::~PaiIO()
{
    ros::shutdown();
}
void PaiIO::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    sendCmd(cmd);
    std::cout << "sendcmd ok" << std::endl;
    recvState(state);
    std::cout << "recvstate ok" << std::endl;
    cmdPanel->updateVelCmd(state);
    std::cout << "updata ok" << std::endl;
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
    std::cout << "ok" << std::endl;
}
void PaiIO::sendCmd(const LowlevelCmd *cmd)
{
#if USE
    for (motor motor_cmd : _send_recv._motors)
    {
        motor_cmd._driver_pointer->set_motor_position(motor_cmd._ID,
                                                      cmd->motorCmd[motor_cmd._num].q,
                                                      cmd->motorCmd[motor_cmd._num].dq,
                                                      cmd->motorCmd[motor_cmd._num].tau,
                                                      cmd->motorCmd[motor_cmd._num].Kp,
                                                      cmd->motorCmd[motor_cmd._num].Kd);
    }
#else

    for (int i = 0; i < 10; i++)
    {
        _lowCmd.motorCmd[i].mode = 0X0A; // alwasy set it to 0X0A
        _lowCmd.motorCmd[i].q = cmd->motorCmd[i].q;

        _lowCmd.motorCmd[i].dq = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].tau = cmd->motorCmd[i].tau;
        _lowCmd.motorCmd[i].Kd = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].Kp = cmd->motorCmd[i].Kp;
    }

    // std::cout<<_lowCmd.motorCmd[6].q<<std::endl;

    for (int m = 0; m < 10; m++)
    {
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
#endif
    ros::spinOnce();
}
void PaiIO::recvState(LowlevelState *state)
{
#if USE
    for (motor motor_cmd : _send_recv._motors)
    {
        motor_cmd.fresh_motor();
        state->motorState[motor_cmd._num].q = motor_cmd.motor_fb_space.position;
        state->motorState[motor_cmd._num].dq = motor_cmd.motor_fb_space.velocity;
        state->motorState[motor_cmd._num].tauEst = motor_cmd.motor_fb_space.torque;
    }
#else
    for (int i = 0; i < 10; i++)
    {
        state->motorState[i].q = _highState.motorState[i].q;
        state->motorState[i].dq = _highState.motorState[i].dq;
        state->motorState[i].tauEst = _highState.motorState[i].tauEst;
    }
    for (int i = 0; i < 3; i++)
    {
        state->imu.quaternion[i] = _highState.imu.quaternion[i];
        state->imu.gyroscope[i] = _highState.imu.gyroscope[i];
        state->position[i] = _highState.position[i];
        state->vWorld[i] = _highState.velocity[i];
    }
    state->imu.quaternion[3] = _highState.imu.quaternion[3];
#endif
}
#if USE // 使用真实机器人
std::string _use = "_real";
#else   // 使用Gazebo
std::string _use = "_gazebo";
void PaiIO::initSend()
{
    _servo_pub[0] = _nm.advertise<pai_msgs::MotorCmd>("/" + _robot_name + _use + "L_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<pai_msgs::MotorCmd>("/" + _robot_name + _use + "L_hip2_controller/command", 1);
    _servo_pub[2] = _nm.advertise<pai_msgs::MotorCmd>("/" + _robot_name + _use + "L_thigh_controller/command", 1);
    _servo_pub[3] = _nm.advertise<pai_msgs::MotorCmd>("/" + _robot_name + _use + "L_calf_controller/command", 1);
    _servo_pub[4] = _nm.advertise<pai_msgs::MotorCmd>("/" + _robot_name + _use + "L_toe_controller/command", 1);
    _servo_pub[5] = _nm.advertise<pai_msgs::MotorCmd>("/" + _robot_name + _use + "R_hip_controller/command", 1);
    _servo_pub[6] = _nm.advertise<pai_msgs::MotorCmd>("/" + _robot_name + _use + "R_hip2_controller/command", 1);
    _servo_pub[7] = _nm.advertise<pai_msgs::MotorCmd>("/" + _robot_name + _use + "R_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<pai_msgs::MotorCmd>("/" + _robot_name + _use + "R_calf_controller/command", 1);
    _servo_pub[9] = _nm.advertise<pai_msgs::MotorCmd>("/" + _robot_name + _use + "R_toe_controller/command", 1);
}
void PaiIO::initRecv()
{
#if USE // 使用真实机器人
    _state_sub = _nm.subscribe("/real/model_states", 1, &PaiIO::StateCallback, this);
#else   // 使用Gazebo
    _state_sub = _nm.subscribe("/gazebo/model_states", 1, &PaiIO::StateCallback, this);
#endif
    _servo_sub[0] = _nm.subscribe("/" + _robot_name + _use + "L_hip_controller/state", 1, &PaiIO::LhipCallback, this);
    _servo_sub[1] = _nm.subscribe("/" + _robot_name + _use + "L_hip2_controller/state", 1, &PaiIO::Lhip2Callback, this);
    _servo_sub[2] = _nm.subscribe("/" + _robot_name + _use + "L_thigh_controller/state", 1, &PaiIO::LthighCallback, this);
    _servo_sub[3] = _nm.subscribe("/" + _robot_name + _use + "L_calf_controller/state", 1, &PaiIO::LcalfCallback, this);
    _servo_sub[4] = _nm.subscribe("/" + _robot_name + _use + "L_toe_controller/state", 1, &PaiIO::LtoeCallback, this);
    _servo_sub[5] = _nm.subscribe("/" + _robot_name + _use + "R_hip_controller/state", 1, &PaiIO::RhipCallback, this);
    _servo_sub[6] = _nm.subscribe("/" + _robot_name + _use + "R_hip2_controller/state", 1, &PaiIO::Rhip2Callback, this);
    _servo_sub[7] = _nm.subscribe("/" + _robot_name + _use + "R_thigh_controller/state", 1, &PaiIO::RthighCallback, this);
    _servo_sub[8] = _nm.subscribe("/" + _robot_name + _use + "R_calf_controller/state", 1, &PaiIO::RcalfCallback, this);
    _servo_sub[9] = _nm.subscribe("/" + _robot_name + _use + "R_toe_controller/state", 1, &PaiIO::RtoeCallback, this);
}

void PaiIO::StateCallback(const gazebo_msgs::ModelStates &msg)
{
    // std::cout <<"StateCallback "<< std::endl;
    int robot_index;
    // std::cout << msg.name.size() << std::endl;
    for (int i = 0; i < msg.name.size(); i++)
    {
        // std::cout<<msg.name[i]<<" "<<_robot_name + "_gazebo"<<std::endl;
        if (msg.name[i] == _robot_name + _use)
        {
            robot_index = i;
        }
    }
    _highState.position[0] = msg.pose[robot_index].position.x;
    _highState.position[1] = msg.pose[robot_index].position.y;
    _highState.position[2] = msg.pose[robot_index].position.z;
    _highState.velocity[0] = msg.twist[robot_index].linear.x;
    _highState.velocity[1] = msg.twist[robot_index].linear.y;
    _highState.velocity[2] = msg.twist[robot_index].linear.z;

    _highState.imu.quaternion[0] = msg.pose[robot_index].orientation.w;
    _highState.imu.quaternion[1] = msg.pose[robot_index].orientation.x;
    _highState.imu.quaternion[2] = msg.pose[robot_index].orientation.y;
    _highState.imu.quaternion[3] = msg.pose[robot_index].orientation.z;

    _highState.imu.gyroscope[0] = msg.twist[robot_index].angular.x;
    _highState.imu.gyroscope[1] = msg.twist[robot_index].angular.y;
    _highState.imu.gyroscope[2] = msg.twist[robot_index].angular.z;
}
void PaiIO::LhipCallback(const pai_msgs::MotorState &msg)
{
    _highState.motorState[0].mode = msg.mode;
    _highState.motorState[0].q = msg.q;
    _highState.motorState[0].dq = msg.dq;
    _highState.motorState[0].tauEst = msg.tauEst;
}
void PaiIO::Lhip2Callback(const pai_msgs::MotorState &msg)
{
    _highState.motorState[1].mode = msg.mode;
    _highState.motorState[1].q = msg.q;
    _highState.motorState[1].dq = msg.dq;
    _highState.motorState[1].tauEst = msg.tauEst;
}
void PaiIO::LthighCallback(const pai_msgs::MotorState &msg)
{
    _highState.motorState[2].mode = msg.mode;
    _highState.motorState[2].q = msg.q;
    _highState.motorState[2].dq = msg.dq;
    _highState.motorState[2].tauEst = msg.tauEst;
}
void PaiIO::LcalfCallback(const pai_msgs::MotorState &msg)
{
    _highState.motorState[3].mode = msg.mode;
    _highState.motorState[3].q = msg.q;
    _highState.motorState[3].dq = msg.dq;
    _highState.motorState[3].tauEst = msg.tauEst;
}
void PaiIO::LtoeCallback(const pai_msgs::MotorState &msg)
{
    _highState.motorState[4].mode = msg.mode;
    _highState.motorState[4].q = msg.q;
    _highState.motorState[4].dq = msg.dq;
    _highState.motorState[4].tauEst = msg.tauEst;
}
void PaiIO::RhipCallback(const pai_msgs::MotorState &msg)
{
    _highState.motorState[5].mode = msg.mode;
    _highState.motorState[5].q = msg.q;
    _highState.motorState[5].dq = msg.dq;
    _highState.motorState[5].tauEst = msg.tauEst;
}
void PaiIO::Rhip2Callback(const pai_msgs::MotorState &msg)
{
    _highState.motorState[6].mode = msg.mode;
    _highState.motorState[6].q = msg.q;
    _highState.motorState[6].dq = msg.dq;
    _highState.motorState[6].tauEst = msg.tauEst;
}
void PaiIO::RthighCallback(const pai_msgs::MotorState &msg)
{
    _highState.motorState[7].mode = msg.mode;
    _highState.motorState[7].q = msg.q;
    _highState.motorState[7].dq = msg.dq;
    _highState.motorState[7].tauEst = msg.tauEst;
}
void PaiIO::RcalfCallback(const pai_msgs::MotorState &msg)
{
    _highState.motorState[8].mode = msg.mode;
    _highState.motorState[8].q = msg.q;
    _highState.motorState[8].dq = msg.dq;
    _highState.motorState[8].tauEst = msg.tauEst;
}
void PaiIO::RtoeCallback(const pai_msgs::MotorState &msg)
{
    _highState.motorState[9].mode = msg.mode;
    _highState.motorState[9].q = msg.q;
    _highState.motorState[9].dq = msg.dq;
    _highState.motorState[9].tauEst = msg.tauEst;
}
#endif
