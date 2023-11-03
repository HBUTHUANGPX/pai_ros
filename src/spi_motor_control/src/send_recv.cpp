#include <ros/ros.h>
#include "Livelybot_Driver.h"
#include "pai_msgs/MotorState.h"
#include "pai_msgs/MotorCmd.h"
class motor
{
private:
    ros::NodeHandle _nm;
    ros::Subscriber _motor_sub; // 接收控制的数据
    ros::Publisher _motor_pub;
    std::string _robot_name, _motor_name;
    Livelybot_Driver *_driver_pointer;

public:
    motor(ros::NodeHandle nm, std::string robot_name, std::string motor_name, Livelybot_Driver *driver) : _nm(nm), _robot_name(robot_name), _motor_name(motor_name), _driver_pointer(driver)
    {
        _nm = nm;
        _robot_name = robot_name;
        _motor_name = motor_name;
        _motor_sub = _nm.subscribe("/" + _robot_name + "_real" + motor_name + "_controller/command", 1, &motor::Callback, this);
        _motor_pub = _nm.advertise<pai_msgs::MotorState>("/" + _robot_name + "_real" + motor_name + "_controller/state", 1);
    }
    // 接受来自于控制端的信号，在回调中处理，控制电机
    ~motor();
    void Callback(const pai_msgs::MotorCmd &msg);
};

motor::~motor()
{
}
void motor::Callback(const pai_msgs::MotorCmd &msg)
{
    // 将消息转发至SPI接口
}

class send_recv
{
private:
    ros::NodeHandle _nm;
    std::string _robot_name;
    std::vector<motor> _motors;
    std::vector<std::string> _motor_names;
    const string spi_dev;
    Livelybot_Driver _driver;

public:
    send_recv(std::string robot_name, const std::string spi_dev) : _robot_name(robot_name), _driver(spi_dev)
    {
        Livelybot_Driver *_driver_pointer = &_driver;

        _motor_names.push_back("L_hip");
        _motor_names.push_back("L_hip2");
        _motor_names.push_back("L_thigh");
        _motor_names.push_back("L_calf");
        _motor_names.push_back("L_toe");
        _motor_names.push_back("R_hip");
        _motor_names.push_back("R_hip2");
        _motor_names.push_back("R_thigh");
        _motor_names.push_back("R_calf");
        _motor_names.push_back("R_toe");
        for (std::string _motor_name : _motor_names)
        {
            _motors.push_back(motor(_nm, _robot_name, _motor_name, _driver_pointer));
        }
    }
    ~send_recv();
};
send_recv::~send_recv()
{
}

int main(int argc, char const *argv[])
{
    /* code */
    const std::string spi_dev("/dev/spidev4.1");

    return 0;
}
