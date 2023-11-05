#include <ros/ros.h>
#include "Livelybot_motor.h"
class send_recv
{
private:
    ros::NodeHandle _nm;
    std::string _robot_name;
    std::vector<std::string> _motor_names;
    const string spi_dev;

protected:
public:
    std::vector<motor> _motors;
    Livelybot_Driver _driver;

    send_recv(ros::NodeHandle nm, std::string robot_name, const std::string spi_dev) : _nm(nm), _robot_name(robot_name), _driver(spi_dev)
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
            int ID,num;
            _nm.getParam(_motor_name+"_ID",ID);
            _nm.getParam(_motor_name+"_num",num);
            std::cout<<_motor_name<<":"<<"ID: "<<ID<<" num: "<<num<<std::endl;
            _motors.push_back(motor(_nm, _robot_name, _motor_name, _driver_pointer,ID,num));
        }
    }
    ~send_recv();
};