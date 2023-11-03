#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>


#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>

#include "pai_msgs/MotorCmd.h"
#include "pai_msgs/MotorState.h"
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include "pai_motor_control_tool.h"

namespace pai_motor_control
{
    class PaiMotorController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    private:
        hardware_interface::JointHandle joint;
        ros::Subscriber sub_cmd, sub_ft;
        control_toolbox::Pid pid_controller_;
        boost::scoped_ptr<realtime_tools::RealtimePublisher<pai_msgs::MotorState>> controller_state_publisher_;
        const std::string& name="HuangPX-LD";
    public:
        std::string name_space;
        std::string joint_name;
        float sensor_torque;
        bool isHip, isThigh, isCalf, rqtTune;
        urdf::JointConstSharedPtr joint_urdf;
        realtime_tools::RealtimeBuffer<pai_msgs::MotorCmd> command;
        pai_msgs::MotorCmd lastCmd;
        pai_msgs::MotorState lastState;
        ServoCmd servoCmd;

        PaiMotorController(/* args */);
        ~PaiMotorController();
        virtual bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
        virtual void starting(const ros::Time &time);
        virtual void update(const ros::Time &time, const ros::Duration &period);
        virtual void stopping();
        void setTorqueCB(const geometry_msgs::WrenchStampedConstPtr &msg);
        void setCommandCB(const pai_msgs::MotorCmdConstPtr &msg);
        void positionLimits(double &position);
        void velocityLimits(double &velocity);
        void effortLimits(double &effort);

        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

    };    
}
#endif