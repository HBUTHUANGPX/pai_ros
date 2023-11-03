#include "motor_controller.h"
#include <pluginlib/class_list_macros.h>

namespace pai_motor_control
{
    PaiMotorController::PaiMotorController()
    {
        rqtTune = false;
        memset(&lastCmd, 0, sizeof(pai_msgs::MotorCmd));
        memset(&lastState, 0, sizeof(pai_msgs::MotorState));
        memset(&servoCmd, 0, sizeof(ServoCmd));
    }
    PaiMotorController::~PaiMotorController()
    {
        sub_ft.shutdown();
        sub_cmd.shutdown();
    }
    void PaiMotorController::setTorqueCB(const geometry_msgs::WrenchStampedConstPtr& msg)
    {
        if(isHip) sensor_torque = msg->wrench.torque.x;
        else sensor_torque = msg->wrench.torque.y;
        // printf("sensor torque%f\n", sensor_torque);
    }

    void PaiMotorController::setCommandCB(const pai_msgs::MotorCmdConstPtr& msg)
    {
        lastCmd.mode = msg->mode;
        lastCmd.q = msg->q;
        lastCmd.Kp = msg->Kp;
        lastCmd.dq = msg->dq;
        lastCmd.Kd = msg->Kd;
        lastCmd.tau = msg->tau;
        // the writeFromNonRT can be used in RT, if you have the guarantee that
        //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
        //  * there is only one single rt thread
        command.writeFromNonRT(lastCmd);
    }
    /**
     * @file motor_controller.cpp
     * @brief Controller initialization in non-realtime
     */
    bool PaiMotorController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        isHip = false;
        isThigh = false;
        isCalf = false;
        sensor_torque = 0;
        name_space = n.getNamespace();
        if (!n.getParam("joint", joint_name))
        // if(0)
        {
            ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
            return false;
        }
        else
        {
            // std::cout<<"=--=-=-=-=-=-=-"<<std::endl;
            ROS_INFO_NAMED(name,n.getNamespace().c_str());
            ROS_INFO_STREAM_NAMED(name,n.getNamespace().c_str());
        }
        
        // load pid param from ymal only if rqt need
        if(rqtTune) {
        // Load PID Controller using gains set on parameter server
        if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
            ROS_ERROR("Failed to LOAD pid");
            return false;
        }
        urdf::Model urdf; // Get URDF info about joint
        if (!urdf.initParamWithNodeHandle("robot_description", n))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf)
        {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
            return false;
        }
        if (joint_name == "FR_hip_joint" || joint_name == "FL_hip_joint" || joint_name == "RR_hip_joint" || joint_name == "RL_hip_joint")
        {
            isHip = true;
        }
        if (joint_name == "FR_calf_joint" || joint_name == "FL_calf_joint" || joint_name == "RR_calf_joint" || joint_name == "RL_calf_joint")
        {
            isCalf = true;
        }
        joint = robot->getHandle(joint_name);

        // Start command subscriber
        sub_ft = n.subscribe(name_space + "/" + "joint_wrench", 1, &PaiMotorController::setTorqueCB, this);
        sub_cmd = n.subscribe("command", 20, &PaiMotorController::setCommandCB, this);

        // pub_state = n.advertise<unitree_legged_msgs::MotorState>(name_space + "/state", 20);
        // Start realtime state publisher
        controller_state_publisher_.reset(
            new realtime_tools::RealtimePublisher<pai_msgs::MotorState>(n, name_space + "/state", 1));

        return true;
    }
    void PaiMotorController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
    }
    void PaiMotorController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
    }
    void PaiMotorController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
    }
    void PaiMotorController::starting(const ros::Time& time) // Controller startup in realtime
    {
        // lastCmd.Kp = 0;
        // lastCmd.Kd = 0;
        double init_pos = joint.getPosition();
        lastCmd.q = init_pos;
        lastState.q = init_pos;
        lastCmd.dq = 0;
        lastState.dq = 0;
        lastCmd.tau = 0;
        lastState.tauEst = 0;
        command.initRT(lastCmd);

        pid_controller_.reset();
    }   
    void PaiMotorController::update(const ros::Time& time, const ros::Duration& period)// Controller update loop in realtime
    {
        double currentPos, currentVel, calcTorque;
        lastCmd = *(command.readFromRT());

        // set command data
        if(lastCmd.mode == PMSM) {
            servoCmd.pos = lastCmd.q;
            std::cout<<"servoCmd.pos:"<<servoCmd.pos<<std::endl;
            positionLimits(servoCmd.pos);
            servoCmd.posStiffness = lastCmd.Kp;
            if(fabs(lastCmd.q - PosStopF) < 0.00001){
                servoCmd.posStiffness = 0;
            }
            servoCmd.vel = lastCmd.dq;
            velocityLimits(servoCmd.vel);
            servoCmd.velStiffness = lastCmd.Kd;
            if(fabs(lastCmd.dq - VelStopF) < 0.00001){
                servoCmd.velStiffness = 0;
            }
            servoCmd.torque = lastCmd.tau;
            effortLimits(servoCmd.torque);
        }
        if(lastCmd.mode == BRAKE) {
            servoCmd.posStiffness = 0;
            servoCmd.vel = 0;
            servoCmd.velStiffness = 20;
            servoCmd.torque = 0;
            effortLimits(servoCmd.torque);
        }

        // } else {
        //     servoCmd.posStiffness = 0;
        //     servoCmd.velStiffness = 5;
        //     servoCmd.torque = 0;
        // }
        
        // rqt set P D gains
        if(rqtTune) {
            double i, i_max, i_min;
            getGains(servoCmd.posStiffness,i,servoCmd.velStiffness,i_max,i_min);
        } 

        currentPos = joint.getPosition();
        std::cout<<"currentPos: "<<currentPos/3.1415926*180<<" "<<currentPos<<std::endl;
        currentVel = computeVel(currentPos, (double)lastState.q, (double)lastState.dq, period.toSec());
        calcTorque = computeTorque(currentPos, currentVel, servoCmd);      
        effortLimits(calcTorque);

        joint.setCommand(calcTorque);

        lastState.q = currentPos;
        lastState.dq = currentVel;
        // lastState.tauEst = calcTorque;
        // lastState.tauEst = sensor_torque;
        lastState.tauEst = joint.getEffort();

        // pub_state.publish(lastState);
        // publish state
        if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
            controller_state_publisher_->msg_.q = lastState.q;
            controller_state_publisher_->msg_.dq = lastState.dq;
            controller_state_publisher_->msg_.tauEst = lastState.tauEst;
            controller_state_publisher_->unlockAndPublish();
        }

        // printf("sensor torque%f\n", sensor_torque);

        // if(joint_name == "wrist1_joint") printf("wrist1 setp:%f  getp:%f t:%f\n", servoCmd.pos, currentPos, calcTorque);
    }

    // Controller stopping in realtime
    void PaiMotorController::stopping(){}

    void PaiMotorController::positionLimits(double &position)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
    }

    void PaiMotorController::velocityLimits(double &velocity)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity);
    }

    void PaiMotorController::effortLimits(double &effort)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort);
    }

}
PLUGINLIB_EXPORT_CLASS(pai_motor_control::PaiMotorController, controller_interface::ControllerBase);
