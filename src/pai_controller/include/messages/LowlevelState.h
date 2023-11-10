#ifndef LOWLEVELSTATE_H
#define LOWLEVELSTATE_H

#include <iostream>
#include <string>
#include "../common/cppTypes.h"
#include "../common/enumClass.h"

struct UserValue{
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    float vx; // vx in body frame
    float vy; // vy in body frame
    float turn_rate;

    UserValue(){
        setZero();
    }
    void setZero(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
        vx = 0;
        vy = 0;
        turn_rate = 0;
    }
};

struct MotorState
{
    unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState()
    {
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4];
    float gyroscope[3];
    float accelerometer[3];

    IMU()
    {
        for(int i = 0; i < 3; i++)
        {
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }
};

struct LowlevelState
{
    IMU imu;
    MotorState motorState[10];
    UserCommand userCmd;
    UserValue userValue;

    float position[3];
    float vWorld[3];
    float rpy[3];

    LowlevelState()
    {
        for(int i = 0; i < 3; i++)
        {
            position[i] = 0;
            vWorld[i] = 0;
            rpy[i] = 0;
        }
        for (size_t i = 0; i < 10; i++)
        {
            motorState[i].dq=0.0;
            motorState[i].ddq=0.0;
            motorState[i].q=0.0;
            motorState[i].tauEst=0.0;
            motorState[i].mode=0;
        }
        
    }
};


#endif //LowlevelState