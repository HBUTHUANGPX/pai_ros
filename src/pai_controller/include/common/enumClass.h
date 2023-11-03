#ifndef _ENUMCLASS_H_
#define _ENUMCLASS_H_

enum class CtrlPlateform{
    Gazebo_Pai,
    Real_Pai
};

enum class UserCommand{
    None,
    Walking,
    FixStand,
    Passive
};
enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    PDSTAND,
    QPSTAND,
    WALKING,
    PUSHING,
    PROBE,
    SLAM,       // slam
};
#endif