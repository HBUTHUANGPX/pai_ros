#include "../../include/interface/Livelybot_motor.h"
// #include "Livelybot_motor.h"
motor::~motor()
{
}
void motor::fresh_motor(MotorState *motorState, float c_t)
{
    motor_fb_space = _driver_pointer->get_motor_state(_ID);
    // todo:
    // spi make crc
    // the fllowing ?: operation is used to prevent sudden changes when viewing images with rqt!!!!!!!!!!!!!!!
    // if (_motor_name == "L_toe" || _motor_name == "R_toe")
    // {
    //     // motorState->q = motor_fb_space.position / 2000000.0 * 360.0;
    //     (abs(motor_fb_space.position / 2000000.0 * 360.0) > 1000.0 ? motorState->q = motorState->q : motorState->q = motor_fb_space.position / 2000000.0 * 360.0);
    //     // motorState->dq = motor_fb_space.velocity / 2000000.0 * 360.0;
    //     (abs(motor_fb_space.velocity / 2000000.0 * 360.0) > 1000.0 ? motorState->dq = motorState->dq : motorState->dq = motor_fb_space.velocity / 2000000.0 *(2 * PI));
    //     // motorState->tauEst = motor_fb_space.torque / 100.0;
    //     (abs(motor_fb_space.torque / 100.0) > 10.0 ? motorState->tauEst = motorState->tauEst : motorState->tauEst = motor_fb_space.torque / 100.0);
    // }
    // else
    {
        // motorState->q = motor_fb_space.position / 100000.0 * 360.0;
        // motorState->dq = motor_fb_space.velocity / 100000.0 * 360.0;
        // motorState->tauEst = motor_fb_space.torque / 100.0;
        (abs(motor_fb_space.position / 100000.0 * 360.0) > 1000.0 ? motorState->q = motorState->q : motorState->q = motor_fb_space.position / 100000.0 * 360.0);
        (abs(motor_fb_space.velocity / 100000.0 * 360.0) > 1000.0 ? motorState->dq = motorState->dq : motorState->dq = motor_fb_space.velocity / 100000.0 * (2 * PI));
        (abs(motor_fb_space.torque / 2000.0) > 10.0 ? motorState->tauEst = motorState->tauEst : motorState->tauEst = motor_fb_space.torque / 2000.0);
    }
    // std::cout << "motor: " << _ID;
    // cout << _motor_name << " ";
    // std::cout << " position: " << motorState->q;
    // std::cout << " velocity: " << motorState->dq;
    // std::cout << " tauEst: " << motorState->tauEst << std::endl;
    _mt_st.q = lp.fresh(motorState->q, _mt_st.q,0.05);
    // std::cout << _mt_st.q << " ";
    _mt_st.dq = motorState->dq;//lp.fresh(motorState->dq, _mt_st.dq,0.7);
    // std::cout << _mt_st.dq << " ";
    _mt_st.tauEst =motorState->tauEst;//lp.fresh(motorState->tauEst, _mt_st.tauEst,0.7);
    // std::cout << _mt_st.tauEst << " ";
    // std::cout << std::endl;
    _mt_st.ddq = c_t;
    // std::cout << " cmd t: " << _mt_st.ddq << std::endl;

    _motor_pub.publish(_mt_st);
    // std::cout<<_ID<<"ok"<<std::endl;
}
