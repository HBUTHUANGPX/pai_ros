#ifndef _LD_USBCAN_H_
#define _LD_USBCAN_H_
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/serial.h>
#include <net/if.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>
#include <termios.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include "LD_Socket_Can.h"
#pragma pack(1)
struct Motor_FDCan_S
{
    int8_t head[5];  // 0x01, 0x00, 0x0A, 0x0B, 0x20  0-4
    int32_t pos;     // 5-9
    int32_t vel;     // 9-12
    int32_t torque;  // 13-16
    int8_t space[2]; // 0x06, 0x23 17-18
    int16_t kp;      // 19-20
    int16_t kd;      // 21-22
    int8_t tail;     // 0x18, 0x04, 0x00, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50
};
#pragma pack()
class LD_Motor
{
private:
public:
    LD_Motor(/* args */);
    ~LD_Motor();
};

LD_Motor::LD_Motor(/* args */)
{
}

LD_Motor::~LD_Motor()
{
}

class LD_Motor_Ctl
{
private:
    Socket_CAN *_can_pointer;
    uint8_t cmd[32];
    // std::ostringstream s_cmd;

public:
    LD_Motor_Ctl(Socket_CAN *pointer) : _can_pointer(pointer)
    {
        uint8_t _c[] = {0x01, 0x00, 0x0A, 0x0B, 0x20,
                        0x00, 0x00, 0x00, 0x00, // 5-8
                        0x00, 0x00, 0x00, 0x00, // 9-12
                        0x00, 0x00, 0x00, 0x00, // 13-16
                        0x06, 0x23,             // 17-18
                        0x3F, 0xFF,             // 19-20
                        0x3F, 0xFF,             // 21-22
                        0x18, 0x04, 0x00, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50};
        for (size_t i = 0; i < 32; i++)
        {
            cmd[i] = _c[i];
        }
    }
    ~LD_Motor_Ctl();
    void loop_read();
    void set_position_32_kpkd(int ID, int32_t pos, int32_t vel, int32_t torque, float kp, float kd);
    void fdcan_push_queue(int32_t ID, uint8_t *data, uint16_t frame_len);
};
void LD_Motor_Ctl::fdcan_push_queue(int32_t ID, uint8_t *data, uint16_t frame_len)
{
    // s_cmd.str(""); // 清除流的内容
    // s_cmd.clear(); // 重置流的错误状态
    std::ostringstream *s_cmd = new std::ostringstream;
    _can_pointer->Append(s_cmd, "can send %X ", ID);
    for (size_t i = 0; i < frame_len; i++)
    {
        _can_pointer->Append(s_cmd, "%02X", static_cast<int>(data[i]));
    }
    _can_pointer->Append(s_cmd, "\n");
    // cout << write(a.fd, cmd.str().c_str(), cmd.str().size()) << endl;
    if (write(_can_pointer->fd, ((*s_cmd).str()).c_str(), ((*s_cmd).str()).size()) == -1)
    {
        perror("write error");
    }
    else
    {
        // ROS_INFO_STREAM(((*s_cmd).str()));
    }
    delete s_cmd;
}
void LD_Motor_Ctl::loop_read()
{
    char buf[1024] = {};
    struct canfd_frame send_frame = {};
    while (1)
    {
        FD_ZERO(&(_can_pointer->rfds));
        FD_SET(_can_pointer->_socket, &(_can_pointer->rfds));
        FD_SET(_can_pointer->fd, &(_can_pointer->rfds));
        const int ret = ::select(std::max(_can_pointer->fd, _can_pointer->_socket) + 1, &(_can_pointer->rfds), nullptr, nullptr, nullptr);
        if (FD_ISSET(_can_pointer->fd, &(_can_pointer->rfds)))
        {
            read(_can_pointer->fd, &buf, sizeof(buf));
            if (_can_pointer->StartsWith(buf, "rcv "))
            {
                Tokenizer tokenizer(buf, " ");
                const auto rcv = tokenizer.next();
                const auto address = tokenizer.next();
                const auto data = tokenizer.next();
                // cout<<rcv<<","<<address<<endl;
                ROS_INFO_STREAM(buf);
                send_frame.len = _can_pointer->ParseCanData(data, send_frame.data);
            }
        }
    }
}
/*********
 * @brief 电机位置-速度-前馈力矩控制，单位是0.00001圈
 * @param kp 电机Kp缩小系数，范围是0-1.0，真实的Kp为此参数乘以电机设定的Kp，Kd同理。
 *********/
void LD_Motor_Ctl::set_position_32_kpkd(int ID, int32_t pos, int32_t vel, int32_t torque, float kp, float kd)
{

    *(int32_t *)&cmd[5] = pos;
    *(int32_t *)&cmd[9] = vel;
    *(int32_t *)&cmd[13] = torque;
    *(int16_t *)&cmd[19] = (int16_t)(kp * 32767);
    *(int16_t *)&cmd[21] = (int16_t)(kd * 32767);
    fdcan_push_queue(0x8000 | ID, (uint8_t *)cmd, sizeof(cmd));
}

LD_Motor_Ctl::~LD_Motor_Ctl()
{
}

#endif