# -*- coding: UTF-8 -*-
import Livelybot_Driver_bind
import numpy
import time
import asyncio
import math
import numpy as np

my_Driver = Livelybot_Driver_bind.Livelybot_Driver("/dev/spidev4.1")


# if __name__== "__main__" :
#     #设置can1线id为1的电机的位置
#     my_Driver.set_motor_position(0x11, 0)

#     #发送指令
#     my_Driver.spi_send()

#     #获得can1线id为1的电机的状态
#     motor_state = my_Driver.get_motor_state(0x11)
#     print("motor_pos:", motor_state.position)
#     footsensor = my_Driver.get_footsensor_data(0x01)
#     print("1:", footsensor[0])
#     #获得imu的数值
#     imu_data = my_Driver.get_imu_data()
#     print("accX:, accY:, accZ:angVelX:%d, angVelY:%d, angVelZ:%d\nangle_roll:%d, angle_pitch:%d, angle_yaw:%d\nmagX:%d, magY:%d, magZ:%d\n",
#     imu_data.accX, imu_data.accY, imu_data.accZ, imu_data.angVelX, imu_data.angVelY, imu_data.angVelZ, 
#     imu_data.angle_roll, imu_data.angle_pitch, imu_data.angle_yaw, imu_data.magX, imu_data.magY, imu_data.magZ)


async def main():
    # motor = moteus.Controller(id=3)
    # state=await motor.set_position(position=math.nan,kp_scale=0.0,kd_scale=0.0,query=True)
    # startPos=state.values[moteus.Register.POSITION]
    # print(startPos)
    kp = 0.3
    kd = 0.7
    dataset = []
    with open('前进.log', 'r') as log_file:
        for line in log_file:
            dataset.append(line.strip().split(' '))
    dataset = np.array(dataset, dtype=float)
    dataset = dataset/1000.0
    # print(dataset)
    ids = [1, 2, 7, 9, 8, 10, 10, 6, 4, 3] # 左,右腿的 abad hip knee
    print("20")
    motors = []
    for item in ids:
        motors.append(moteus.Controller(item))
    # for i in range(6):
    #     state=await motors[i].set_position(position=math.nan,velocity=0,feedforward_torque=0,kp_scale=0,kd_scale=0,query=True)
    #     print(state.values[moteus.Register.POSITION])
    yaw_pos = [0.0, 0.0]
    yaw_motor = [moteus.Controller(5), moteus.Controller(10)]
    print("29")
    for i in range(2):
        state=await yaw_motor[i].set_position(position=math.nan,kp_scale=kp*0,kd_scale=kd*0,query=True)
        #print(state)
        yaw_pos[i]=state.values[moteus.Register.POSITION] 

    print("35")
    startPos = np.zeros(10)
    offset = np.zeros(10)
    print("37")
    for i in range(10):
        state=await motors[i].set_position(position=math.nan,kp_scale=kp*0,kd_scale=kd*0,query=True)
        startPos[i]=state.values[moteus.Register.POSITION] 
        offset[i]=state.values[moteus.Register.POSITION] 
    print(startPos)
    
    duration = 100 
    

    await yaw_motor[0].set_position(position=yaw_pos[0],velocity=0,feedforward_torque=0,kp_scale=kp,kd_scale=kd)
    await yaw_motor[1].set_position(position=yaw_pos[1],velocity=0,feedforward_torque=0,kp_scale=kp,kd_scale=kd)

    print("50")
    for step in range(1000):
        # print(step)
        q = dataset[step] 
        # for i in range(6):
        #     q[i] = q[i] / 6.28
        q_cmd = np.zeros(10)
        # q_cmd = -q
        q_cmd[2]=-q[0]/6.28 * 0.0
        q_cmd[3]=-q[1]/6.28
        q_cmd[4]=(q[1]+q[2])/6.28
        q_cmd[0]=-(q[1]+q[2])/6.28*20
        q_cmd[1]=-q_cmd[0]

        q_cmd[7]=-q[3]/6.28 * 0.0
        q_cmd[8]=q[4]/6.28
        q_cmd[9]=-(q[4]+q[5])/6.28
        q_cmd[5]=(q[4]+q[5])/6.28*20
        q_cmd[6]=-q_cmd[5]
        if step != 0:
            duration = 4
        for n in range(duration):
                rate=n/duration
                for i in range(10):
                    # if i != 1:
                    #     continue
                    pos=startPos[i]+(q_cmd[i]-startPos[i])*rate
                    print(pos)
                    if i==0 or i==1 or i==5 or i==6:
                        # pos += offset[i]
                        continue
                    state = await motors[i].set_position(position=pos,velocity=0,feedforward_torque=0,kp_scale=kp,kd_scale=kd)
                    # print(state)
                    startPos[i]= pos
                print()
                await asyncio.sleep(0.01)