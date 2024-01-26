//
// Created by 1 on 2023-11-07.
//
#include "Attitude_Task.h"

float TargetAngle1 = 0,TargetAngle2 = 0;
enum GPStates gpstate = STOP;
enum DPStates dpstate = NONE;
float NewHeartbeat = 0;//心跳值
//全局姿态控制
int Global_IMU_Control = 0;

void StandUp_Posture(void)
{
    AllMotorSpeedLimit(SpeedNormal);
    Get_Target(0,PI);
    SetCoupledThetaPositionAll();
}

void LieDown_Posture(void)
{
    AllMotorSpeedLimit(SpeedMin);
    for(int i = 1;i < 9;i ++)
    {
        AngleWant_MotorX[i] = 0;
    }
}
void MarkingTime(void)
{
    AllMotorSpeedLimit(SpeedFast);
    ChangeGainOfPID(3.8f,0,0.6f,0);
//    ChangeYawOfPID(50.0f,0.5f,2500.0f,10.0f);
//    YawControl(yawwant, &state_detached_params[2], 1.0f);
    gait_detached(state_detached_params[2],0.0,0.75,0.5,0.25,
                  1.0f,1.0f,1.0f,1.0f);
}
//实际运行Trot步态
void Trot(float direction,int8_t kind)
{
    switch(kind)
    {
        case 0://小步Trot
            AllMotorSpeedLimit(SpeedNormal);
            NewHeartbeat = 6;
            ChangeGainOfPID(3.5f,0,0.6f,0);
//            ChangeYawOfPID(200.0f,2.0f,3000.0f,10.0f);
//            YawControl(yawwant, &state_detached_params[4], direction);
            gait_detached(state_detached_params[4],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        case 1://大步Trot
            AllMotorSpeedLimit(SpeedFast);
            NewHeartbeat = 5;
            ChangeGainOfPID(3.8f,0,0.6f,0);
//            ChangeYawOfPID(1000.0f,10.0f,4000.0f,15.0f);
//            YawControl(yawwant, &state_detached_params[1], direction);
            gait_detached(state_detached_params[1],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        default:
            break;
    }
}

//慢步
void Walk(float direction,uint8_t speed)
{
    NewHeartbeat = 4;
    AllMotorSpeedLimit(SpeedFast);
    ChangeGainOfPID(3.5f,0,0.6f,0);
//    ChangeYawOfPID(100.0f,0.5f,2500.0f,10.0f);
//    YawControl(yawwant, &state_detached_params[3], direction);
    gait_detached(state_detached_params[3],0.0,0.75,0.5,0.25,direction,direction,direction,direction);
}
//转弯步态
void Turn(int state_flag,int speed_flag)
{
    float length;

    if(speed_flag == 'f')
    {
        length = 20.0f;
        state_detached_params[0].detached_params_0.freq = 4.0f;
        state_detached_params[0].detached_params_1.freq = 4.0f;
        state_detached_params[0].detached_params_2.freq = 4.0f;
        state_detached_params[0].detached_params_3.freq = 4.0f;
    }
    else if(speed_flag == 's')
    {
        length = 5.0f;
        state_detached_params[0].detached_params_0.freq = 1.5f;
        state_detached_params[0].detached_params_1.freq = 1.5f;
        state_detached_params[0].detached_params_2.freq = 1.5f;
        state_detached_params[0].detached_params_3.freq = 1.5f;
    }

    NewHeartbeat = 5;
    AllMotorSpeedLimit(SpeedFast);
    ChangeGainOfPID(4.0f,0,0.6f,0);
    switch (state_flag) {
        case 'l':
            state_detached_params[0].detached_params_0.step_length = -length;
            state_detached_params[0].detached_params_1.step_length = -length;
            state_detached_params[0].detached_params_2.step_length = length;
            state_detached_params[0].detached_params_3.step_length = length;
            break;
        case 'r':
            state_detached_params[0].detached_params_0.step_length = length;
            state_detached_params[0].detached_params_1.step_length = length;
            state_detached_params[0].detached_params_2.step_length = -length;
            state_detached_params[0].detached_params_3.step_length = -length;
            break;
        default:
            break;
    }
    gait_detached(state_detached_params[0],  0.0f, 0.5f, 0.5f, 0.0f,
                  1.0f, 1.0f, 1.0f,1.0f);
}

void Up_and_Down(float val)
{
    Get_Target(0,PI);
    AngleWant_MotorX[1]=TargetAngle1-offset_front_0 + val;
    AngleWant_MotorX[2]=TargetAngle2-offset_front_1 + val;//+10.0f
    AngleWant_MotorX[3]=TargetAngle1-offset_back_0 + val;//+5.0f
    AngleWant_MotorX[4]=TargetAngle2-offset_back_1 + val;
    AngleWant_MotorX[5]=-TargetAngle2+offset_front_1 + val;//-4.0f
    AngleWant_MotorX[6]=-TargetAngle1+offset_front_0 + val;
    AngleWant_MotorX[7]=-TargetAngle2+offset_back_1 + val;
    AngleWant_MotorX[8]=-TargetAngle1+offset_back_0 + val;

}