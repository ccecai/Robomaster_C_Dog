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
    Get_Target(0,PI);
    SetCoupledThetaPositionAll();

}

void LieDown_Posture(void)
{
    for(int i = 1;i < 9;i ++)
    {
        AngleWant_MotorX[i] = 0;
    }
}
//测试用Trot步态
void Test_Move(void)
{
    ChangeGainOfPID(60,0,0.6f,0);
    gait_detached(state_detached_params[9],Leg1_Delay,Leg2_Delay,Leg3_Delay,Leg4_Delay,Forward,Forward,Forward,Forward);
}
//实际运行Trot步态
void Trot(float direction,int8_t kind)
{
    switch(kind)
    {
        case 0://大步Trot
            NewHeartbeat = 5;
            ChangeGainOfPID(60,0,0.6f,0);
            gait_detached(state_detached_params[0],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        case 1://小步Trot
            NewHeartbeat = 6;
            ChangeGainOfPID(7.0f,0,0.6f,0);
//            YawControl(yawwant, &state_detached_params[11],direction);
            gait_detached(state_detached_params[11],0.0f, 0.5f, 0.5f, 0.0f,
                          direction,direction,direction,direction);
            break;
        default:
            break;
    }
}

//慢步
void Walk(float direction,uint8_t speed)
{
//    PID_Set_KP_KI_KD(&Yaw_PID_Loop,1.7,0,1.0);
//    YawControl(yawwant,&state_detached_params[1],direction);
//    Yaw_PID_Loop.SumError_limit = 2500;Yaw_PID_Loop.Output_limit = 45;
    NewHeartbeat = 4;
    ChangeGainOfPID(60,0,0.6f,0);
    gait_detached(state_detached_params[1],0.0,0.75,0.5,0.25,direction,direction,direction,direction);
}
//转弯步态
void Turn(int state_flag)
{
    NewHeartbeat = 5;
//    ChangeGainOfPID(60,0,0.6f,0);
    switch (state_flag) {
        case 'l':
            state_detached_params[8].detached_params_0.step_length = -20.0f;
            state_detached_params[8].detached_params_1.step_length = -20.0f;
            state_detached_params[8].detached_params_2.step_length = 20.0f;
            state_detached_params[8].detached_params_3.step_length = 20.0f;
            break;
        case 'r':
            state_detached_params[8].detached_params_0.step_length = 20.0f;
            state_detached_params[8].detached_params_1.step_length = 20.0f;
            state_detached_params[8].detached_params_2.step_length = -20.0f;
            state_detached_params[8].detached_params_3.step_length = -20.0f;
            break;
        default:
            break;
    }
    gait_detached(state_detached_params[8],  0.0f, 0.5f, 0.5f, 0.0f,
                  1.0f, 1.0f, 1.0f,1.0f);
}

void Dog_Posture(void)
{
    switch (gpstate) {
        case HALT:
            StandUp_Posture();
            break;
        case END:
            LieDown_Posture();
            break;
        case TURN_LEFT:
            Turn('l');
            break;
        case TURN_RIGHT:
            Turn('r');
            break;
        case MARCH:
            Walk(Forward,0);
            break;
        case MARCH_BACK:
            Walk(Backward,0);
            break;
        case Tro:
            Trot(Forward,1);
        default:
            break;
    }
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