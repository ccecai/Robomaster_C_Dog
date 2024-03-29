//
// Created by 1 on 2023-11-07.
//

#ifndef MY_SCUDOG_ATTITUDE_SLOVE_H
#define MY_SCUDOG_ATTITUDE_SLOVE_H

#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

//#define offset_front_0 0.1320f
//#define offset_front_1 0.5346f
//#define offset_back_0  0.1320f//(-121.9f)
//#define offset_back_1  0.5346f//207.2f
#define offset_front_0 0.78f
#define offset_front_1 1.22f
#define offset_back_0  0.78f//(-121.9f)
#define offset_back_1  1.22f//207.2f
#define PI 3.1415926535f
//大小腿长
#define L1 15.0f //????100mm
#define L2 27.85f//?ó??200mm
//腿长限位
#define LegLenthExtremeMax 45.0f //
#define LegLenthMax 40.0f //
#define LegLenthMin 20.0f //
#define LegStandLenth 18.0f //
#define LegSquatLenth 11.2f //
//腿长限位
#define StepLenthMin 0.0f
#define StepLenthMax (2*LegLenthExtremeMax*0.866f*0.9f) //大小大概在45cm
#define StepLenthMax_Half (LegLenthExtremeMax*0.866f*0.9f)
//状态数上限配置
#define StatesMaxNum 20


typedef struct
{
    float stance_height ;//狗身到地面的距离
    float step_length ;//一步的距离
    float up_amp ;//上部振幅
    float down_amp ;//下部振幅
    float flight_percent ;//摆动相百分比
    float freq ;//一步的频率
} GaitParams;
//四条腿的结构体参数

typedef struct
{
    uint8_t GaitID;
    GaitParams detached_params_0;
    GaitParams detached_params_1;
    GaitParams detached_params_2;
    GaitParams detached_params_3;

} DetachedParam;

extern float Leg1_Delay,Leg2_Delay,Leg3_Delay,Leg4_Delay;
extern DetachedParam state_detached_params[StatesMaxNum];
extern DetachedParam StateDetachedParams_Copy[];
extern float AngleWant_MotorX[9];
extern float x,y;

void Get_Target(float theta1,float theta2);
void SetCoupledThetaPositionAll(void);
void SetCoupledThetaPosition(int LegId);
void Output_Angle(void);
void gait_detached(	DetachedParam d_params,
                       float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
                       float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction);
void Change_SinStateDetachedParams(DetachedParam *State,int8_t id,int8_t legid,float stance_height,float step_length,
                                   float up_amp,float down_amp,float flight_percent,float freq);
void CartesianToTheta(void);
void SinTrajectory (float t,GaitParams params, float gaitOffset,float leg_diretion,float angle,int LegId);
void CoupledMoveLeg(float t, GaitParams params,float gait_offset, float leg_direction, int LegId, float angle);
void AttitudeControl(float roll_set,float pitch_set,float yaw_set,DetachedParam *State_Detached_Params,int direction);
void YawControl(float yaw_set,DetachedParam *State_Detached_Params,int direction);

#endif //MY_SCUDOG_ATTITUDE_SLOVE_H
