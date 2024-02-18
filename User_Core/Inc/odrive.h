#ifndef __ODRIVE__H
#define __ODRIVE__H

#include "main.h"

/**
 * 得到编码器反馈电机速度与位置的结构体
 */

//测试改变float成int32_t会不会解决bug

typedef union
{
    uint8_t data_8[8];
    float data_pos;
    float data_vel;
}Feedback;

typedef union
{
    uint8_t data_v8[4];
    float data_v;
}vfeedback;
/**
 * 输出速度模式所用的结构体
 */

typedef union
{
	uint8_t data_8[4];
	float   data_f;
    int32_t data_int;
    uint32_t data_uint;
}union_32;

/**
 * 输出位置模式所用的结构体
 */

typedef union
{
    uint8_t data_8[4];
    float data_f;
    int16_t data_v;
    int16_t data_t;
}union_16;

/**
 * 设置电机状态的结构体
 */

typedef union
{
    uint8_t data_8[8];
    uint32_t data_id;
}SetState;

/**
 * 设置电机控制模式的结构体
 */

typedef union
{
    uint8_t data_8[4];
    uint32_t data_c;
    uint32_t data_i;
}SetMode;

/**
 * RM电机接收所用的结构体
 */

typedef union
{
    uint8_t data_u8[8];
    float   data_f[2];
    int data_i32[2];
    uint32_t data_u32[2];

}union_64;

/*********
 *
 * 在标准CAN通信中Odrive驱动电机时CAN ID的部分中bit10-bit5的部分为存放电机ID的部分，
 * 即为下方的宏定义，其中Axis1_ID 对应的是电机id为1时，id移位五位后的结果，
 * 注意：这里的id为1是十进制而非十六进制。
 *
 *********/

#define Axis1_ID 0x020		//1<<5
#define Axis2_ID 0x040		//2<<5
#define Axis3_ID 0x060		//3<<5
#define Axis4_ID 0x080		//4<<5
#define Axis5_ID 0x0A0		//5<<5
#define Axis6_ID 0x0C0		//6<<5
#define Axis7_ID 0x0E0		//7<<5
#define Axis8_ID 0x100		//8<<5

/**
 * 让八个电机停止转动，是MCU发给电机
 */

#define Axis1_Get_Motor_Error 		 Axis1_ID + 0x002
#define Axis2_Get_Motor_Error 		 Axis2_ID + 0x002
#define Axis3_Get_Motor_Error 		 Axis3_ID + 0x002
#define Axis4_Get_Motor_Error 		 Axis4_ID + 0x002
#define Axis5_Get_Motor_Error 		 Axis5_ID + 0x002
#define Axis6_Get_Motor_Error 		 Axis6_ID + 0x002
#define Axis7_Get_Motor_Error 		 Axis7_ID + 0x002
#define Axis8_Get_Motor_Error 		 Axis8_ID + 0x002

/**
 * 检测八个电机的编码器是否出问题，是电机发给MCU
 */

#define Axis1_Get_Encoder_Error 	 Axis1_ID + 0x004
#define Axis2_Get_Encoder_Error 	 Axis2_ID + 0x004
#define Axis3_Get_Encoder_Error 	 Axis3_ID + 0x004
#define Axis4_Get_Encoder_Error 	 Axis4_ID + 0x004
#define Axis5_Get_Encoder_Error 	 Axis5_ID + 0x004
#define Axis6_Get_Encoder_Error 	 Axis6_ID + 0x004
#define Axis7_Get_Encoder_Error 	 Axis7_ID + 0x004
#define Axis8_Get_Encoder_Error 	 Axis8_ID + 0x004

/**
 * 设置八个电机的设置模式，是MCU发给电机
 * 我们应当只使用=8时候的闭环控制模式
 */

#define Set_Axis1_Requested_State 	 Axis1_ID + 0x007
#define Set_Axis2_Requested_State 	 Axis2_ID + 0x007
#define Set_Axis3_Requested_State 	 Axis3_ID + 0x007
#define Set_Axis4_Requested_State 	 Axis4_ID + 0x007
#define Set_Axis5_Requested_State 	 Axis5_ID + 0x007
#define Set_Axis6_Requested_State 	 Axis6_ID + 0x007
#define Set_Axis7_Requested_State 	 Axis7_ID + 0x007
#define Set_Axis8_Requested_State 	 Axis8_ID + 0x007

/**
 * 检测八个电机的编码器的速度与位置
 */

#define Get_Axis1_Encoder            Axis1_ID + 0x009
#define Get_Axis2_Encoder            Axis2_ID + 0x009
#define Get_Axis3_Encoder            Axis3_ID + 0x009
#define Get_Axis4_Encoder            Axis4_ID + 0x009
#define Get_Axis5_Encoder            Axis5_ID + 0x009
#define Get_Axis6_Encoder            Axis6_ID + 0x009
#define Get_Axis7_Encoder            Axis7_ID + 0x009
#define Get_Axis8_Encoder            Axis8_ID + 0x009

/**
 * 设置八个电机的控制模式，是MCU发给电机
 * =0时为电压控制模式
 * =1时为力矩控制模式
 * =2时为速度控制模式
 * =3时为位置控制模式
 */

#define Set_Axis1_Controller_Modes   Axis1_ID + 0x00B
#define Set_Axis2_Controller_Modes   Axis2_ID + 0x00B
#define Set_Axis3_Controller_Modes   Axis3_ID + 0x00B
#define Set_Axis4_Controller_Modes   Axis4_ID + 0x00B
#define Set_Axis5_Controller_Modes   Axis5_ID + 0x00B
#define Set_Axis6_Controller_Modes   Axis6_ID + 0x00B
#define Set_Axis7_Controller_Modes   Axis7_ID + 0x00B
#define Set_Axis8_Controller_Modes   Axis8_ID + 0x00B

/**
 *设置目标位置以及前馈速度与前馈力矩
 */

#define Set_Axis1_Set_Input_Pos      Axis1_ID + 0x00C
#define Set_Axis2_Set_Input_Pos      Axis2_ID + 0x00C
#define Set_Axis3_Set_Input_Pos      Axis3_ID + 0x00C
#define Set_Axis4_Set_Input_Pos      Axis4_ID + 0x00C
#define Set_Axis5_Set_Input_Pos      Axis5_ID + 0x00C
#define Set_Axis6_Set_Input_Pos      Axis6_ID + 0x00C
#define Set_Axis7_Set_Input_Pos      Axis7_ID + 0x00C
#define Set_Axis8_Set_Input_Pos      Axis8_ID + 0x00C

/**
 *设置目标速度以及前馈力矩
 */

#define Set_Axis1_Set_Input_Vel      Axis1_ID + 0x00D
#define Set_Axis2_Set_Input_Vel      Axis2_ID + 0x00D
#define Set_Axis3_Set_Input_Vel      Axis3_ID + 0x00D
#define Set_Axis4_Set_Input_Vel      Axis4_ID + 0x00D
#define Set_Axis5_Set_Input_Vel      Axis5_ID + 0x00D
#define Set_Axis6_Set_Input_Vel      Axis6_ID + 0x00D
#define Set_Axis7_Set_Input_Vel      Axis7_ID + 0x00D
#define Set_Axis8_Set_Input_Vel      Axis8_ID + 0x00D

/**
 * 将目标电机的通信方式由CAN转换为USB
 */

#define Turn_Axis1_to_USB             Axis1_ID + 0x1E
#define Turn_Axis2_to_USB             Axis2_ID + 0x1E
#define Turn_Axis3_to_USB             Axis3_ID + 0x1E
#define Turn_Axis4_to_USB             Axis4_ID + 0x1E
#define Turn_Axis5_to_USB             Axis5_ID + 0x1E
#define Turn_Axis6_to_USB             Axis6_ID + 0x1E
#define Turn_Axis7_to_USB             Axis7_ID + 0x1E
#define Turn_Axis8_to_USB             Axis8_ID + 0x1E

/**
 *Odrive state 的所含状态
 */

#define AXIS_STATE_UNDEFINED  				                        0
#define AXIS_STATE_IDLE  					                        1
#define AXIS_STATE_STARTUP_SEQUENCE  		                        2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE   	                    3
#define AXIS_STATE_MOTOR_CALIBRATION  			                    4
#define AXIS_STATE_ENCODER_INDEX_SEARCH  		                    6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION  	                    7
#define AXIS_STATE_CLOSED_LOOP_CONTROL  		                    8
#define AXIS_STATE_LOCKIN_SPIN   				                    9
#define AXIS_STATE_ENCODER_DIR_FIND  		                        10
#define AXIS_STATE_HOMING  						                    11
#define AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION                12
#define AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION  		            13

/**
 * 控制模式所含状态
 */

#define CONTROL_MODE_VOLTAGE_CONTROL  								0
#define CONTROL_MODE_TORQUE_CONTROL   								1
#define CONTROL_MODE_VELOCITY_CONTROL   							2
#define CONTROL_MODE_POSITION_CONTROL    							3

/**
 * 输入模式所含状态
 */

#define INPUT_MODE_INACTIVE    										0
#define INPUT_MODE_PASSTHROUGH    									1
#define INPUT_MODE_VEL_RAMP    										2
#define INPUT_MODE_POS_FILTER    									3
#define INPUT_MODE_MIX_CHANNELS   									4
#define INPUT_MODE_TRAP_TRAJ    									5
#define INPUT_MODE_TORQUE_RAMP    									6
#define INPUT_MODE_MIRROR    										7
#define INPUT_MODE_Tuning    										8

/**
 * 文档中规定最大转速为42+-10%
 * 额定转速为12+-10%
 */

#define SpeedMax 35
#define SpeedFast 25
#define SpeedNormal 15
#define SpeedMin 10
#define ReductionRatio 8

extern Feedback GIM6010[9];
extern float begin_pos[9];

void Odrive_Axis_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque,uint32_t stdid);
void Odrive_Axis_Set_Input_Position(CAN_HandleTypeDef *_hcan, float Input_Pos,int16_t Vel_FF,int16_t Torque_FF,uint32_t stdid);
void Odrive_Set_State(CAN_HandleTypeDef *_hcan, uint32_t State,uint32_t stdid);
void Odrive_Set_ControlMode(CAN_HandleTypeDef *_hcan, uint32_t ContorlMode,uint32_t InputMode,uint32_t stdid);
void Odrive_Encoder_feedback(CAN_HandleTypeDef *_hcan, float Pos,float Vel,uint32_t stdid);
void Odrive_val_output(uint8_t id);
void AllMotor_valOutput(void);
void Odrive_Postion_output(uint8_t id);
void AllMotor_PositionOutput(void);
void Odrive_feedback_record(Feedback *ptr,uint8_t *data);
void Motor_Init(void);
void Odrive_CAN_to_USB(CAN_HandleTypeDef *_hcan,uint32_t stdid);
void AllMotorSpeedLimit(int16_t Limit_Speed);
void Read_beginPos(void);
void Leg_Output(void);

#endif
