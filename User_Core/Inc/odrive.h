#ifndef __ODRIVE__H
#define __ODRIVE__H

#include "main.h"


typedef union
{
	uint8_t data_8[4];
	float   data_f;
    int32_t data_int;
    uint32_t data_uint;
}union_32;

typedef union
{
    uint8_t data_u8[8];
    float   data_f[2];
    int data_i32[2];
    uint32_t data_u32[2];
}union_64;

#define Axis1_ID 0x020		//1<<5
#define Axis2_ID 0x040		//2<<5
#define Axis3_ID 0x060		//3<<5
#define Axis4_ID 0x080		//4<<5
#define Axis5_ID 0x0A0		//5<<5
#define Axis6_ID 0x0C0		//6<<5
#define Axis7_ID 0x0E0		//7<<5
#define Axis8_ID 0x100		//8<<5

#define Axis0_Get_Motor_Error 		 Axis0_ID + 0x002
#define Axis1_Get_Motor_Error 		 Axis1_ID + 0x002

#define Axis0_Get_Encoder_Error  	 Axis0_ID + 0x004
#define Axis1_Get_Encoder_Error 	 Axis1_ID + 0x004

#define Set_Axis0_Requested_State    Axis0_ID + 0x007
#define Set_Axis1_Requested_State    Axis1_ID + 0x007

#define Set_Axis0_Controller_Modes   Axis0_ID + 0x00B
#define Set_Axis1_Controller_Modes   Axis1_ID + 0x00B

#define Set_Axis0_Set_Input_Pos      Axis0_ID + 0x00C
#define Set_Axis1_Set_Input_Pos      Axis1_ID + 0x00C

#define Set_Axis0_Set_Input_Vel      Axis0_ID + 0x00D
#define Set_Axis1_Set_Input_Vel      Axis1_ID + 0x00D

#define AXIS_STATE_UNDEFINED  				0
#define AXIS_STATE_IDLE  					1
#define AXIS_STATE_STARTUP_SEQUENCE  		2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE   	3
#define AXIS_STATE_MOTOR_CALIBRATION  			4
#define AXIS_STATE_ENCODER_INDEX_SEARCH  		6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION  	7
#define AXIS_STATE_CLOSED_LOOP_CONTROL  		8
#define AXIS_STATE_LOCKIN_SPIN   				9
#define AXIS_STATE_ENCODER_DIR_FIND  		    10
#define AXIS_STATE_HOMING  						11
#define AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION    12
#define AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION  		13

#define CONTROL_MODE_VOLTAGE_CONTROL  								0
#define CONTROL_MODE_TORQUE_CONTROL   								1
#define CONTROL_MODE_VELOCITY_CONTROL   							2
#define CONTROL_MODE_POSITION_CONTROL    							3

#define INPUT_MODE_INACTIVE    										0
#define INPUT_MODE_PASSTHROUGH    									1
#define INPUT_MODE_VEL_RAMP    										2
#define INPUT_MODE_POS_FILTER    									3
#define INPUT_MODE_MIX_CHANNELS   									4
#define INPUT_MODE_TRAP_TRAJ    									5
#define INPUT_MODE_TORQUE_RAMP    									6
#define INPUT_MODE_MIRROR    										7
#define INPUT_MODE_Tuning    										8


void Odrive_Axis0_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque);
void Odrive_Axis1_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque);

#endif
