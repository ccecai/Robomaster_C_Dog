#ifndef __CAN_REC_H__
#define __CAN_REC_H__

#include "can.h"
#include "odrive.h"

typedef struct
{
    int16_t set_voltage;          //电压
    uint16_t angle;          //转子角度 abs angle_now range:[0,8191]
    int16_t speed;          //转子速度
    int16_t torque_current;       //扭矩（以电流值为单位）
    uint8_t temp;                 //电机温度
    int32_t total_angle;          //转子转过的总角度
    int16_t total_cnt;            //转子转过的总圈数
    uint16_t offset_angle;        //上电时的转子位置（初始位置）
    uint16_t last_angle;          //abs angle_now range:[0,8191]
    uint32_t msg_cnt;              //消息计数值，收到一次就+1
}moto_info_t;

union union_float
{
    uint8_t data_8[4];
    float data_32;
};

#define FW 0x201
#define LW 0x202
#define RW 0x203
#define BW 0x204
#define Code 0x300
#define Back 0x301
#define Close 0x302
//电机属性宏
#define M3508_CURRENT_MAX   10000
#define M6020_CURRENT_MAX   30000
#define M2006_CURRENT_MAX   10000

//电机反馈报文接收相关宏
#define MOTOR_MAX_NUM 8

#define ID_PAW 0x206
#define PAW (6-1)

#define ID_RISE 0x203
#define RISE (3-1)


//全局变量声明
extern moto_info_t motor_info[MOTOR_MAX_NUM];
extern int Locker_ID,Back_flag,Close_flag;
//用户函数声明
void my_can_filter_init_recv_all(CAN_HandleTypeDef *_hcan);
void can1_filter_init();
void can2_filter_init();
void set_current(CAN_HandleTypeDef *_hcan, int16_t id_range, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void motor_info_record(moto_info_t *ptr, uint8_t *data);
void motor_info_init();
void SendWheelData(CAN_HandleTypeDef *_hcan, uint32_t wheel_id, float wheel_spd, float wheel_direction);

#endif /* __CAN_REC_H__ */