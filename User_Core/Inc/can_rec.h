#ifndef __CAN_REC_H__
#define __CAN_REC_H__
#include "can.h"

typedef struct
{
    int16_t set_voltage;          //��ѹ
    uint16_t angle;          //ת�ӽǶ� abs angle_now range:[0,8191]
    int16_t speed;          //ת���ٶ�
    int16_t torque_current;       //Ť�أ��Ե���ֵΪ��λ��
    uint8_t temp;                 //����¶�
    int32_t total_angle;          //ת��ת�����ܽǶ�
    int16_t total_cnt;            //ת��ת������Ȧ��
    uint16_t offset_angle;        //�ϵ�ʱ��ת��λ�ã���ʼλ�ã�
    uint16_t last_angle;          //abs angle_now range:[0,8191]
    uint32_t msg_cnt;              //��Ϣ����ֵ���յ�һ�ξ�+1
}moto_info_t;

typedef union
{
    uint8_t data_8[8];
    uint16_t data_16[4];
    int32_t data_int[2];
    uint32_t data_uint[2];
    float data_f[2];
}union_64;

typedef union
{
    uint8_t data_8[4];
    uint16_t data_16[2];
    int32_t data_int;
    uint32_t data_uint;
    float data_f;
}union_32;

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
//������Ժ�
#define M3508_CURRENT_MAX   10000
#define M6020_CURRENT_MAX   30000
#define M2006_CURRENT_MAX   10000

//����������Ľ�����غ�
#define MOTOR_MAX_NUM 8

#define ID_PAW 0x206
#define PAW (6-1)

#define ID_RISE 0x203
#define RISE (3-1)


//ȫ�ֱ�������
extern moto_info_t motor_info[MOTOR_MAX_NUM];
extern int Locker_ID,Back_flag,Close_flag;
//�û���������
void my_can_filter_init_recv_all(CAN_HandleTypeDef *_hcan);
void can1_filter_init();
void can2_filter_init();
void set_current(CAN_HandleTypeDef *_hcan, int16_t id_range, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void motor_info_record(moto_info_t *ptr, uint8_t *data);
void motor_info_init();
void SendWheelData(CAN_HandleTypeDef *_hcan, uint32_t wheel_id, float wheel_spd, float wheel_direction);

#endif /* __CAN_REC_H__ */