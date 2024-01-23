#include "odrive.h"
#include "can.h"
/*
����Ϊstm32F4xxϵ��оƬ����ODRIVE��CAN����ͨ��API

��Ҫע����ǣ�CAN���������õ���1Mbps��ÿ��ָ������100λ/֡������һ��ָ�����ɵ�ʱ��Ӧ������100us��������

�ϲ�������ʱ��Ҫע��ϵͳ��������
�ϲ�������ʱ��Ҫע��ϵͳ��������
�ϲ�������ʱ��Ҫע��ϵͳ��������
*/

union_32 union_32f;
union_16 union_16f;

void Odrive_Axis2_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis2_Set_Input_Vel;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

	union_32f.data_f = Input_Vel;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_32f.data_f = Torque;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];
    TxData[6] = union_32f.data_8[2];
    TxData[7] = union_32f.data_8[3];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_Axis1_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis1_Set_Input_Vel;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_32f.data_f = Input_Vel;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_32f.data_f = Torque;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];
    TxData[6] = union_32f.data_8[2];
    TxData[7] = union_32f.data_8[3];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_Axis3_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis3_Set_Input_Vel;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_32f.data_f = Input_Vel;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_32f.data_f = Torque;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];
    TxData[6] = union_32f.data_8[2];
    TxData[7] = union_32f.data_8[3];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
void Odrive_Axis4_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis4_Set_Input_Vel;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_32f.data_f = Input_Vel;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_32f.data_f = Torque;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];
    TxData[6] = union_32f.data_8[2];
    TxData[7] = union_32f.data_8[3];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
void Odrive_Axis5_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis5_Set_Input_Vel;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_32f.data_f = Input_Vel;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_32f.data_f = Torque;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];
    TxData[6] = union_32f.data_8[2];
    TxData[7] = union_32f.data_8[3];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
void Odrive_Axis6_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis6_Set_Input_Vel;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_32f.data_f = Input_Vel;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_32f.data_f = Torque;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];
    TxData[6] = union_32f.data_8[2];
    TxData[7] = union_32f.data_8[3];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
void Odrive_Axis7_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis7_Set_Input_Vel;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_32f.data_f = Input_Vel;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_32f.data_f = Torque;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];
    TxData[6] = union_32f.data_8[2];
    TxData[7] = union_32f.data_8[3];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}
void Odrive_Axis8_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis8_Set_Input_Vel;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_32f.data_f = Input_Vel;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_32f.data_f = Torque;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];
    TxData[6] = union_32f.data_8[2];
    TxData[7] = union_32f.data_8[3];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_Axis1_Set_Input_Position(CAN_HandleTypeDef *_hcan, float Input_Pos,int16_t Vel_FF,int16_t Torque_FF)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis1_Set_Input_Pos;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_16f.data_f = Input_Pos;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_16f.data_v = Vel_FF;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];

    union_16f.data_t = Torque_FF;
    TxData[6] = union_32f.data_8[0];
    TxData[7] = union_32f.data_8[1];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_Axis2_Set_Input_Position(CAN_HandleTypeDef *_hcan, float Input_Pos,int16_t Vel_FF,int16_t Torque_FF)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis2_Set_Input_Pos;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_16f.data_f = Input_Pos;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_16f.data_v = Vel_FF;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];

    union_16f.data_t = Torque_FF;
    TxData[6] = union_32f.data_8[0];
    TxData[7] = union_32f.data_8[1];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_Axis3_Set_Input_Position(CAN_HandleTypeDef *_hcan, float Input_Pos,int16_t Vel_FF,int16_t Torque_FF)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis3_Set_Input_Pos;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_16f.data_f = Input_Pos;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_16f.data_v = Vel_FF;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];

    union_16f.data_t = Torque_FF;
    TxData[6] = union_32f.data_8[0];
    TxData[7] = union_32f.data_8[1];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_Axis4_Set_Input_Position(CAN_HandleTypeDef *_hcan, float Input_Pos,int16_t Vel_FF,int16_t Torque_FF)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis4_Set_Input_Pos;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_16f.data_f = Input_Pos;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_16f.data_v = Vel_FF;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];

    union_16f.data_t = Torque_FF;
    TxData[6] = union_32f.data_8[0];
    TxData[7] = union_32f.data_8[1];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_Axis5_Set_Input_Position(CAN_HandleTypeDef *_hcan, float Input_Pos,int16_t Vel_FF,int16_t Torque_FF)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis5_Set_Input_Pos;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_16f.data_f = Input_Pos;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_16f.data_v = Vel_FF;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];

    union_16f.data_t = Torque_FF;
    TxData[6] = union_32f.data_8[0];
    TxData[7] = union_32f.data_8[1];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_Axis6_Set_Input_Position(CAN_HandleTypeDef *_hcan, float Input_Pos,int16_t Vel_FF,int16_t Torque_FF)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis6_Set_Input_Pos;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_16f.data_f = Input_Pos;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_16f.data_v = Vel_FF;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];

    union_16f.data_t = Torque_FF;
    TxData[6] = union_32f.data_8[0];
    TxData[7] = union_32f.data_8[1];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_Axis7_Set_Input_Position(CAN_HandleTypeDef *_hcan, float Input_Pos,int16_t Vel_FF,int16_t Torque_FF)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis7_Set_Input_Pos;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_16f.data_f = Input_Pos;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_16f.data_v = Vel_FF;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];

    union_16f.data_t = Torque_FF;
    TxData[6] = union_32f.data_8[0];
    TxData[7] = union_32f.data_8[1];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_Axis8_Set_Input_Position(CAN_HandleTypeDef *_hcan, float Input_Pos,int16_t Vel_FF,int16_t Torque_FF)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = Set_Axis8_Set_Input_Pos;
    TxHeader.IDE = 0;
    TxHeader.RTR = 0;
    TxHeader.DLC = 8;

    union_16f.data_f = Input_Pos;
    TxData[0] = union_32f.data_8[0];
    TxData[1] = union_32f.data_8[1];
    TxData[2] = union_32f.data_8[2];
    TxData[3] = union_32f.data_8[3];

    union_16f.data_v = Vel_FF;
    TxData[4] = union_32f.data_8[0];
    TxData[5] = union_32f.data_8[1];

    union_16f.data_t = Torque_FF;
    TxData[6] = union_32f.data_8[0];
    TxData[7] = union_32f.data_8[1];

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_val_output(uint8_t id)
{
    switch(id)
    {
        case 1:
            Odrive_Axis1_Set_Input_Vel(&hcan1,AngleLoop[1].Out_put,0.0f);
            osDelay(1);
            break;
        case 2:
            Odrive_Axis2_Set_Input_Vel(&hcan1,AngleLoop[2].Out_put,0.0f);
            osDelay(1);
            break;
        case 3:
            Odrive_Axis3_Set_Input_Vel(&hcan1,AngleLoop[3].Out_put,0.0f);
            osDelay(1);
            break;
        case 4:
            Odrive_Axis4_Set_Input_Vel(&hcan1,AngleLoop[4].Out_put,0.0f);
            osDelay(1);
            break;
        case 5:
            Odrive_Axis5_Set_Input_Vel(&hcan1,AngleLoop[5].Out_put,0.0f);
            osDelay(1);
            break;
        case 6:
            Odrive_Axis6_Set_Input_Vel(&hcan1,AngleLoop[6].Out_put,0.0f);
            osDelay(1);
            break;
        case 7:
            Odrive_Axis7_Set_Input_Vel(&hcan1,AngleLoop[7].Out_put,0.0f);
            osDelay(1);
            break;
        case 8:
            Odrive_Axis8_Set_Input_Vel(&hcan1,AngleLoop[8].Out_put,0.0f);
            osDelay(1);
            break;
        default:
            break;
    }
}

void Odrive_Postion_output(uint8_t id)
{
    switch(id)
    {
        case 1:
            Odrive_Axis1_Set_Input_Position(&hcan1,AngleWant_MotorX[1],1,0);
            osDelay(1);
            break;
        case 2:
            Odrive_Axis2_Set_Input_Position(&hcan1,AngleWant_MotorX[2],1,0);
            osDelay(1);
            break;
        case 3:
            Odrive_Axis3_Set_Input_Position(&hcan1,AngleWant_MotorX[3],1,0);
            osDelay(1);
            break;
        case 4:
            Odrive_Axis4_Set_Input_Position(&hcan1,AngleWant_MotorX[4],1,0);
            osDelay(1);
            break;
        case 5:
            Odrive_Axis5_Set_Input_Position(&hcan1,AngleWant_MotorX[5],1,0);
            osDelay(1);
            break;
        case 6:
            Odrive_Axis6_Set_Input_Position(&hcan1,AngleWant_MotorX[6],1,0);
            osDelay(1);
            break;
        case 7:
            Odrive_Axis7_Set_Input_Position(&hcan1,AngleWant_MotorX[7],1,0);
            osDelay(1);
            break;
        case 8:
            Odrive_Axis8_Set_Input_Position(&hcan1,AngleWant_MotorX[8],1,0);
            osDelay(1);
            break;
        default:
            break;
    }
}

void AllMotor_valOutput(void)
{
    for(uint8_t Id = 1;Id < 9;Id ++)
    {
        Odrive_val_output(Id);
    }
}

void AllMotor_PositionOutput(void)
{
    for(uint8_t Id = 1;Id < 9;Id ++)
    {
        Odrive_Postion_output(Id);
    }
}