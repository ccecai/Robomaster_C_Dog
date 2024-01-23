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

void Odrive_Axis0_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t mbox;         //����ʹ�õ���can����

//    TxHeader.StdId = Set_Axis0_Set_Input_Vel;
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



