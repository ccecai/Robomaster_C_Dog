#include "odrive.h"
#include "can.h"
/*
这是为stm32F4xx系列芯片适配ODRIVE的CAN总线通信API

需要注意的是，CAN波特率配置的是1Mbps，每条指令差不多有100位/帧，所以一条指令发送完成的时间应该是在100us数量级上

上层代码调节时需要注意系统带宽！！！
上层代码调节时需要注意系统带宽！！！
上层代码调节时需要注意系统带宽！！！
*/

union_32 union_32f;

void Odrive_Axis0_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t mbox;         //发送使用到的can邮箱

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

    //等一个空の邮箱呢
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //发送成功了吗？失败就卡住了捏
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void Odrive_Axis1_Set_Input_Vel(CAN_HandleTypeDef *_hcan, float Input_Vel,float Torque)
{
    static CAN_TxHeaderTypeDef TxHeader;
    static uint8_t TxData[8];
    static uint32_t mbox;         //发送使用到的can邮箱

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

    //等一个空の邮箱呢
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //发送成功了吗？失败就卡住了捏
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}



