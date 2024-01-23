#include "can_rec.h"
#include "can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"

moto_info_t motor_info[MOTOR_MAX_NUM];       //rm������ص���������

/**
 * @brief can1��������ʼ����ֻ���˶�ѧ�������������
 */
void can1_filter_init()
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterBank = 0;

    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure) != HAL_OK)
    {
        Error_Handler();
    }


    /* Start the CAN peripheral */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    /* Activate CAN RX notification */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }

    /* Activate CAN TX notification */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}
/**
 * @brief can2��������ʼ������ȫ������
 */
void can2_filter_init()
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterBank = 14;

    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfigStructure) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start the CAN peripheral */
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    /* Activate CAN RX notification */
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }

    /* Activate CAN TX notification */
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}
/**
  * @brief  ʹ��can�����ĸ�rm����ĵ���(����ʽ)
  * @param  id_range ����id ǰ�ĸ����Ϊ200�����ĸ�Ϊ1FF
  * @param  currentx �����е�x������ĵ���
  */
void set_current(CAN_HandleTypeDef *_hcan, int16_t id_range, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t mbox;         //����ʹ�õ���can����

    TxHeader.StdId = id_range;	         // id��ֵ
    TxHeader.IDE = 0;                     // ��׼֡
    TxHeader.RTR = 0;                     //
    TxHeader.DLC = 8;                     // 8�ֽ�����֡

    /*���ݸ�ֵ*/
    TxData[0] = (current1 >> 8) & 0xff;
    TxData[1] = (current1) & 0xff;
    TxData[2] = (current2 >> 8) & 0xff;
    TxData[3] = (current2) & 0xff;
    TxData[4] = (current3 >> 8) & 0xff;
    TxData[5] = (current3) & 0xff;
    TxData[6] = (current4 >> 8) & 0xff;
    TxData[7] = (current4) & 0xff;

    //��һ���դ�������
    while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);

    //���ͳɹ�����ʧ�ܾͿ�ס����
    if (HAL_CAN_AddTxMessage(_hcan, &TxHeader, TxData, &mbox) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ��3508�ķ������Ľ��н���
 * @param ptr Ŀ����
 * @param data 8�ֽ�can����
 */
void motor_info_record(moto_info_t *ptr, uint8_t *data)
{
    /*�״��ϵ�ʱ��ȡ���ת�ӵĳ�ʼλ��*/
    if(ptr->msg_cnt == 0)
        ptr->offset_angle = ptr->angle = ((data[0] << 8) | data[1]);    //ת��λ��
    else
    {
        /*ת����Ϣ��ȡ*/
        ptr->last_angle = ptr->angle;
        ptr->angle = ((data[0] << 8) | data[1]);    //ת��λ��
        ptr->speed = ((data[2] << 8) | data[3]);    //ת���ٶ�
        ptr->torque_current = ((data[4] << 8) | data[5]);    //ת��Ť��
        ptr->temp = data[6];   //����¶�

        /*�ж�Ȧ���Ƿ�Ӽ�*/
        if(ptr->angle - ptr->last_angle > 4095)
            ptr->total_cnt--;
        else if(ptr->angle - ptr->last_angle < -4095)
            ptr->total_cnt++;

        /*����ת�����ܽǶ�*/
        ptr->total_angle = ptr->total_cnt * 8192 + ptr->angle - ptr->offset_angle;
    }

    ptr->msg_cnt++;     //��Ϣ����+1
}

/**
  * @brief  ����fifo0�Ļص������������൱��can��rx0�ж�
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //���ջص�����
{
    HAL_StatusTypeDef HAL_RetVal;
    CAN_RxHeaderTypeDef RxHeader;
    union_64 rxdata;
    /*����ż�¼*/
    static uint8_t index;

    if(hcan == &hcan1)  // ���ϲ���ӵķ��������ݵ�
    {
        HAL_RetVal = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rxdata.data_u8);      //��CAN1�������ݣ�ͨ�������������FIFO0,����RxMessage����֡

        if(HAL_RetVal == HAL_OK)
        {
            if(RxHeader.StdId >= 0x201 && RxHeader.StdId <= 0x208)
            {
                index = RxHeader.StdId - 0x201;   //�ṹ������0-7��Ӧ���ID1-8
                motor_info_record(&motor_info[index], rxdata.data_u8);   //���
            }
            __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);   //��һ�£���Ȼ�Ϳ�ס��
        }
    }

}