//
// Created by 1 on 2024-01-15.
//
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң����������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж���������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "RemoteControl.h"
#include "main.h"


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //�ڴ滺����1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data
//ң�������Ʊ���
RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame, but set 36 bytes
//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ң������ʼ��
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          ��ȡң��������ָ��
  * @param[in]      none
  * @retval         ң��������ָ��
  */
RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
//


void Posture_Controller(RC_ctrl_t *local_rc_ctrl)
{
    if(local_rc_ctrl->rc.s[1] == 3)
    {
        AllLegsSpeedLimit(SpeedMode_FAST);
        StandUp_Posture();
    }

    else if(local_rc_ctrl->rc.s[1] == 1)
    {
        AllLegsSpeedLimit(SpeedMode_VERYSLOW);
        LieDown_Posture();
    }

    else if(local_rc_ctrl->rc.s[1] == 2 && local_rc_ctrl->rc.ch[1] > 330)
    {
        AllLegsSpeedLimit(SpeedMode_FAST);
        Trot(Backward,1);
    }

    else if(local_rc_ctrl->rc.s[1] == 2 && local_rc_ctrl->rc.ch[1] < -330)
    {
        AllLegsSpeedLimit(SpeedMode_FAST);
        Trot(Forward,1);
    }

    else if(local_rc_ctrl->rc.s[1] == 2 && local_rc_ctrl->rc.ch[0] > 330)
    {
        AllLegsSpeedLimit(SpeedMode_FAST);
        Turn('r');
    }

    else if(local_rc_ctrl->rc.s[1] == 2 && local_rc_ctrl->rc.ch[0] < -330)
    {
        AllLegsSpeedLimit(SpeedMode_FAST);
        Turn('l');
    }

    else if(local_rc_ctrl->rc.s[1] == 2 && local_rc_ctrl->rc.ch[0] > -330 && local_rc_ctrl->rc.ch[0] < 330 && local_rc_ctrl->rc.ch[1] > -330 && local_rc_ctrl->rc.ch[1] < 330)
    {
        AllLegsSpeedLimit(SpeedMode_FAST);
        StandUp_Posture();
    }

    else if(local_rc_ctrl->rc.s[1] == 2 && local_rc_ctrl->rc.ch[3] > 330)
    {
        AllLegsSpeedLimit(SpeedMode_FAST);
        Up_and_Down((float)((local_rc_ctrl->rc.ch[3] - 330)) / 500);
    }

    else if(local_rc_ctrl->rc.s[1] == 2 && local_rc_ctrl->rc.ch[3] < -330)
    {
        AllLegsSpeedLimit(SpeedMode_FAST);
        Up_and_Down((float)((local_rc_ctrl->rc.ch[3] + 300)) / 500);
    }
//    else if(local_rc_ctrl->rc.s[1] == 2 && local_rc_ctrl.rc.s[0] == 3)
//    {
//
//    }

}