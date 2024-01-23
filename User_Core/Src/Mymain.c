//
// Created by 1 on 2023-11-02.
//
#include "Mymain.h"
#include "bsp_delay.h"

void Mymain_Init(void)
{
    can1_filter_init();
    can2_filter_init();
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//蜂鸣器
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); //5V电源输出调节
    HAL_TIM_Base_Start_IT(&htim2);
    delay_init();
}