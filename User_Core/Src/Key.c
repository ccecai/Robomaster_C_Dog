//
// Created by 1 on 2023-11-04.
//
#include "Key.h"

void Key_Scan(void)
{
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_SET)
    {
        osDelay(20);
        while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_SET);
        Key_Task();
        osDelay(20);
    }
}

void Key_Task(void)
{

}