//
// Created by 1 on 2023-11-04.
//
#include "LED.h"

int count;

void LED_Flash(void)
{
    switch (count) {
        case 0:
            HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET);
            break;
        default:
            break;

    }
    if(count++ == 2)
        count = 0;

}