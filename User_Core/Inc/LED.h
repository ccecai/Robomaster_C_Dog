//
// Created by 1 on 2023-11-04.
//

#ifndef ROBOMASTER_A_LED_H
#define ROBOMASTER_A_LED_H

#include "gpio.h"

#define BUZZER_ON __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,500)
#define BUZZER_OFF __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0)

void LED_Flash(void);

#endif //ROBOMASTER_A_LED_H
