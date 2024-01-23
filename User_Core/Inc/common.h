//
// Created by hyz on 2023/11/24.
//

#ifndef ESP8266_DEMO_COMMON_H
#define ESP8266_DEMO_COMMON_H
#include "stm32f4xx_hal.h"

/********************************** 函数声明 ***************************************/
void                     USART_printf                       (  UART_HandleTypeDef * USARTx, char * Data, ... );
#endif //ESP8266_DEMO_COMMON_H
