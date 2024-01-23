#ifndef __PRINTF_H__
#define __PRINTF_H__
#include "usart.h"

void usart_printf(const char *format, ...);
void usart_printf1(const char *format, ...);

#endif /* __PRINTF_H__ */