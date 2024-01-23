//
// Created by hyz on 2023/11/24.
//
#include "common.h"
#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include "retarget.h"
static char *                 itoa                                ( int value, char * string, int radix );



/*
 * 函数名：USARTx_printf
 * 描述  ：格式化输出，类似于C库中的printf，但这里没有用到C库
 * 输入  ：-USARTx 串口通道，这里用的是串口3，即USART3
 *		     -Data   要发送到串口的内容的指针
 *			   -...    其他参数
 * 输出  ：无
 * 返回  ：无
 * 调用  ：外部调用
 *         典型应用USART3_printf( USART3, "\r\n this is a demo \r\n" );
 *            		 USART3_printf( USART3, "\r\n %d \r\n", i );
 *            		 USART3_printf( USART3, "\r\n %s \r\n", j );
 */
void USART_printf ( UART_HandleTypeDef * USARTx, char * Data, ... )
{
    const char *s;
    int d;
    char buf[16];


    va_list ap;
    va_start(ap, Data);

    while ( * Data != 0 )     // 判断是否到达字符串结束符
    {
        if ( * Data == 0x5c )  //'\'
        {
            switch ( *++Data )
            {
                case 'r':							          //回车符
                    HAL_UART_Transmit(USARTx, (uint8_t *)0x0d,1,10);
                    Data ++;
                    break;

                case 'n':							          //换行符
                    HAL_UART_Transmit(USARTx, (uint8_t *)0x0a,1,10);
                    Data ++;
                    break;

                default:
                    Data ++;
                    break;
            }
        }

        else if ( * Data == '%')
        {									  //
            switch ( *++Data )
            {
                case 's':										  //字符串
                    s = va_arg(ap, const char *);

                    for ( ; *s; s++)
                    {
                        // 这有可能有问题
                        HAL_UART_Transmit(USARTx,(uint8_t *)s, 1,20);
                        while( __HAL_UART_GET_FLAG(USARTx, UART_FLAG_TXE) == RESET );
                    }

                    Data++;

                    break;

                case 'd':
                    //十进制
                    d = va_arg(ap, int);

                    itoa(d, buf, 10);

                    for (s = buf; *s; s++)
                    {
                        HAL_UART_Transmit(USARTx,(uint8_t *)s, 1,50);
//                        USART_SendData(USARTx,*s);
                        while( __HAL_UART_GET_FLAG(USARTx, UART_FLAG_TXE) == RESET );
                    }

                    Data++;

                    break;

                default:
                    Data++;

                    break;

            }
        }

        else HAL_UART_Transmit(USARTx, (uint8_t *)Data++, 1,50);

        while ( __HAL_UART_GET_FLAG ( USARTx, UART_FLAG_TXE ) == RESET );

    }
}


/*
 * 函数名：itoa
 * 描述  ：将整形数据转换成字符串
 * 输入  ：-radix =10 表示10进制，其他结果为0
 *         -value 要转换的整形数
 *         -buf 转换后的字符串
 *         -radix = 10
 * 输出  ：无
 * 返回  ：无
 * 调用  ：被USARTx_printf()调用
 */
static char * itoa( int value, char *string, int radix )
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;

    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */



