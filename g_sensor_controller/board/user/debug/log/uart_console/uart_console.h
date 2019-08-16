#ifndef  __UART_CONSOLE_H__
#define  __UART_CONSOLE_H__

#ifdef   __cplusplus
#define  UART_CONSOLE_BEGIN        extern "C" {
#define  UART_CONSOLE_END          }
#else
#define  UART_CONSOLE_BEGIN      
#define  UART_CONSOLE_END        
#endif

#include "stdint.h"

#define  UART_CONSOLE_RX_BUFFER_SIZE              32
#define  UART_CONSOLE_TX_BUFFER_SIZE              4096


#define  UART_CONSOLE_PORT                        2
#define  UART_CONSOLE_BAUD_RATES                  115200
#define  UART_CONSOLE_DATA_BITS                   8
#define  UART_CONSOLE_STOP_BITS                   1

/**
* @brief 串口uart初始化
* @param 无
* @return =0 成功
* @return <0 失败
* @note
*/
void uart_console_init(void);

/**
* @brief 串口uart读取数据
* @param dst 读取的数据存储的目的地
* @param size 期望读取的数量
* @return  实际读取的数量
* @note
*/

uint32_t uart_console_read(char *dst,uint32_t size);

/**
* @brief 串口uart写入数据
* @param src 写入的数据的源地址
* @param size 期望写入的数量
* @return  实际写入的数量
* @note
*/

uint32_t uart_console_write(char *src,uint32_t size);

/**
* @brief log 串口中断
* @param 无
* @param 无
* @return 无
* @note
*/
void uart_console_isr();

UART_CONSOLE_END



#endif

