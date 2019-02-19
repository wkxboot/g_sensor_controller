#ifndef  __LOG_UART_H__
#define  __LOG_UART_H__

#ifdef  __cplusplus
    extern "C" {
#endif
#include "stdint.h"

#define  ST_SERIAL_UART                       1
#define  NXP_SERIAL_UART                      2
#define  LOG_SERIAL_UART                      ST_SERIAL_UART


#define  LOG_UART_RX_BUFFER_SIZE              32
#define  LOG_UART_TX_BUFFER_SIZE              1024


#define  LOG_UART_PORT                        1
#define  LOG_UART_BAUD_RATES                  115200
#define  LOG_UART_DATA_BITS                   8
#define  LOG_UART_STOP_BITS                   1

extern int log_serial_uart_handle;

#if (LOG_SERIAL_UART == ST_SERIAL_UART)
#include "st_serial_uart_hal_driver.h"
#elif 
#elif (LOG_SERIAL_UART == NXP_SERIAL_UART)
#include "nxp_serial_uart_hal_driver.h"
#else
#error "LOG_SERIAL_UART must be ST_SERIAL_UART or NXP_SERIAL_UART."
#endif


/*
* @brief 串口uart初始化
* @param 无
* @return =0 成功
* @return <0 失败
* @note
*/

int log_serial_uart_init(void);

/*
* @brief 串口uart读取数据
* @param dst 读取的数据存储的目的地
* @param size 期望读取的数量
* @return  实际读取的数量
* @note
*/

int log_serial_uart_read(char *dst,uint32_t size);

/*
* @brief 串口uart写入数据
* @param src 写入的数据的源地址
* @param size 期望写入的数量
* @return  实际写入的数量
* @note
*/

int log_serial_uart_write(char *src,uint32_t size);


#ifdef  __cplusplus
    }
#endif



#endif

