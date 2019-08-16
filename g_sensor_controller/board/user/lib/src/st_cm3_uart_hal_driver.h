#ifndef  __ST_UART_HAL_DRIVER_H__
#define  __ST_UART_HAL_DRIVER_H__

#include "stdint.h"
#include "usart.h"
#include "xuart.h"

#ifdef  __cplusplus
    extern "C" {
#endif

extern xuart_hal_driver_t xuart_hal_driver;
/*
* @brief 串口初始化驱动
* @param port uart端口
* @param bauds 波特率
* @param data_bits 数据宽度
* @param stop_bit 停止位
* @return 
* @note
*/
int st_uart_hal_init(uint8_t port,uint32_t bauds,uint8_t data_bit,uint8_t stop_bit);

/*
* @brief 串口去初始化驱动
* @param port uart端口
* @return = 0 成功
* @return < 0 失败
* @note
*/
int st_uart_hal_deinit(uint8_t port);

/*
* @brief 串口发送为空中断使能驱动
* @param port uart端口
* @return 无
* @note
*/
void st_uart_hal_enable_txe_it(uint8_t port);

/*
* @brief 串口发送为空中断禁止驱动
* @param port uart端口
* @return 无
* @note
*/
void st_uart_hal_disable_txe_it(uint8_t port);

/*
* @brief 串口接收不为空中断使能驱动
* @param port uart端口
* @return 无
* @note
*/  
void st_uart_hal_enable_rxne_it(uint8_t port);

/*
* @brief 串口接收不为空中断禁止驱动
* @param port uart端口
* @return 无
* @note
*/
void st_uart_hal_disable_rxne_it(uint8_t port);

/*
* @brief 串口中断routine驱动
* @param port uart端口
* @return 无
* @note
*/
void st_uart_hal_isr(xuart_handle_t *handle);





#ifdef  __cplusplus
     } 
#endif

#endif