#ifndef  __NXP_SERIAL_UART_HAL_DRIVER_H__
#define  __NXP_SERIAL_UART_HAL_DRIVER_H__

#ifdef  __cplusplus
    extern "C" {
#endif

#include "stdint.h"
#include "serial.h"

extern serial_hal_driver_t nxp_serial_uart_hal_driver;
/*
* @brief 串口初始化驱动
* @param port uart端口号
* @param bauds 波特率
* @param data_bits 数据宽度
* @param stop_bit 停止位
* @return 
* @note
*/
int nxp_serial_uart_hal_init(uint8_t port,uint32_t baud_rates,uint8_t data_bits,uint8_t stop_bits);

/*
* @brief 串口去初始化驱动
* @param port uart端口号
* @return = 0 成功
* @return < 0 失败
* @note
*/
int nxp_serial_uart_hal_deinit(uint8_t port);


/*
* @brief 串口发送为空中断使能驱动
* @param port uart端口号
* @return 无
* @note
*/ 
void nxp_serial_uart_hal_enable_txe_it(uint8_t port);

/*
* @brief 串口发送为空中断禁止驱动
* @param port uart端口号
* @return 无
* @note
*/
void nxp_serial_uart_hal_disable_txe_it(uint8_t port);

/*
* @brief 串口接收不为空中断使能驱动
* @param port uart端口号
* @return 无
* @note
*/ 
void nxp_serial_uart_hal_enable_rxne_it(uint8_t port);

/*
* @brief 串口接收不为空中断禁止驱动
* @param port uart端口号
* @return 无
* @note
*/
void nxp_serial_uart_hal_disable_rxne_it(uint8_t port);

/*
* @brief 串口中断routine驱动
* @param handle uart的serial句柄
* @return 无
* @note
*/
void nxp_serial_uart_hal_isr(int handle);



#ifdef  __cplusplus
     } 
#endif





#endif