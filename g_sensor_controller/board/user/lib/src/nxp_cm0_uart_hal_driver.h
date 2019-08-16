#ifndef  __NXP_CM0_UART_HAL_DRIVER_H__
#define  __NXP_CM0_UART_HAL_DRIVER_H__
 
#ifdef  __cplusplus
#define  NXP_CM0_UART_HAL_DRIVER_BEGIN extern "C" {
#define  NXP_CM0_UART_HAL_DRIVER_END   }
#else
#define  NXP_CM0_UART_HAL_DRIVER_BEGIN
#define  NXP_CM0_UART_HAL_DRIVER_END
#endif
 
NXP_CM0_UART_HAL_DRIVER_BEGIN

#include "stdint.h"
#include "xuart.h"

extern xuart_hal_driver_t xuart_hal_driver;

/**
* @brief 串口中断routine驱动
* @param handle uart的serial句柄
* @return 无
* @note
*/
void nxp_uart_hal_isr(xuart_handle_t * handle);




NXP_CM0_UART_HAL_DRIVER_END
#endif



