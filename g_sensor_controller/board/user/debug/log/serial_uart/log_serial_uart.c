/*****************************************************************************
*  log serial uart库                                                          
*  Copyright (C) 2019 wkxboot 1131204425@qq.com.                             
*                                                                            
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file     log_serial_uart.c                                                   
*  @brief    log serial uart库                                                                                                                                                                                             
*  @author   wkxboot                                                      
*  @email    1131204425@qq.com                                              
*  @version  v1.0.0                                                  
*  @date     2019/1/10                                            
*  @license  GNU General Public License (GPL)                                
*                                                                            
*                                                                            
*****************************************************************************/

#include "serial.h"
#include "log_serial_uart.h"

int log_serial_uart_handle;




#if (LOG_SERIAL_UART == ST_SERIAL_UART)
static serial_hal_driver_t *log_serial_uart_driver = &st_serial_uart_hal_driver;
#elif (LOG_SERIAL_UART == NXP_SERIAL_UART)
static serial_hal_driver_t *log_serial_uart_driver = &nxp_serial_uart_hal_driver;
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
int log_serial_uart_init(void)
{
    int rc;
    rc = serial_create(&log_serial_uart_handle,LOG_UART_RX_BUFFER_SIZE,LOG_UART_TX_BUFFER_SIZE);
    if (rc != 0) {
        return -1;
    }
    rc = serial_register_hal_driver(log_serial_uart_handle,log_serial_uart_driver);
    if (rc != 0) {
        return -1;
    }

    rc = serial_open(log_serial_uart_handle,LOG_UART_PORT,LOG_UART_BAUD_RATES,LOG_UART_DATA_BITS,LOG_UART_STOP_BITS);
    if (rc != 0) {
        return -1;
    }
    return 0;
}

/*
* @brief 串口uart读取数据
* @param dst 读取的数据存储的目的地
* @param size 期望读取的数量
* @return  实际读取的数量
* @note
*/

int log_serial_uart_read(char *dst,uint32_t size)
{
    return serial_read(log_serial_uart_handle,dst,size);
}

/*
* @brief 串口uart写入数据
* @param src 写入的数据的源地址
* @param size 期望写入的数量
* @return  实际写入的数量
* @note
*/

int log_serial_uart_write(char *src,uint32_t size)
{
    return serial_write(log_serial_uart_handle,src,size);
}




