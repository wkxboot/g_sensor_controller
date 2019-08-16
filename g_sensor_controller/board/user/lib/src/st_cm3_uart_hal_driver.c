/*****************************************************************************
*  st serial uart hal driver                                                          
*  Copyright (C) 2019 wkxboot 1131204425@qq.com.                             
*                                                                            
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file     st_serial_uart_hal_driver.c                                                   
*  @brief    st serial uart hal driver                                                                                                                                                                                             
*  @author   wkxboot                                                      
*  @email    1131204425@qq.com                                              
*  @version  v1.0.0                                                  
*  @date     2019/1/10                                            
*  @license  GNU General Public License (GPL)                                
*                                                                            
*                                                                            
*****************************************************************************/
#include "st_cm3_uart_hal_driver.h"


extern UART_HandleTypeDef huart1,huart2,huart3;

/*st serial uart驱动结构体*/
xuart_hal_driver_t xuart_hal_driver = {
.init = st_uart_hal_init,
.deinit = st_uart_hal_deinit,
.enable_txe_it = st_uart_hal_enable_txe_it,
.disable_txe_it = st_uart_hal_disable_txe_it,
.enable_rxne_it = st_uart_hal_enable_rxne_it,
.disable_rxne_it = st_uart_hal_disable_rxne_it
};


/* *****************************************************************************
*
*        st cm3 uart hal driver在IAR freertos下的移植
*
********************************************************************************/

/*
* @brief 根据uart端口查找uart句柄
* @param port uart端口
* @return uart句柄
* @note
*/
static UART_HandleTypeDef *st_uart_hal_get_handle_by_port(uint8_t port)
{
    UART_HandleTypeDef *st_uart_handle;
    if (port == 1) {
        st_uart_handle = &huart1;
        st_uart_handle->Instance = USART1;
    } else if (port == 2) {
        st_uart_handle = &huart2;
        st_uart_handle->Instance = USART2;
    }else if (port == 3) {
        st_uart_handle = &huart3;
        st_uart_handle->Instance = USART3;
    }else {
        st_uart_handle = &huart1;
        st_uart_handle->Instance = USART1;
    }
    return st_uart_handle;
}


/**
* @brief 串口初始化驱动
* @param port 端口
* @param baudrate 波特率
* @param data_bit 数据宽度
* @param stop_bit 停止位
* @return 
* @note
*/
static int st_uart_hal_init(uint8_t port,uint32_t baudrate,uint8_t data_bit,uint8_t stop_bit)
{
    UART_HandleTypeDef *st_uart_handle;

    st_uart_handle = st_uart_hal_get_handle_by_port(port);

    st_uart_handle->Init.BaudRate = baudrate;
    if (data_bit == 8) {
        st_uart_handle->Init.WordLength = UART_WORDLENGTH_8B;
    }else{
        st_uart_handle->Init.WordLength = UART_WORDLENGTH_9B;
    }
    if (stop_bit == 1) {
        st_uart_handle->Init.StopBits = UART_STOPBITS_1;
    }else {
        st_uart_handle->Init.StopBits = UART_STOPBITS_2; 
    }

    st_uart_handle->Init.Parity = UART_PARITY_NONE;
    st_uart_handle->Init.Mode = UART_MODE_TX_RX;
    st_uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    st_uart_handle->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(st_uart_handle) != HAL_OK) {
        return -1;
    }

    return 0;
}


/**
* @brief 串口去初始化驱动
* @param port uart端口
* @return = 0 成功
* @return < 0 失败
* @note
*/
static int st_uart_hal_deinit(uint8_t port)
{
    return 0;
}


/**
* @brief 串口发送为空中断使能驱动
* @param port uart端口
* @return 无
* @note
*/
static void st_uart_hal_enable_txe_it(uint8_t port)
{
    UART_HandleTypeDef *st_uart_handle;

    st_uart_handle = st_uart_hal_get_handle_by_port(port);
    /*使能发送中断*/
    __HAL_UART_ENABLE_IT(st_uart_handle,/*UART_IT_TXE*/UART_IT_TC);   
}


/**
* @brief 串口发送为空中断禁止驱动
* @param port uart端口
* @return 无
* @note
*/
static void st_uart_hal_disable_txe_it(uint8_t port)
{
    UART_HandleTypeDef *st_uart_handle;

    st_uart_handle = st_uart_hal_get_handle_by_port(port);
    /*禁止发送中断*/
    __HAL_UART_DISABLE_IT(st_uart_handle, /*UART_IT_TXE*/UART_IT_TC);   
}


/**
* @brief 串口接收不为空中断使能驱动
* @param port uart端口
* @return 无
* @note
*/  
static void st_uart_hal_enable_rxne_it(uint8_t port)
{
    UART_HandleTypeDef *st_uart_handle;

    st_uart_handle = st_uart_hal_get_handle_by_port(port);
    /*使能接收中断*/
    __HAL_UART_ENABLE_IT(st_uart_handle,UART_IT_RXNE);  
}


/**
* @brief 串口接收不为空中断禁止驱动
* @param port uart端口
* @return 无
* @note
*/
static void st_uart_hal_disable_rxne_it(uint8_t port)
{
    UART_HandleTypeDef *st_uart_handle;

    st_uart_handle = st_uart_hal_get_handle_by_port(port);
    /*禁止接收中断*/
    __HAL_UART_DISABLE_IT(st_uart_handle,UART_IT_RXNE); 
}


/**
* @brief 串口中断routine驱动
* @param port uart端口
* @return 无
* @note
*/
void st_uart_hal_isr(xuart_handle_t *handle)
{
    uint32_t rc;
    uint8_t recv_byte,send_byte;
    UART_HandleTypeDef *st_uart_handle;

    st_uart_handle = st_uart_hal_get_handle_by_port(handle->setting.port);

    uint32_t tmp_flag = 0, tmp_it_source = 0; 
  
    tmp_flag = __HAL_UART_GET_FLAG(st_uart_handle, UART_FLAG_RXNE);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(st_uart_handle, UART_IT_RXNE);
  
    /*接收中断*/
    if((tmp_flag != RESET) && (tmp_it_source != RESET)) { 
        recv_byte = (char)(st_uart_handle->Instance->DR & (uint8_t)0x00FF);
        xuart_isr_put_char(handle,recv_byte);
    }

    tmp_flag = __HAL_UART_GET_FLAG(st_uart_handle, /*UART_FLAG_TXE*/UART_FLAG_TC);
    tmp_it_source = __HAL_UART_GET_IT_SOURCE(st_uart_handle, /*UART_IT_TXE*/UART_IT_TC);
  
    /*发送中断*/
    if ((tmp_flag != RESET) && (tmp_it_source != RESET)) {
        rc = xuart_isr_get_char(handle,&send_byte);
        if (rc) {
            st_uart_handle->Instance->DR = send_byte;
        }
    }  
}