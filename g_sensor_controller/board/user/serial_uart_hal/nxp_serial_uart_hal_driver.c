/*****************************************************************************
*  nxp serial uart hal driver                                                          
*  Copyright (C) 2019 wkxboot 1131204425@qq.com.                             
*                                                                            
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file     nxp_serial_uart_hal_driver.c                                                   
*  @brief    nxp serial uart hal driver                                                                                                                                                                                             
*  @author   wkxboot                                                      
*  @email    1131204425@qq.com                                              
*  @version  v1.0.0                                                  
*  @date     2019/1/10                                            
*  @license  GNU General Public License (GPL)                                
*                                                                            
*                                                                            
*****************************************************************************/
#include "fsl_usart.h"
#include "fsl_clock.h"
#include "pin_mux.h"
#include "nxp_serial_uart_hal_driver.h"


/* *****************************************************************************
*
*        lpc54606 serial uart hal driver在IAR freertos下的移植
*
********************************************************************************/
/*nxp serial uart驱动结构体*/
serial_hal_driver_t nxp_serial_uart_hal_driver = {
.init = nxp_serial_uart_hal_init,
.deinit = nxp_serial_uart_hal_deinit,
.enable_txe_it = nxp_serial_uart_hal_enable_txe_it,
.disable_txe_it = nxp_serial_uart_hal_disable_txe_it,
.enable_rxne_it = nxp_serial_uart_hal_enable_rxne_it,
.disable_rxne_it = nxp_serial_uart_hal_disable_rxne_it
};

/*
* @brief 根据uart端口查找uart句柄
* @param port uart端口号
* @return uart句柄
* @note
*/
static USART_Type *nxp_serial_uart_search_handle_by_port(uint8_t port)
{
    USART_Type *nxp_uart_handle;
    
    if(port == 0){
        nxp_uart_handle = USART0;
    }else if(port == 1){
        nxp_uart_handle = USART1;
    }else if(port == 2){
        nxp_uart_handle = USART2;
    }else if(port == 3){
        nxp_uart_handle = USART3;
    }else if(port == 4){
        nxp_uart_handle = USART4;
    }else if(port == 5){
        nxp_uart_handle = USART5;
    }else if(port == 6){
        nxp_uart_handle = USART6;
    }else if(port == 7){
        nxp_uart_handle = USART7;
    }else if(port == 8){
        nxp_uart_handle = USART8;
    }else if(port == 9){
        nxp_uart_handle = USART9;
    }else{
        nxp_uart_handle = USART0;
    }

    return nxp_uart_handle;
}

/*
* @brief 根据uart端口查找uart中断号,时钟id和时钟源
* @param port uart端口号
* @return 0：成功 其他：失败
* @note
*/
static int nxp_serial_uart_search_irq_num_and_clk_by_port(uint8_t port,IRQn_Type *nxp_uart_irq_num,clock_name_t *clk_name,clock_attach_id_t *clk_src)
{
  
    if (port == 0){
        *nxp_uart_irq_num = FLEXCOMM0_IRQn;
        *clk_name = kCLOCK_Flexcomm0;
        *clk_src = kFRO12M_to_FLEXCOMM0;
    }else if(port == 1){
        *nxp_uart_irq_num = FLEXCOMM1_IRQn;
        *clk_name = kCLOCK_Flexcomm1;
        *clk_src = kFRO12M_to_FLEXCOMM1;
    }else if(port == 2){
        *nxp_uart_irq_num = FLEXCOMM2_IRQn;
        *clk_name = kCLOCK_Flexcomm2;
        *clk_src = kFRO12M_to_FLEXCOMM2;
    }else if(port == 3){
        *nxp_uart_irq_num = FLEXCOMM3_IRQn;
        *clk_name = kCLOCK_Flexcomm3;
        *clk_src = kFRO12M_to_FLEXCOMM3;
    }else if(port == 4){
        *nxp_uart_irq_num = FLEXCOMM4_IRQn;
        *clk_name = kCLOCK_Flexcomm4;
        *clk_src = kFRO12M_to_FLEXCOMM4;
    }else if(port == 5){
        *nxp_uart_irq_num = FLEXCOMM5_IRQn;
        *clk_name = kCLOCK_Flexcomm5;
        *clk_src = kFRO12M_to_FLEXCOMM5;;
    }else if(port == 6){
        *nxp_uart_irq_num = FLEXCOMM6_IRQn;
        *clk_name = kCLOCK_Flexcomm6;
        *clk_src = kFRO12M_to_FLEXCOMM6;
    }else if(port == 7){
        *nxp_uart_irq_num = FLEXCOMM7_IRQn;
        *clk_name = kCLOCK_Flexcomm7;
        *clk_src = kFRO12M_to_FLEXCOMM7;
    }else if(port == 8){
        *nxp_uart_irq_num = FLEXCOMM8_IRQn;
        *clk_name = kCLOCK_Flexcomm8;
        *clk_src = kFRO12M_to_FLEXCOMM8;
    }else if(port == 9){
        *nxp_uart_irq_num = FLEXCOMM9_IRQn;
        *clk_name = kCLOCK_Flexcomm9;
        *clk_src = kFRO12M_to_FLEXCOMM9;
    }else{
        *nxp_uart_irq_num = FLEXCOMM0_IRQn;
        *clk_name = kCLOCK_Flexcomm0;
        *clk_src = kFRO12M_to_FLEXCOMM0;
    }

    return 0;
}

/*
* @brief 串口初始化驱动
* @param port uart端口号
* @param bauds 波特率
* @param data_bits 数据宽度
* @param stop_bit 停止位
* @return 
* @note
*/
int nxp_serial_uart_hal_init(uint8_t port,uint32_t baud_rates,uint8_t data_bits,uint8_t stop_bits)
{
    status_t status;
    usart_config_t config;
    IRQn_Type  serial_irq_num;
    clock_name_t clk_name;
    clock_attach_id_t clk_src;
    USART_Type *nxp_uart_handle; 

    nxp_serial_uart_search_irq_num_and_clk_by_port(port,&serial_irq_num,&clk_name,&clk_src);
    nxp_uart_handle = nxp_serial_uart_search_handle_by_port(port);

    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = baud_rates;
    if(data_bits == 8){
    config.bitCountPerChar = kUSART_8BitsPerChar;
    }else{
    config.bitCountPerChar = kUSART_7BitsPerChar;
    }
    if(stop_bits ==1){
    config.stopBitCount = kUSART_OneStopBit;
    }else{
    config.stopBitCount = kUSART_TwoStopBit;
    }
    
    config.parityMode = kUSART_ParityDisabled;
   
    config.loopback = false;
    config.enableRx = true;
    config.enableTx = true;

    CLOCK_AttachClk(clk_src);
    /* Initialize the USART with configuration. */
    status=USART_Init(nxp_uart_handle, &config, CLOCK_GetFreq(clk_name));
 
  if (status != kStatus_Success){
    return -1;
  } 
  NVIC_SetPriority(serial_irq_num, 3);
  EnableIRQ(serial_irq_num);
  return 0;
}


/*
* @brief 串口去初始化驱动
* @param port uart端口号
* @return = 0 成功
* @return < 0 失败
* @note
*/
int nxp_serial_uart_hal_deinit(uint8_t port)
{
    return 0; 
}


/*
* @brief 串口发送为空中断使能驱动
* @param port uart端口号
* @return 无
* @note
*/ 
void nxp_serial_uart_hal_enable_txe_it(uint8_t port)
{
    USART_Type *nxp_uart_handle; 

    nxp_uart_handle = nxp_serial_uart_search_handle_by_port(port);
    USART_EnableInterrupts(nxp_uart_handle,kUSART_TxLevelInterruptEnable);
}
 

/*
* @brief 串口发送为空中断禁止驱动
* @param port uart端口号
* @return 无
* @note
*/
void nxp_serial_uart_hal_disable_txe_it(uint8_t port)
{
    USART_Type *nxp_uart_handle; 

    nxp_uart_handle = nxp_serial_uart_search_handle_by_port(port);
    USART_DisableInterrupts(nxp_uart_handle,kUSART_TxLevelInterruptEnable);  
}


/*
* @brief 串口接收不为空中断使能驱动
* @param port uart端口号
* @return 无
* @note
*/ 
void nxp_serial_uart_hal_enable_rxne_it(uint8_t port)
{
    USART_Type *nxp_uart_handle; 

    nxp_uart_handle = nxp_serial_uart_search_handle_by_port(port);
    USART_EnableInterrupts(nxp_uart_handle,kUSART_RxLevelInterruptEnable);
}


/*
* @brief 串口接收不为空中断禁止驱动
* @param port uart端口号
* @return 无
* @note
*/
void nxp_serial_uart_hal_disable_rxne_it(uint8_t port)
{
    USART_Type *nxp_uart_handle; 

    nxp_uart_handle = nxp_serial_uart_search_handle_by_port(port);
    USART_DisableInterrupts(nxp_uart_handle,kUSART_RxLevelInterruptEnable);
}


/*
* @brief 串口中断routine驱动
* @param handle uart的serial句柄
* @return 无
* @note
*/
void nxp_serial_uart_hal_isr(int handle)
{
    int result;
    uint32_t tmp_flag = 0, tmp_it_source = 0; 
    char  send_byte,recv_byte;
    USART_Type *nxp_uart_handle; 

    nxp_uart_handle = nxp_serial_uart_search_handle_by_port(((serial_t *)handle)->port);

    tmp_flag = USART_GetEnabledInterrupts(nxp_uart_handle);
    tmp_it_source =USART_GetStatusFlags(nxp_uart_handle);
  
    /*接收中断处理*/
    if((tmp_it_source & kUSART_RxFifoNotEmptyFlag) && (tmp_flag & kUSART_RxLevelInterruptEnable)){
        recv_byte=USART_ReadByte(nxp_uart_handle);
        isr_serial_put_byte_from_recv(handle,recv_byte);

    }
    /*发送中断处理*/
    if((tmp_it_source & kUSART_TxFifoEmptyFlag) && (tmp_flag & kUSART_TxLevelInterruptEnable)){
        result =isr_serial_get_byte_to_send(handle,&send_byte);
        if (result == 1) {
            USART_WriteByte(nxp_uart_handle, send_byte);
        }
    }
}
