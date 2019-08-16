/**
****************************************************************************************                                                                                                                                                       
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file       xuart.c
*  @brief      uart库
*  @author     wkxboot
*  @version    v1.0.0
*  @date       2019/7/3
*  @copyright  <h4><left>&copy; copyright(c) 2019 wkxboot 1131204425@qq.com</center></h4>  
*                                                                            
*                                                                            
****************************************************************************************/
#include "xuart.h"
#include "debug_assert.h"


 /** 内部维护的实体*/
typedef struct
{
    uint8_t is_driver_register;/**< 驱动是否注册*/
    xuart_hal_driver_t *driver;/**< 驱动指针*/
}xuart_instance_t;

 /** 内部维护的实体*/
static xuart_instance_t xuart;


/**
* @brief  从串口非阻塞的读取数据
* @param handle 串口句柄
* @param dst 数据保存目的地址
* @param size 期望读取的数量
* @return 读取的数量
* @note 可重入
*/
uint32_t xuart_read(xuart_handle_t *handle,uint8_t *dst,uint32_t size)
{
    uint32_t read = 0;

    DEBUG_ASSERT_FALSE(xuart.is_driver_register);
    DEBUG_ASSERT_NULL(handle);
    DEBUG_ASSERT_NULL(dst);

    XUART_ENTER_CRITICAL();
    read = circle_buffer_read(&handle->recv,dst,size);

    if (read > 0 && handle->is_rxne_int_enable == 0) {
        handle->is_rxne_int_enable = 1;
        xuart.driver->enable_rxne_it(handle->setting.port);  
    }
    XUART_EXIT_CRITICAL();

    return read;
}

/**
* @brief  从串口非阻塞的写入指定数量的数据
* @param handle 串口句柄
* @param src 数据源地址
* @param size 期望写入的数量
* @return 实际写入的数量
* @note 可重入
*/
uint32_t xuart_write(xuart_handle_t *handle,const uint8_t *src,uint32_t size)
{
    uint32_t write = 0;

    DEBUG_ASSERT_FALSE(xuart.is_driver_register);
    DEBUG_ASSERT_NULL(handle);
    DEBUG_ASSERT_NULL(src);

    XUART_ENTER_CRITICAL();
    write = circle_buffer_write(&handle->send,src,size);
    if (write > 0 && handle->is_txe_int_enable == 0){
        handle->is_txe_int_enable = 1;
        xuart.driver->enable_txe_it(handle->setting.port);
    }
    XUART_EXIT_CRITICAL();

    return write;
}


/**
* @brief 串口缓存清除
* @param handle 串口句柄
* @return 无
* @note 
*/
void xuart_clear(xuart_handle_t *handle)
{

    DEBUG_ASSERT_FALSE(xuart.is_driver_register);
    DEBUG_ASSERT_NULL(handle);
    DEBUG_ASSERT_NULL(handle->is_port_open);
    
    XUART_ENTER_CRITICAL();
    handle->is_txe_int_enable = 0;
    handle->is_rxne_int_enable = 1;
    xuart.driver->disable_txe_it(handle->setting.port);
    xuart.driver->enable_rxne_it(handle->setting.port);
    circle_buffer_flush(&handle->recv);
    circle_buffer_flush(&handle->send);
    XUART_EXIT_CRITICAL();
}


/**
* @brief 打开串口
* @param handle 创建输出的串口句柄指针
* @param port 端口号
* @param baudrate 波特率
* @param data_bits 数据位
* @param stop_bit 数据
* @param handle 创建输出的串口句柄指针
* @return -1：失败 0：成功
* @note 
*/
int xuart_open(xuart_handle_t *handle,uint8_t port,uint32_t baudrate,uint8_t data_bit,uint8_t stop_bit,
                  uint8_t *rx_buffer,uint32_t rx_size,uint8_t *tx_buffer,uint32_t tx_size)
{
    int rc;

    DEBUG_ASSERT_FALSE(xuart.is_driver_register);
    DEBUG_ASSERT_NULL(handle);

    XUART_ENTER_CRITICAL();
    rc = xuart.driver->init(port,baudrate,data_bit,stop_bit);
    if (rc != 0){
        return -1;
    }

    handle->setting.port = port;
    handle->setting.baudrate = baudrate;
    handle->setting.data_bit = data_bit;
    handle->setting.stop_bit = stop_bit;
    xuart.driver->enable_rxne_it(handle->setting.port);
    xuart.driver->disable_txe_it(handle->setting.port);
    handle->is_rxne_int_enable = 1;
    handle->is_txe_int_enable = 0;
    circle_buffer_init(&handle->recv,rx_buffer,rx_size);
    circle_buffer_init(&handle->send,tx_buffer,tx_size);
    handle->is_port_open = 1;
    XUART_EXIT_CRITICAL();

    return 0;
}

/**
* @brief  关闭串口
* @param handle 串口句柄
* @return -1： 失败 0：失败
* @note 
*/
int xuart_close(xuart_handle_t *handle)
{
    int rc = 0;
    
    DEBUG_ASSERT_FALSE(xuart.is_driver_register);
    DEBUG_ASSERT_NULL(handle);

    if (handle->is_port_open == 0) {
        return 0;
    }
    
    XUART_ENTER_CRITICAL();
    rc = xuart.driver->deinit(handle->setting.port);
    if (rc != 0){
        return -1;
    }
 
    xuart.driver->disable_rxne_it(handle->setting.port);
    xuart.driver->disable_txe_it(handle->setting.port);
    handle->is_rxne_int_enable = 0;
    handle->is_txe_int_enable = 0;
    handle->is_port_open = 0;
    circle_buffer_flush(&handle->recv);
    circle_buffer_flush(&handle->send); 

    XUART_EXIT_CRITICAL();
 
    return 0;
}

/**
* @brief 串口注册驱动
* @param driver 串口硬件驱动
* @return 初始化是否成功
* @retval 0 成功
* @retval -1 失败
* @note
*/
int xuart_register_hal_driver(xuart_hal_driver_t *driver)
{ 
    if (driver == NULL || driver->init == NULL || driver->deinit == NULL  || \
        driver->enable_txe_it == NULL  || driver->disable_txe_it == NULL  || \
        driver->enable_rxne_it == NULL || driver->disable_rxne_it == NULL) {
        return -1;
    }
    xuart.driver = driver;
    xuart.is_driver_register = 1;
    return 0;
}

/**
* @brief 串口中断接收一个字节
* @param handle 串口句柄
* @param recv 接收到的字节
* @return none
* @note
*/
uint32_t xuart_isr_put_char(xuart_handle_t *handle,uint8_t recv)
{
    uint32_t write;

    DEBUG_ASSERT_FALSE(xuart.is_driver_register);
    DEBUG_ASSERT_NULL(handle);
    DEBUG_ASSERT_NULL(handle->is_port_open);
    
    write = circle_buffer_write(&handle->recv,&recv,1);
    if (write == 0 ) {
        /*禁止接收中断*/
        handle->is_rxne_int_enable = 0;
        xuart.driver->disable_rxne_it(handle->setting.port);
    }  
    return write;
}

/**
* @brief 串口中断发送一个字节
* @param handle 串口句柄
* @param recv 需要发送的字节
* @return none
* @note
*/
uint32_t xuart_isr_get_char(xuart_handle_t *handle,uint8_t *send)
{
    uint32_t read;

    DEBUG_ASSERT_FALSE(xuart.is_driver_register);
    DEBUG_ASSERT_NULL(handle);
    DEBUG_ASSERT_NULL(handle->is_port_open);

    read = circle_buffer_read(&handle->send,send,1);
    if (read == 0 ) {
        /*禁止发送中断*/
        handle->is_txe_int_enable = 0;
        xuart.driver->disable_txe_it(handle->setting.port);
    }
    return read;
}


#if XUART_IN_FREERTOS  > 0
#include "xtimer.h"
/**
* @brief 串口等待数据
* @param handle 串口句柄
* @param timeout 超时时间
* @return 接收缓存中的数据量
* @note 
*/
uint32_t xuart_select(xuart_handle_t *handle,uint32_t timeout)
{
    xtimer_t timer;

    DEBUG_ASSERT_FALSE(xuart.is_driver_register);
    DEBUG_ASSERT_NULL(handle);
    DEBUG_ASSERT_NULL(handle->is_port_open);

    xtimer_init(&timer,0,timeout);

    while (xtimer_value(&timer) > 0 && circle_buffer_size(&handle->recv) == 0) {
        osDelay(1);
    }
     
    return circle_buffer_size(&handle->recv);
}


/**
* @brief  串口等待数据发送完毕
* @param handle 串口句柄
* @param timeout 超时时间
* @return 发送缓存中的数据量
* @note 
*/
uint32_t xuart_complete(xuart_handle_t *handle,uint32_t timeout)
{
    xtimer_t timer;

    DEBUG_ASSERT_FALSE(xuart.is_driver_register);
    DEBUG_ASSERT_NULL(handle);
    DEBUG_ASSERT_NULL(handle->is_port_open);

    xtimer_init(&timer,0,timeout);
    while (xtimer_value(&timer) > 0 && circle_buffer_size(&handle->send) > 0) {
        osDelay(1);
    }

    return circle_buffer_size(&handle->send);
}

#endif

