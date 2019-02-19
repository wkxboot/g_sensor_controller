/*****************************************************************************
*  串口库函数                                                          
*  Copyright (C) 2019 wkxboot 1131204425@qq.com.                             
*                                                                            
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file     serial.c                                                   
*  @brief    串口库函数                                                                                                                                                                                             
*  @author   wkxboot                                                      
*  @email    1131204425@qq.com                                              
*  @version  v1.0.2                                                  
*  @date     2019/1/8                                            
*  @license  GNU General Public License (GPL)                                
*                                                                            
*                                                                            
*****************************************************************************/
#include "cmsis_os.h"
#include "utils.h"
#include "serial.h"
#include "log.h"



/*
* @brief  从串口非阻塞的读取指定数量的数据
* @param handle 串口句柄
* @param dst 数据目的地址
* @param size 期望读取的数量
* @return < 0 读取错误
* @return >= 0 实际读取的数量
* @note 可重入
*/
int serial_read(int handle,char *dst,int size)
{
    int read;
    serial_t *s;
    
    s = (serial_t *)handle;
  
    if (s->init == false || size < 0){
        return -1;
    }
    read = circle_buffer_read(&s->recv,dst,size); 

    SERIAL_ENTER_CRITICAL();
    if (read > 0 && s->full == true) {
        s->full = false;
        s->driver->enable_rxne_it(s->port);  
    }
    SERIAL_EXIT_CRITICAL();

    return read;
}

/*
* @brief  从串口非阻塞的写入指定数量的数据
* @param handle 串口句柄
* @param src 数据源地址
* @param size 期望写入的数量
* @return < 0 写入错误
* @return >= 0 实际写入的数量
* @note 可重入
*/
int serial_write(int handle,const char *src,int size)
{
    int write;
    serial_t *s;
    
    log_assert(src);
    s = (serial_t *)handle; 
    if (s->init == false || size < 0){
        return -1;
    }

    write = circle_buffer_write(&s->send,src,size);
    SERIAL_ENTER_CRITICAL();
    if (write > 0 && s->empty == true){
        s->empty = false;
        s->driver->enable_txe_it(s->port);
    }
    SERIAL_EXIT_CRITICAL();

    return write;
}
/*
* @brief  串口刷新
* @param handle 串口句柄
* @return < 0 失败
* @return >=0 刷新的接收缓存数量
* @note 
*/
int serial_flush(int handle)
{
    int size;
    serial_t *s;
 
    s = (serial_t *)handle; 	

    if (s->registered == false ) {
        log_error("serial handle:%d not registered.\r\n",handle);
        return -1;
    }
    SERIAL_ENTER_CRITICAL();
    s->empty = true;
    s->full = false;
    s->driver->disable_txe_it(s->port);
    s->driver->enable_rxne_it(s->port);
    circle_buffer_flush(&s->send);
    size = circle_buffer_flush(&s->recv);
    SERIAL_EXIT_CRITICAL();

    return size;
}
/*
* @brief  打开串口
* @param handle 串口句柄
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int serial_open(int handle,uint8_t port,uint32_t baud_rates,uint8_t data_bits,uint8_t stop_bits)
{
    int rc;
    serial_t *s;
 
    s = (serial_t *)handle;	   

    if (s->registered == false){
        log_error("serial handle:%d not registered.\r\n",handle);
        return -1;
    }
 
    rc = s->driver->init(port,baud_rates,data_bits,stop_bits);
    if (rc != 0){
        return -1;
    }
    SERIAL_ENTER_CRITICAL();
    s->init = true;
    s->port = port;
    s->baud_rates = baud_rates;
    s->data_bits = data_bits;
    s->stop_bits = stop_bits;
    s->driver->enable_rxne_it(s->port);
    SERIAL_EXIT_CRITICAL();

    return 0;
}

/*
* @brief  关闭串口
* @param handle 串口句柄
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int serial_close(int handle)
{
    int rc = 0;
    serial_t *s;

    s=(serial_t *)handle;

    if (s->registered == false){
        return -1;
    }

    rc = s->driver->deinit(s->port);
    if (rc != 0){
        return -1;
    }

    SERIAL_ENTER_CRITICAL();
    s->init = false;
    s->empty = true;
    s->full = false;
    s->driver->disable_rxne_it(s->port);
    s->driver->disable_txe_it(s->port);
    SERIAL_EXIT_CRITICAL();
 
    return 0;
}

/*
* @brief  串口等待数据
* @param handle 串口句柄
* @param timeout 超时时间
* @return < 0 失败
* @return = 0 等待超时
* @return > 0 等待的数据量
* @note 
*/
int serial_select(int handle,uint32_t timeout)
{
    int size;
    utils_timer_t timer;
    serial_t *s;
 
    s=(serial_t *)handle;	

    if (s->init == false) {
        log_error("serial handle:%d not open.\r\n",handle);
        return -1;
    } 
    utils_timer_init(&timer,timeout,false);

    do {
        size = circle_buffer_used_size(&s->recv);
        if (size == 0) {
            osDelay(1);
        }
    } while (utils_timer_value(&timer) > 0 && size == 0);
        
    return size;
}


/*
* @brief  串口等待数据发送完毕
* @param handle 串口句柄
* @param timeout 超时时间
* @return < 0 失败
* @return = 0 等待发送超时
* @return > 0 实际发送的数据量
* @note 
*/
int serial_complete(int handle,uint32_t timeout)
{
    utils_timer_t timer;
    serial_t *s;

    s =(serial_t *)handle;	 

    if (s->init == false){
        log_error("serial handle:%d not open.\r\n",handle);
        return -1;
    } 
    utils_timer_init(&timer,timeout,false);

    while (utils_timer_value(&timer) > 0 && s->empty == false) {
        osDelay(1);
    }

    return circle_buffer_used_size(&s->send);
}

/*
* @brief  串口注册硬件驱动
* @param handle 串口句柄
* @param driver 硬件驱动指针
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int serial_register_hal_driver(int handle,serial_hal_driver_t *driver)
{
    serial_t *s;
 
    s=(serial_t *)handle; 

    log_assert(driver);
    log_assert(driver->init);
    log_assert(driver->deinit);
    log_assert(driver->enable_txe_it);
    log_assert(driver->disable_txe_it);
    log_assert(driver->enable_rxne_it);
    log_assert(driver->disable_rxne_it);

    s->driver = driver;
    s->registered = true;

    return 0;
}



/*
* @brief  串口中断发送routine
* @param handle 串口句柄
* @param byte_send 从发送循环缓存中取出的将要发送的一个字节
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int isr_serial_get_byte_to_send(int handle,char *byte_send)
{
    int size;
    serial_t *s;
  
    s=(serial_t *)handle;	

    if (s->init == false){
        log_error("serial handle:%d not open.\r\n",handle);
        return -1;
    } 
    SERIAL_ENTER_CRITICAL();
    size = circle_buffer_read(&s->send,byte_send,1);
    /*发送缓存中已经没有待发送的数据，关闭发送中断*/
    if (size == 0) {
        s->empty = true; 
        s->driver->disable_txe_it(s->port);
    }
    SERIAL_EXIT_CRITICAL();

    return size;
}
/*
* @brief  串口中断接收routine
* @param handle 串口句柄
* @param byte_send 把从串口读取的一个字节放入接收循环缓存
* @return = 0 失败
* @return = 0 成功
* @note 
*/
int isr_serial_put_byte_from_recv(int handle,char recv_byte)
{
 
    int size;
    serial_t *s;

    s=(serial_t *)handle;	
 
    if (s->init == false){
        log_error("serial handle:%d not open.\r\n",handle);
        return -1;
    } 
    SERIAL_ENTER_CRITICAL();
    size = circle_buffer_write(&s->recv,&recv_byte,1);
    /*接收缓存中已经没有空间，关闭接收中断*/
    if (size == 0) {
        s->full = true;
        s->driver->disable_rxne_it(s->port);
    }
    SERIAL_EXIT_CRITICAL();

    return size;
}
/*
* @brief  串口创建
* @param handle 串口句柄
* @param rx_size 接收循环缓存容量
* @param tx_size 发送循环缓存容量
* @return = 0 成功
* @return < 0 失败
* @note     rx_size和tx_size必须是2的x次方
*/
int serial_create(int *handle,uint32_t rx_size,uint32_t tx_size)
{ 
    char *prx_buffer = NULL,*ptx_buffer = NULL;
    serial_t *s = NULL;
 
    log_assert(handle);

    log_assert(IS_POWER_OF_TWO(rx_size));
    log_assert(IS_POWER_OF_TWO(tx_size));

    prx_buffer = SERIAL_MALLOC(rx_size);
    ptx_buffer = SERIAL_MALLOC(tx_size);
    s = SERIAL_MALLOC(sizeof(serial_t));

    if (s == NULL || prx_buffer == NULL || ptx_buffer == NULL) {
        goto err_exit;
    }

    s->recv.buffer = prx_buffer;
    s->send.buffer = ptx_buffer;
 
    s->recv.size = rx_size;
    s->recv.mask = rx_size - 1;
    s->send.size = tx_size;
    s->send.mask = tx_size - 1;
    s->recv.read = 0;
    s->recv.write = 0;
 
    s->send.read = 0;
    s->send.write = 0;

    s->driver = NULL;
    s->registered = false;
    s->handle = (int)s;
    *handle = s->handle;

    return 0;

err_exit:
    if (prx_buffer) {
        SERIAL_FREE(prx_buffer);
    }
 
    if (ptx_buffer) {
        SERIAL_FREE(ptx_buffer);
    }
 
    if (s) {
        SERIAL_FREE(s);
    }

    return -1;		
}

/*
* @brief  串口销毁
* @param handle 串口句柄
* @return = 0 成功
* @note 
*/
int serial_destroy(int handle)
{
    serial_t *s;
 
    s = ( serial_t *)handle;

    if (s->handle != handle){
        log_error("serial handle:%d invalid.\r\n",handle);
        return -1;
    } 
    SERIAL_FREE(s->recv.buffer);
    SERIAL_FREE(s->send.buffer);
    SERIAL_FREE(s);

    return 0;
}


