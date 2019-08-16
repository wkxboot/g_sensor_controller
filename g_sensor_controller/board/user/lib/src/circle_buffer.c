/**
******************************************************************************
*  循环缓存库函数                                                          
*  Copyright (C) 2019 wkxboot 1131204425@qq.com.                             
*                                                                            
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file     circle_buffer.c                                                   
*  @brief    循环缓存库函数                                                                                                                                                                                             
*  @author   wkxboot                                                      
*  @email    1131204425@qq.com                                              
*  @version  v1.0.0                                                  
*  @date     2019/1/8                                            
*  @license  GNU General Public License (GPL)                                
*                                                                            
*                                                                            
*****************************************************************************/
#include "circle_buffer.h"

#if  CIRCLE_BUFFER_DEBUG > 0
#define  CIRCLE_BUFFER_ASSERT(x)    { if ((x) == 0) { while (1); }}
#else 
#define  CIRCLE_BUFFER_ASSERT(x)
#endif

/**
* @brief 循环缓存初始化
* @param cb 循环缓存指针
* @param buffer 数据缓存地址
* @param size 数据缓存大小
* @return 无
* @note 
*/
void circle_buffer_init(circle_buffer_t *cb,uint8_t *buffer,uint32_t size)
{  
    CIRCLE_BUFFER_ASSERT(cb);
    CIRCLE_BUFFER_ASSERT(buffer);

    cb->buffer = buffer;
    cb->read = 0;
    cb->write = 0;
    cb->read_offset = 0;
    cb->write_offset = 0;
    cb->size = size;

}

/**
* @brief 循环缓存当前数据量
* @param cb 循环缓存指针
* @return 循环缓存当前数据量
* @
* @note
*/
uint32_t circle_buffer_size(circle_buffer_t *cb)
{
    CIRCLE_BUFFER_ASSERT(cb);
    CIRCLE_BUFFER_ASSERT(cb->write >= cb->read);
    return cb->write - cb->read;
}

/**
* @brief 循环缓存冲刷
* @param cb 循环缓存指针
* @param 
* @return  冲刷的长度
* @note 
*/
uint32_t circle_buffer_flush(circle_buffer_t *cb)
{
    uint32_t size;
    
    CIRCLE_BUFFER_ASSERT(cb);

    size = cb->write - cb->read;
    cb->read = cb->write;   
    cb->read_offset = cb->read % cb->size;
    cb->write_offset = cb->write % cb->size;

    return size;
}

/**
* @brief 循环缓存是否已满
* @param cb 循环缓存指针
* @return 1：已满 0：未满
* @note
*/
uint8_t circle_buffer_is_full(circle_buffer_t *cb)
{  
    CIRCLE_BUFFER_ASSERT(cb);

    return cb->write - cb->read == cb->size ? 1 : 0;
}

/**
* @brief 循环缓存是否已空
* @param cb 循环缓存指针
* @return 1：已空 0：未空
* @
* @note
*/
uint8_t circle_buffer_is_empty(circle_buffer_t *cb)
{
    return cb->read == cb->write ? 1 : 0;
}

/**
* @brief  读取循环缓存中的数据
* @param  cb 循环缓存指针
* @param  dst 目的地址
* @param  size 期望读取的数量
* @return 实际读取的数量
* @note
*/
uint32_t circle_buffer_read(circle_buffer_t *cb,uint8_t *dst,uint32_t size)
{
    uint32_t read_size,remain_size;
    uint32_t buffer_size;
    
    CIRCLE_BUFFER_ASSERT(cb);
    CIRCLE_BUFFER_ASSERT(dst);

    /*当前缓存数量*/
    buffer_size = cb->write - cb->read;
    if (size > buffer_size) {
        size = buffer_size;
    }
    
    /*数据边界计算*/
    if (size + cb->read_offset > cb->size) {
        read_size = cb->size - cb->read_offset;
        memcpy(dst,&cb->buffer[cb->read_offset],read_size);
        remain_size = size - read_size;
        memcpy(dst + read_size,&cb->buffer[0],remain_size);
        cb->read_offset = remain_size;
    } else {
        memcpy(dst,&cb->buffer[cb->read_offset],size);
        cb->read_offset += size;
        if (cb->read_offset >= cb->size) {
            cb->read_offset = 0;
        }
    }
    cb->read += size;

    return size;
}


/**
* @brief 循环缓存写入数据
* @param cb 循环缓存指针
* @param src 数据源地址
* @param size 期望写入的数量
* @return 实际写入的数量
* @note
*/
uint32_t circle_buffer_write(circle_buffer_t *cb,const uint8_t *src,uint32_t size)
{
    uint32_t write_size,remain_size;
    uint32_t free_size;
    
    CIRCLE_BUFFER_ASSERT(cb);
    CIRCLE_BUFFER_ASSERT(src);

    /*当前可用空间*/
    free_size = cb->size - (cb->write - cb->read);
    if (size > free_size) {
        size = free_size;
    }
    /*数据边界计算*/
    if (size + cb->write_offset > cb->size) {
        write_size = cb->size - cb->write_offset;
        memcpy(&cb->buffer[cb->write_offset],src,write_size);
        remain_size = size - write_size;
        memcpy(&cb->buffer[0],src + write_size,remain_size);
        cb->write_offset = remain_size;
    } else {
        memcpy(&cb->buffer[cb->write_offset],src,size);
        cb->write_offset += size;
        if (cb->write_offset >= cb->size) {
            cb->write_offset = 0;
        }
    }
    cb->write += size;

    return size;
}