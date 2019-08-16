#ifndef  __CIRCLE_BUFFER_H__
#define  __CIRCLE_BUFFER_H__
#include "stdint.h"
#include "stddef.h"
#include "string.h"

#ifdef   __cplusplus
#define  CIRCLE_BUFFER_BEGIN        extern "C" {
#define  CIRCLE_BUFFER_END          }
#else
#define  CIRCLE_BUFFER_BEGIN      
#define  CIRCLE_BUFFER_END        
#endif


CIRCLE_BUFFER_BEGIN 


#define  CIRCLE_BUFFER_DEBUG    1


typedef struct
{
    uint8_t *buffer;
    uint32_t read;
    uint32_t write;
    uint32_t read_offset;
    uint32_t write_offset;
    uint32_t size;
}circle_buffer_t;


/**
* @brief 循环缓存初始化
* @param cb 循环缓存指针
* @param buffer 数据缓存地址
* @param size 数据缓存大小
* @return 无
* @note 
*/
void circle_buffer_init(circle_buffer_t *cb,uint8_t *buffer,uint32_t size);

/**
* @brief 循环缓存当前数据量
* @param cb 循环缓存指针
* @return 循环缓存当前数据量
* @
* @note
*/
uint32_t circle_buffer_size(circle_buffer_t *cb);

/**
* @brief 循环缓存刷新
* @param cb 循环缓存指针
* @param 
* @return  刷新的长度
* @note 
*/
uint32_t circle_buffer_flush(circle_buffer_t *cb);

/**
* @brief 循环缓存是否已满
* @param cb 循环缓存指针
* @return true 已满
* @return false 未满
* @note
*/
uint8_t circle_buffer_is_full(circle_buffer_t *cb);

/**
* @brief 循环缓存是否已空
* @param cb 循环缓存指针
* @return true 已空
* @return false 未空
* @note
*/
uint8_t circle_buffer_is_empty(circle_buffer_t *cb);

/*
* @brief  读取循环缓存中的数据
* @param  cb   循环缓存指针
* @param  dst  目的地址
* @param  size 期望读取的数量
* @return 实际读取的数量
* @note
*/
uint32_t circle_buffer_read(circle_buffer_t *cb,uint8_t *dst,uint32_t size);

/**
* @brief 循环缓存写入数据
* @param cb 循环缓存指针
* @param src 数据源地址
* @param size 期望写入的数量
* @return 实际写入的数量
* @note
*/
uint32_t circle_buffer_write(circle_buffer_t *cb,const uint8_t *src,uint32_t size);

CIRCLE_BUFFER_END


#endif


