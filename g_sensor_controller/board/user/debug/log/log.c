/**
******************************************************************************
*  log库函数                                                          
*  Copyright (C) 2019 wkxboot 1131204425@qq.com.                             
*                                                                            
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file     log.c                                                   
*  @brief    log库函数                                                                                                                                                                                             
*  @author   wkxboot                                                      
*  @email    1131204425@qq.com                                              
*  @version  v1.0.1                                                  
*  @date     2019/1/10                                            
*  @license  GNU General Public License (GPL)                                
*                                                                            
*                                                                            
*****************************************************************************/
#include "log.h"

#if  LOG_USE_SEGGER_RTT > 0
#include "SEGGER_RTT.h"
#endif

#if  LOG_USE_UART_CONSOLE > 0
#include "uart_console.h"
#endif


/** 日志全局输出等级*/
volatile uint8_t log_level_globle = LOG_LEVEL_GLOBLE_DEFAULT;
 
                                                                        
	
/**
* @brief 终端日志初始化
* @param 无
* @return 无
* @note
*/
void log_init(void)
{
#if LOG_USE_SEGGER_RTT > 0
    SEGGER_RTT_Init();
#endif

#if  LOG_USE_UART_CONSOLE > 0
    uart_console_init();
#endif
}

/**
* @brief 设置日志全局输出等级
* @param lelvel 日志等级
* @return = 0 成功
* @return < 0 失败
* @note level >= LOG_LEVEL_OFF && level <=LOG_LEVEL_ARRAY
*/
int log_set_level(uint8_t level)
{
    if (level > LOG_LEVEL_LOWEST) {
        return -1;
    }
 
    log_level_globle = level;
    return 0;  
}

/**
* @brief 终端读取输入
* @param dst 读取数据存储的目的地址
* @param size 期望读取的数量
* @return 实际读取的数量
* @note
*/

uint32_t log_read(char *dst,uint32_t size)
{
    uint32_t read_cnt;
#if    LOG_USE_SEGGER_RTT > 0
    read_cnt = SEGGER_RTT_Read(0,dst,size);
#elif  LOG_USE_UART_CONSOLE > 0
    read_cnt = uart_console_read(dst,size);
#endif
    return read_cnt;
}


/**
* @brief 终端日志输出
* @param level 输出等级
* @param format 格式化字符串
* @param ... 可变参数
* @return 实际写入的数量
* @note
*/

int log_printf(uint8_t level,const char *format,...)
{
    int rc = 0;
    uint32_t size;
    static char log_print_buffer[LOG_PRINTF_BUFFER_SIZE];
    va_list ap;
    
    va_start(ap,format);

    if (level <= log_level_globle ) {
        vsnprintf(log_print_buffer,LOG_PRINTF_BUFFER_SIZE,format,ap);
        size = strlen(log_print_buffer);
#if    LOG_USE_SEGGER_RTT > 0
        rc = SEGGER_RTT_Write(0,log_print_buffer,size);
#elif  LOG_USE_UART_CONSOLE > 0
        rc = uart_console_write(log_print_buffer,size);
#endif
    }
    va_end(ap);
    return rc;
}

/**
* @brief 日志时间
* @param 无
* @return 
* @note 日志时间
*/
__weak uint32_t log_time(void)
{
    return 0;
}


/**
* @brief 日志断言
* @param 无
* @return 
* @note 
*/ 
__weak void log_assert_handler(int line,char *file_name)
{
    log_error("#############系统断言错误! ##############\r\n");
    log_error("断言文件：%s.\r\n",file_name);
    log_error("断言行号：%d.\r\n",line);
    while(1);
}


