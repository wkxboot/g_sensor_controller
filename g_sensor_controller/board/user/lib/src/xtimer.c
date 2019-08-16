/**
******************************************************************************                                                                                                                                                       
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file       xtimer.c
*  @brief      tiny timer
*  @author     wkxboot
*  @version    v1.0.0
*  @date       2019/7/3
*  @copyright  <h4><left>&copy; copyright(c) 2019 wkxboot 1131204425@qq.com</center></h4>  
*                                                                            
*                                                                            
*****************************************************************************/
#include "cmsis_os.h"
#include "xtimer.h"
#include "debug_assert.h"


/**
* @brief 定时器初始化
* @details
* @param timer 定时器指针
* @param dir 定时器增长方向
* @param timeout 定时器超时时间
* @return 初始化结果
* @retval 0 成功
* @retval -1 失败
* @attention
* @note
*/
int xtimer_init(xtimer_t *timer,uint8_t dir,uint32_t timeout)
{
    if (timer == NULL) {
        return -1;
    }
    timer->dir = dir;
    timer->timeout = timeout;
    timer->start = osKernelSysTick();

    return 0;

}

/**
* @brief 定时器当前值
* @details
* @param timer 定时器指针
* @return 定时器当前值
* @attention 如果是向上增长，当流逝的时间大于超时值时，返回超时值；
*            如果时向下增长，当流逝的时间大于超时值时，返回0 。
* @note
*/
uint32_t xtimer_value(xtimer_t *timer)
{
    uint32_t time_elapse;

    DEBUG_ASSERT_NULL(timer);

    time_elapse = osKernelSysTick() - timer->start; 

    if (timer->dir > 0) {
        return  time_elapse >= timer->timeout ? timer->timeout : time_elapse;
    }

    return  time_elapse < timer->timeout ? timer->timeout - time_elapse : 0; 

}