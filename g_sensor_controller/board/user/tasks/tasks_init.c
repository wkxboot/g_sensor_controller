#include "cmsis_os.h"
#include "debug_task.h"
#include "watch_dog_task.h"
#include "scale_task.h"
#include "tasks_init.h"
#include "adc_task.h"
#include "lock_task.h"
#include "temperature_task.h"
#include "compressor_task.h"
#include "communication_task.h"
#include "log.h"

/*
* @brief 任务初始化
* @param 无
* @return 无
* @note
*/

void tasks_init(void)
{
    /**************************************************************************/  
    /* 任务消息队列                                                           */
    /**************************************************************************/  

    /*通信消息队列*/
    osMessageQDef(communication_task_msg_q,4,uint32_t);
    communication_task_msg_q_id = osMessageCreate(osMessageQ(communication_task_msg_q),0);
    log_assert_null_ptr(communication_task_msg_q_id);

    /*温度消息队列*/
    osMessageQDef(temperature_task_msg_q,4,uint32_t);
    temperature_task_msg_q_id = osMessageCreate(osMessageQ(temperature_task_msg_q),0);
    log_assert_null_ptr(temperature_task_msg_q_id);

    /*压缩机消息队列*/
    osMessageQDef(compressor_task_msg_q,4,uint32_t);
    compressor_task_msg_q_id = osMessageCreate(osMessageQ(compressor_task_msg_q),0);
    log_assert_null_ptr(compressor_task_msg_q_id);

    /*锁控消息队列*/
    osMessageQDef(lock_task_msg_q,4,uint32_t);
    lock_task_msg_q_id = osMessageCreate(osMessageQ(lock_task_msg_q),0);
    log_assert_null_ptr(lock_task_msg_q_id);

    /**************************************************************************/  
    /* 任务创建                                                               */
    /**************************************************************************/  
    /*调试任务*/
    osThreadDef(debug_task, debug_task, osPriorityNormal, 0, 256);
    debug_task_hdl = osThreadCreate(osThread(debug_task), NULL);
    log_assert_null_ptr(debug_task_hdl);

    /*看门狗任务*/
    
    osThreadDef(watch_dog_task, watch_dog_task, osPriorityNormal, 0, 256);
    watch_dog_task_hdl = osThreadCreate(osThread(watch_dog_task), NULL);
    log_assert_null_ptr(watch_dog_task_hdl);

    /*锁控任务*/
    osThreadDef(lock_task, lock_task, osPriorityNormal, 0, 256);
    lock_task_hdl = osThreadCreate(osThread(lock_task), NULL);
    log_assert_null_ptr(lock_task_hdl);

    /*压缩机任务*/
    osThreadDef(compressor_task, compressor_task, osPriorityNormal, 0, 256);
    compressor_task_hdl = osThreadCreate(osThread(compressor_task), NULL);
    log_assert_null_ptr(compressor_task_hdl);

    /*ADC任务*/
    osThreadDef(adc_task, adc_task, osPriorityNormal, 0, 256);
    adc_task_hdl = osThreadCreate(osThread(adc_task), NULL);
    log_assert_null_ptr(adc_task_hdl);

    /*温度任务*/
    osThreadDef(temperature_task, temperature_task, osPriorityNormal, 0, 256);
    temperature_task_hdl = osThreadCreate(osThread(temperature_task), NULL);
    log_assert_null_ptr(temperature_task_hdl);

    /*主控器通信任务*/
    osThreadDef(communication_task, communication_task, osPriorityNormal, 0, 400);
    communication_task_hdl = osThreadCreate(osThread(communication_task), NULL);
    log_assert_null_ptr(communication_task_hdl);

}

