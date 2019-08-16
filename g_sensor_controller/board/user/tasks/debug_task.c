#include "board.h"
#include "cmsis_os.h"
#include "cpu_utils.h"
#include "debug_task.h"
#include "lock_task.h"
#include "tasks_init.h"
#include "device_env.h"
#include "log.h"

osThreadId   debug_task_hdl;

/*
* @brief 调试任务
* @param argument 任务参数
* @param
* @return 无
* @note
*/
void debug_task(void const * argument)
{
    osStatus   status;
    char cmd[20];
    uint8_t level;
    uint8_t read_cnt;
    lock_task_message_t lock_msg;

    while (1) {
        osDelay(DEBUG_TASK_INTERVAL); 
   
        read_cnt = log_read(cmd,19);
        cmd[read_cnt] = 0;

        /*设置日志输出等级*/
        if (strncmp(cmd,"set level ",strlen("set level ")) == 0) {
            level = atoi(cmd + strlen("set level "));
            log_set_level(level);
        }
        /*查询cpu使用率*/
        if (strncmp(cmd,"cpu",strlen("cpu")) == 0) {
           log_info("cpu:%d%%.",osGetCPUUsage());
        }
        /*开锁*/
        if (strncmp(cmd,"unlock",strlen("unlock")) == 0) {
            lock_msg.request.type = LOCK_TASK_MSG_TYPE_DEBUG_UNLOCK_LOCK;
            status = osMessagePut(lock_task_msg_q_id,(uint32_t)&lock_msg,0);
            if (status != osOK) {
                log_error("debug put unlock msg err:%d.\r\n",status);
            }
        }
        /*关锁*/
        if (strncmp(cmd,"lock",strlen("lock")) == 0) {
            lock_msg.request.type = LOCK_TASK_MSG_TYPE_DEBUG_LOCK_LOCK;
            status = osMessagePut(lock_task_msg_q_id,(uint32_t)&lock_msg,0);
            if (status != osOK) {
                log_error("debug put lock msg err:%d.\r\n",status);
            }
        }
        /*clear eeprom*/
        if (strncmp(cmd,"clear",strlen("clear")) == 0) {
            if (device_env_clear() == 0) {
                log_info("clear env ok.\r\n");
            } else {
                log_error("clear env err.\r\n");
            }
        } 
 
    }
}  
  
 