#include "board.h"
#include "cmsis_os.h"
#include "cpu_utils.h"
#include "cpu_task.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "log.h"

osThreadId   cpu_task_hdl;

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

void cpu_task(void const * argument)
{
    char cmd[20];
    uint8_t level;
    uint8_t read_cnt;

    while (1) {
        log_debug("cpu:%d%%.",osGetCPUUsage());
        bsp_sys_led_toggle();
        osDelay(250); 
 
        
    /*设置日志输出等级*/
    read_cnt = log_read(cmd,19);
    cmd[read_cnt] = 0;
    if (strncmp(cmd,"set level ",strlen("set level ")) == 0) {
        level = atoi(cmd + strlen("set level "));
        log_set_level(level);
    }
 
 
    }
}  
  
 