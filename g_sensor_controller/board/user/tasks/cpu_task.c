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
    uint8_t read_cnt;

    while (1) {
        log_debug("cpu:%d%%.",osGetCPUUsage());
        bsp_sys_led_toggle();
        osDelay(250); 
 
        read_cnt = log_read(cmd,20);
        if (read_cnt == 2) {
            log_debug("hello.\r\n");
        }
 
 
    }
}  
  
 