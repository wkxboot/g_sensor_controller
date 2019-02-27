#include "cmsis_os.h"
#include "scale_task.h"
#include "controller_task.h"
#include "cpu_task.h"
#include "tasks_init.h"
#include "firmware_version.h"
#include "log.h"


void tasks_init()
{
    /*创建任务*/
    /*cpu任务*/
    osThreadDef(cpu_task, cpu_task, osPriorityNormal, 0, 128);
    cpu_task_hdl = osThreadCreate(osThread(cpu_task), NULL);
    log_assert(cpu_task_hdl);

    /*主控器通信任务*/
    osThreadDef(controller_task, controller_task, osPriorityNormal, 0, 256);
    controller_task_hdl = osThreadCreate(osThread(controller_task), NULL);
    log_assert(controller_task_hdl);

    log_info("firmware version: %s.\r\n",FIRMWARE_VERSION_STR);

}

