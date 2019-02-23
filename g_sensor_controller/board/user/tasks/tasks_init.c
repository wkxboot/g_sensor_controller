#include "cmsis_os.h"
#include "scale_task.h"
#include "controller_task.h"
#include "cpu_task.h"
#include "tasks_init.h"
#include "firmware_version.h"
#include "log.h"


void tasks_init()
{
    osMessageQDef(scale_task_msg_q,10,uint32_t);
    scale_task_msg_q_id = osMessageCreate(osMessageQ(scale_task_msg_q),0);
    /*主控器配置消息队列*/
    osMessageQDef(controller_task_cfg_msg_q,1,uint32_t);
    controller_task_cfg_msg_q_id = osMessageCreate(osMessageQ(controller_task_cfg_msg_q),0);
    log_assert(controller_task_cfg_msg_q_id);
    /*主控器净重消息队列*/
    osMessageQDef(controller_task_net_weight_msg_q,1,uint32_t);
    controller_task_net_weight_msg_q_id = osMessageCreate(osMessageQ(controller_task_net_weight_msg_q),0);
    log_assert(controller_task_net_weight_msg_q_id);
    /*主控器去皮消息队列*/
    osMessageQDef(controller_task_remove_tare_weight_msg_q,1,uint32_t);
    controller_task_remove_tare_weight_msg_q_id = osMessageCreate(osMessageQ(controller_task_remove_tare_weight_msg_q),0);
    log_assert(controller_task_remove_tare_weight_msg_q_id);
    /*主控器0点校准消息队列*/
    osMessageQDef(controller_task_calibration_zero_msg_q,1,uint32_t);
    controller_task_calibration_zero_msg_q_id = osMessageCreate(osMessageQ(controller_task_calibration_zero_msg_q),0);
    log_assert(controller_task_calibration_zero_msg_q_id);
    /*主控器净重消息队列*/
    osMessageQDef(controller_task_calibration_full_msg_q,1,uint32_t);
    controller_task_calibration_full_msg_q_id = osMessageCreate(osMessageQ(controller_task_calibration_full_msg_q),0);
    log_assert(controller_task_calibration_full_msg_q_id);

    /*创建任务*/
    /*cpu任务*/
    osThreadDef(cpu_task, cpu_task, osPriorityNormal, 0, 128);
    cpu_task_hdl = osThreadCreate(osThread(cpu_task), NULL);
    log_assert(cpu_task_hdl);
    /*电子秤任务*/
    osThreadDef(scale_task, scale_task, osPriorityNormal, 0, 256);
    scale_task_hdl = osThreadCreate(osThread(scale_task), NULL);
    log_assert(scale_task_hdl);
    /*主控器通信任务*/
    osThreadDef(controller_task, controller_task, osPriorityNormal, 0, 256);
    controller_task_hdl = osThreadCreate(osThread(controller_task), NULL);
    log_assert(controller_task_hdl);

    log_info("firmware version: %s.\r\n",FIRMWARE_VERSION_STR);

}

