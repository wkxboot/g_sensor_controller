#include "board.h"
#include "cmsis_os.h"
#include "xtimer.h"
#include "board.h"
#include "tasks_init.h"
#include "lock_task.h"
#include "communication_task.h"
#include "log.h"

osThreadId   lock_task_hdl;
osMessageQId lock_task_msg_q_id;
osTimerId    lock_controller_timer_id;


typedef struct
{
    struct
    {
     uint8_t status;
     uint32_t hold_on_time;
    }door_sensor;
    struct
    {
     uint8_t status;
     uint32_t hold_on_time;
    }lock_sensor;
    struct
    {
     uint8_t status;
     uint32_t hold_on_time;
    }hole_sensor;
    struct
    {
    uint8_t status;
    uint32_t hold_on_time;
    bool manual_unlock;
    }manual_switch;
}lock_controller_t;


/*锁控对象实体*/
static volatile lock_controller_t lock_controller;

static void lock_controller_timer_expired(void const *argument);


/*
* @brief 锁控定时器初始化
* @param 无
* @return 无
* @note
*/
static void lock_controller_timer_init(void)
{
    osTimerDef(lock_controller_timer,lock_controller_timer_expired);
    lock_controller_timer_id = osTimerCreate(osTimer(lock_controller_timer),osTimerPeriodic,0);
    log_assert_null_ptr(lock_controller_timer_id);
}
/*
* @brief 锁控定时器开始
* @param 无
* @return 无
* @note
*/
static void lock_controller_timer_start(void)
{
    osTimerStart(lock_controller_timer_id,LOCK_TASK_LOCK_CONTROLLER_TIMER_TIMEOUT);  
    log_debug("lock controller timer start.\r\n"); 
}
/*
* @brief 锁控定时器回调
* @param argument 回调参数
* @return 无
* @note
*/
static void lock_controller_timer_expired(void const *argument)
{ 
    osStatus os_status;
    static lock_task_message_t req_msg;

    uint8_t status;
    /*锁传感器状态轮询*/
    status = bsp_lock_sensor_status();
    if (status != lock_controller.lock_sensor.status) {
        lock_controller.lock_sensor.hold_on_time += LOCK_TASK_LOCK_CONTROLLER_TIMER_TIMEOUT;
        if (lock_controller.lock_sensor.hold_on_time >= LOCK_TASK_STATUS_HOLD_ON_TIME) {
            lock_controller.lock_sensor.hold_on_time = 0;
            lock_controller.lock_sensor.status = status;
   
            if (lock_controller.lock_sensor.status == BSP_LOCK_STATUS_UNLOCKED) {
                log_info("lock status change to --> UNLOCKED.\r\n");
            } else {
                log_info("lock status change to --> LOCKED.\r\n");
            }
        }
    } else {
        lock_controller.lock_sensor.hold_on_time = 0;
    }
    /*锁孔传感器状态轮询*/
    status = bsp_hole_sensor_status();
    if (status != lock_controller.hole_sensor.status) {
        lock_controller.hole_sensor.hold_on_time += LOCK_TASK_LOCK_CONTROLLER_TIMER_TIMEOUT;
        if (lock_controller.hole_sensor.hold_on_time >= LOCK_TASK_STATUS_HOLD_ON_TIME) {
            lock_controller.hole_sensor.hold_on_time = 0;
            lock_controller.hole_sensor.status = status;
   
            if (lock_controller.hole_sensor.status == BSP_HOLE_STATUS_OPEN) {
                log_info("hole status change to --> OPEN.\r\n");
            } else {
                log_info("hole status change to --> CLOSE.\r\n");
            }
        }
    } else {
        lock_controller.hole_sensor.hold_on_time = 0;
    }
    /*门磁传感器状态轮询*/
    status = bsp_door_sensor_status();
    if (status != lock_controller.door_sensor.status) {
        lock_controller.door_sensor.hold_on_time += LOCK_TASK_LOCK_CONTROLLER_TIMER_TIMEOUT;
        if (lock_controller.door_sensor.hold_on_time >= LOCK_TASK_STATUS_HOLD_ON_TIME) {
            lock_controller.door_sensor.hold_on_time = 0;
            lock_controller.door_sensor.status = status;
   
            if (lock_controller.door_sensor.status == BSP_DOOR_STATUS_OPEN) {
                log_info("door status change to --> OPEN.\r\n");
            } else {
                log_info("door status change to --> CLOSE.\r\n");
            }
        }
    } else {
        lock_controller.door_sensor.hold_on_time = 0;
    }

    /*手动按键状态轮询*/
    status = bsp_unlock_sw_status();
    if (status != lock_controller.manual_switch.status) {
        lock_controller.manual_switch.hold_on_time += LOCK_TASK_LOCK_CONTROLLER_TIMER_TIMEOUT;
        if (lock_controller.manual_switch.hold_on_time >= LOCK_TASK_STATUS_HOLD_ON_TIME) {
            lock_controller.manual_switch.hold_on_time = 0;
            lock_controller.manual_switch.status = status;

            if (status == BSP_UNLOCK_SW_STATUS_PRESS) {
                if (lock_controller.manual_switch.manual_unlock == false) {
                    lock_controller.manual_switch.manual_unlock = true;
        
                    /*检测到手动开门*/
                    req_msg.request.type = LOCK_TASK_MSG_TYPE_MANUAL_UNLOCK_LOCK;
                    os_status = osMessagePut(lock_task_msg_q_id,(uint32_t)&req_msg,0);
                    if (os_status != osOK) {
                        log_error("put manual unlock msg err:%d.\r\n",status);
                    }
                }
            } else {
                if (lock_controller.manual_switch.manual_unlock == true) {
                    lock_controller.manual_switch.manual_unlock = false;
                    /*检测到松手关门*/
                    req_msg.request.type = LOCK_TASK_MSG_TYPE_MANUAL_LOCK_LOCK;
                    os_status = osMessagePut(lock_task_msg_q_id,(uint32_t)&req_msg,0);
                    if (os_status != osOK) {
                        log_error("put manual lock msg err:%d.\r\n",status);
                    }
                }
            }

         }
    } else {
        lock_controller.manual_switch.hold_on_time = 0;
    }
    
}

/*
* @brief 锁控任务
* @param argument 任务参数
* @return 无
* @note
*/
void lock_task(void const *argument)
{
    osStatus   status;
    osEvent    os_event;
    lock_task_message_t req_msg,rsp_msg;
    xtimer_t timer;
 
 
    osMessageQDef(lock_task_msg_q,4,uint32_t);
    lock_task_msg_q_id = osMessageCreate(osMessageQ(lock_task_msg_q),lock_task_hdl);
    log_assert_null_ptr(lock_task_msg_q_id); 
 
    lock_controller_timer_init();
    lock_controller_timer_start();
 
    while (1) {
        os_event = osMessageGet(lock_task_msg_q_id,LOCK_TASK_MSG_WAIT_TIMEOUT);
        if (os_event.status == osEventMessage) {
            req_msg = *(lock_task_message_t *)os_event.value.v;
 
        /*获取门状态*/
        if (req_msg.request.type == LOCK_TASK_MSG_TYPE_DOOR_STATUS) {   
            rsp_msg.response.type = LOCK_TASK_MSG_TYPE_RSP_DOOR_STATUS;
  
            if (lock_controller.door_sensor.status == BSP_DOOR_STATUS_OPEN) { 
                rsp_msg.response.status = LOCK_TASK_STATUS_DOOR_OPEN;
            } else {
                rsp_msg.response.status = LOCK_TASK_STATUS_DOOR_CLOSE; 
            }
            status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t)&rsp_msg,LOCK_TASK_PUT_MSG_TIMEOUT);
            if (status != osOK) {
                log_error("lock put door status msg err:%d.\r\n",status);
            }
        }

        /*获取锁状态*/
        if (req_msg.request.type == LOCK_TASK_MSG_TYPE_LOCK_STATUS) {   
            rsp_msg.response.type = LOCK_TASK_MSG_TYPE_RSP_LOCK_STATUS;
  
            if (lock_controller.lock_sensor.status == BSP_LOCK_STATUS_LOCKED) { 
                rsp_msg.response.status = LOCK_TASK_STATUS_LOCK_LOCKED;
            } else {
                rsp_msg.response.status = LOCK_TASK_STATUS_LOCK_UNLOCKED; 
            }
            status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t)&rsp_msg,LOCK_TASK_PUT_MSG_TIMEOUT);
            if (status != osOK) {
                log_error("lock put door status msg err:%d.\r\n",status);
            }
        }

        /*开锁*/
        if (req_msg.request.type == LOCK_TASK_MSG_TYPE_UNLOCK_LOCK){ 
            log_debug("unlock lock...\r\n");
            /*执行开锁操作*/
            bsp_lock_ctrl_open();

            xtimer_init(&timer,0,LOCK_TASK_UNLOCK_TIMEOUT);

            while (xtimer_value(&timer) > 0 && (lock_controller.lock_sensor.status != BSP_LOCK_STATUS_UNLOCKED || lock_controller.hole_sensor.status != BSP_HOLE_STATUS_OPEN) ){
                osDelay(10);
            }
            rsp_msg.response.type = LOCK_TASK_MSG_TYPE_RSP_UNLOCK_LOCK_RESULT;
            if (lock_controller.lock_sensor.status == BSP_LOCK_STATUS_UNLOCKED && lock_controller.hole_sensor.status == BSP_HOLE_STATUS_OPEN) {               
                rsp_msg.response.result = LOCK_TASK_SUCCESS;
                log_debug("unlock success.\r\n");
            } else {
                /*如果开锁失败，就把锁关闭*/
                bsp_lock_ctrl_close();
                rsp_msg.response.result = LOCK_TASK_FAIL;
                log_error("unlock fail.timeout.\r\n");
            }
   
            status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t)&rsp_msg,LOCK_TASK_PUT_MSG_TIMEOUT);
            if (status != osOK){
                log_error("lock put unlock msg err:%d.\r\n",status);
            } 
        }
 
        /*关锁*/
        if (req_msg.request.type == LOCK_TASK_MSG_TYPE_LOCK_LOCK){ 
            log_debug("lock lock...\r\n");
            /*只有在没手动开门的情况下，才执行关锁操作*/
            if (lock_controller.manual_switch.manual_unlock == false) {
                bsp_lock_ctrl_close();
            }

            xtimer_init(&timer,0,LOCK_TASK_LOCK_TIMEOUT);

            while (xtimer_value(&timer) > 0 && (lock_controller.lock_sensor.status != BSP_LOCK_STATUS_LOCKED || lock_controller.hole_sensor.status != BSP_HOLE_STATUS_CLOSE) ){
                osDelay(10);
            }
            rsp_msg.response.type = LOCK_TASK_MSG_TYPE_RSP_LOCK_LOCK_RESULT;
            if (lock_controller.lock_sensor.status == BSP_LOCK_STATUS_LOCKED && lock_controller.hole_sensor.status == BSP_HOLE_STATUS_CLOSE) {               
                rsp_msg.response.result = LOCK_TASK_SUCCESS;
                log_debug("lock success.\r\n");
            } else {
                /*如果关锁失败，就把锁打开*/
                bsp_lock_ctrl_open();
                rsp_msg.response.result = LOCK_TASK_FAIL;
                log_error("lock fail.timeout.\r\n");
            }
   
            status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t)&rsp_msg,LOCK_TASK_PUT_MSG_TIMEOUT);
            if (status != osOK){
                log_error("lock put lock msg err:%d.\r\n",status);
            } 
        }

        /*手动按键开锁*/
        if (req_msg.request.type == LOCK_TASK_MSG_TYPE_MANUAL_UNLOCK_LOCK){ 
            log_info("manual unlock lock...\r\n");
            /*执行开锁操作*/
            bsp_lock_ctrl_open();
        }

        /*手动按键关锁*/
        if (req_msg.request.type == LOCK_TASK_MSG_TYPE_MANUAL_LOCK_LOCK){ 
            log_info("manual lock lock...\r\n");
            /*执行开锁操作*/
            bsp_lock_ctrl_close();
        }


        /*调试接口上锁*/
        if (req_msg.request.type == LOCK_TASK_MSG_TYPE_DEBUG_LOCK_LOCK){ 
            log_info("debug lock lock...\r\n");
            /*执行上锁操作*/
            bsp_lock_ctrl_close();
        }

        /*调试接口开锁*/
        if (req_msg.request.type == LOCK_TASK_MSG_TYPE_DEBUG_UNLOCK_LOCK){ 
            log_info("debug unlock lock...\r\n");
            /*执行开锁操作*/
            bsp_lock_ctrl_open();
        }
   
 }
 
 }
}