#include "board.h"
#include "cmsis_os.h"
#include "board.h"
#include "serial.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "protocol_task.h"
#include "door_lock_task.h"
#include "log.h"
#define LOG_MODULE_NAME   "[door_lock]"
#define LOG_MODULE_LEVEL   LOG_LEVEL_INFO


osThreadId   door_lock_task_hdl;
osMessageQId door_lock_task_msg_q_id;
osTimerId    door_lock_sensor_timer_id;

static task_msg_t protocol_msg;
static volatile uint8_t door_status,lock_status,lock_hole_status;
static volatile uint16_t door_status_stable_time,lock_status_stable_time,lock_hole_status_stable_time;


static void door_lock_sensor_timer_expired(void const *argument);



static void door_lock_sensor_timer_init()
{
 osTimerDef(door_lock_sensor_timer,door_lock_sensor_timer_expired);
 door_lock_sensor_timer_id=osTimerCreate(osTimer(door_lock_sensor_timer),osTimerPeriodic,0);
 log_assert(door_lock_sensor_timer_id);
}

static void door_lock_sensor_timer_start(void)
{
 osTimerStart(door_lock_sensor_timer_id,DOOR_LOCK_TASK_SENSOR_TIMER_TIMEOUT_VALUE);  
 log_debug("door lock sensor_timer start.\r\n"); 
}
/*
static void door_lock_sensor_timer_stop(void)
{
 osTimerStop(door_lock_sensor_timer_id);  
 log_debug("door lock sensor_timer stop.\r\n");  
}
*/
static void door_lock_sensor_timer_expired(void const *argument)
{ 
  uint8_t status;
  status =bsp_lock_sensor_status();
  if(status != lock_status){
   lock_status_stable_time+=DOOR_LOCK_TASK_SENSOR_TIMER_TIMEOUT_VALUE;
   if(lock_status_stable_time >= LOCK_DOOR_TASK_STATUS_STABLE_TIME){
   lock_status_stable_time = 0;
   lock_status = status;
   
   if(lock_status == BSP_LOCK_STATUS_OPEN){
   log_info("lock status change to --> OPEN.\r\n");
   }else{
   log_info("lock status change to --> CLOSE.\r\n");
   }
   }
  }else{
  lock_status_stable_time = 0;
  }
 
  status =bsp_lock_hole_sensor_status();
  if(status != lock_hole_status){
   lock_hole_status_stable_time+=DOOR_LOCK_TASK_SENSOR_TIMER_TIMEOUT_VALUE;
   if(lock_hole_status_stable_time >= LOCK_DOOR_TASK_STATUS_STABLE_TIME){
   lock_hole_status_stable_time = 0;
   lock_hole_status = status;
   
   if(lock_hole_status == BSP_LOCK_HOLE_STATUS_OPEN){
   log_info("lock hole status change to --> OPEN.\r\n");
   }else{
   log_info("lock hole status change to --> CLOSE.\r\n");
   }
   }
  }else{
  lock_hole_status_stable_time = 0;
  }
  
  status=bsp_door_sensor_status();
  if(status != door_status){
   door_status_stable_time+=DOOR_LOCK_TASK_SENSOR_TIMER_TIMEOUT_VALUE;
   if(door_status_stable_time >= LOCK_DOOR_TASK_STATUS_STABLE_TIME){
   door_status_stable_time = 0;
   door_status = status;
   
   if(door_status == BSP_DOOR_STATUS_OPEN){
   log_info("door status change to --> OPEN.\r\n");
   }else{
   log_info("door status change to --> CLOSE.\r\n");
   }
   }
  }else{
  door_status_stable_time = 0;
  }
  
  
  
 
}


void door_lock_task(void const * argument)
{
 osStatus   status;
 osEvent    os_msg;
 task_msg_t *msg;

 uint32_t   lock_timeout;
 
 
 osMessageQDef(door_lock_task_msg_q,2,uint32_t);
 door_lock_task_msg_q_id = osMessageCreate(osMessageQ(door_lock_task_msg_q),door_lock_task_hdl);
 log_assert(door_lock_task_msg_q_id); 
 
 /*等待任务同步*/
 xEventGroupSync(tasks_sync_evt_group_hdl,TASKS_SYNC_EVENT_DOOR_LOCK_TASK_RDY,TASKS_SYNC_EVENT_ALL_TASKS_RDY,osWaitForever);
 log_debug("door lock task sync ok.\r\n");
 
 door_lock_sensor_timer_init();
 door_lock_sensor_timer_start();
 
 osDelay(LOCK_DOOR_TASK_STATUS_STABLE_TIME * 2);
 /*上电判断门锁状态*/
 if(door_status == BSP_DOOR_STATUS_OPEN){
  log_info("pwr on.unlock.\r\n");
  bsp_lock_ctrl_open();  
 }
 
 while(1){
 os_msg = osMessageGet(door_lock_task_msg_q_id,DOOR_LOCK_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t *)os_msg.value.v;
 
/*获取锁状态*/
 if(msg->type == REQ_DOOR_STATUS){   
  protocol_msg.type = RESPONSE_DOOR_STATUS;
  
  if(door_status == BSP_DOOR_STATUS_OPEN){ 
   protocol_msg.door_status = DOOR_LOCK_TASK_DOOR_OPEN_STATUS;
   log_debug("door is open.\r\n"); 
  }else{
   protocol_msg.door_status = DOOR_LOCK_TASK_DOOR_CLOSE_STATUS;
  log_debug("door is close.\r\n");  
  }
  status = osMessagePut(protocol_task_msg_q_id,(uint32_t)&protocol_msg,DOOR_LOCK_TASK_PUT_MSG_TIMEOUT_VALUE);
  if(status != osOK){
  log_error("door lock put door status msg err:%d.\r\n",status);
  }
 }
 
 /*获取锁状态*/
 if(msg->type == REQ_LOCK_STATUS){ 
  protocol_msg.type = RESPONSE_LOCK_STATUS;
  
  if(lock_status == BSP_LOCK_STATUS_OPEN){ 
  protocol_msg.lock_status = DOOR_LOCK_TASK_LOCK_OPEN_STATUS;
  log_debug("lock is open.\r\n"); 
  }else{
  protocol_msg.lock_status = DOOR_LOCK_TASK_LOCK_CLOSE_STATUS;
  log_debug("lock is close.\r\n");  
  }
  status = osMessagePut(protocol_task_msg_q_id,(uint32_t)&protocol_msg,DOOR_LOCK_TASK_PUT_MSG_TIMEOUT_VALUE);
  if(status != osOK){
  log_error("door lock put lock status msg err:%d.\r\n",status);
  }
 }
  /*开锁*/
 if(msg->type == REQ_UNLOCK){ 
  log_debug("unlock.\r\n");
  bsp_lock_ctrl_open();
  
  lock_timeout = DOOR_LOCK_TASK_LOCK_TIMEOUT_VALUE;
  while(lock_timeout > 0){
  if(lock_status      != BSP_LOCK_STATUS_OPEN ||
     lock_hole_status != BSP_LOCK_HOLE_STATUS_OPEN){
   osDelay(1);
   lock_timeout--;
  }else{
  break;
  }
  }
  protocol_msg.type = RESPONSE_UNLOCK_RESULT;
  if(lock_timeout > 0){
  protocol_msg.unlock_result = DOOR_LOCK_TASK_UNLOCK_SUCCESS;
  log_info("unlock success.\r\n");
  }else{
  protocol_msg.unlock_result = DOOR_LOCK_TASK_UNLOCK_FAILURE;
  /*如果开锁失败，就把锁关闭*/
  bsp_lock_ctrl_close();
  log_error("unlock fail.\r\n");
  }
  
  status = osMessagePut(protocol_task_msg_q_id,(uint32_t)&protocol_msg,DOOR_LOCK_TASK_PUT_MSG_TIMEOUT_VALUE);
  if(status != osOK){
  log_error("door lock put unlock msg err:%d.\r\n",status);
  }
  
 }
 
 /*关锁*/
 if(msg->type == REQ_LOCK){  
  log_debug("lock.\r\n");
  bsp_lock_ctrl_close();

  lock_timeout = DOOR_LOCK_TASK_LOCK_TIMEOUT_VALUE;
  while(lock_timeout > 0){
  if(lock_status      != BSP_LOCK_STATUS_CLOSE      || \
     lock_hole_status != BSP_LOCK_HOLE_STATUS_CLOSE || \
     door_status      != BSP_DOOR_STATUS_CLOSE) {
   osDelay(1);
   lock_timeout--;
  }else{
  break;
  }
  }
  protocol_msg.type = RESPONSE_LOCK_RESULT;
  if(lock_timeout > 0){
  protocol_msg.unlock_result = DOOR_LOCK_TASK_LOCK_SUCCESS;
  log_info("lock success.\r\n");
  }else{
  protocol_msg.unlock_result = DOOR_LOCK_TASK_LOCK_FAILURE;
  /*如果关锁失败，就把锁打开*/
  bsp_lock_ctrl_open();
  log_error("lock fail.\r\n");
  }
  
  
  status = osMessagePut(protocol_task_msg_q_id,(uint32_t)&protocol_msg,DOOR_LOCK_TASK_PUT_MSG_TIMEOUT_VALUE);
  if(status != osOK){
  log_error("door lock put lock msg err:%d.\r\n",status);
  }
 }
   
 }
 
 }
}