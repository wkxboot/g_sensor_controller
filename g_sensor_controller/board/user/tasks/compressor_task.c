#include "cmsis_os.h"
#include "tasks_init.h"
#include "compressor_task.h"
#include "temperature_task.h"
#include "board.h"
#include "log.h"
#define  LOG_MODULE_LEVEL    LOG_LEVEL_DEBUG
#define  LOG_MODULE_NAME     "[c_task]"

osThreadId   compressor_task_hdl;
osMessageQId compressor_task_msg_q_id;


osTimerId    compressor_work_timer_id;
osTimerId    compressor_wait_timer_id;
osTimerId    compressor_rest_timer_id;

static task_msg_t  t_msg;
static task_msg_t  *ptr_msg;

typedef enum
{
COMPRESSOR_STATUS_RDY=0, /*就绪状态*/
COMPRESSOR_STATUS_REST,  /*长时间工作后停机的状态*/
COMPRESSOR_STATUS_WORK,  /*压缩机工作状态*/
COMPRESSOR_STATUS_WAIT   /*两次开机之间的状态*/
}compressor_status_t;


typedef struct
{
osMutexId           mutex;
compressor_status_t status;
}compressor_t;

static compressor_t compressor;

static void compressor_work_timer_init(void);
static void compressor_wait_timer_init(void);
static void compressor_rest_timer_init(void);

static void compressor_work_timer_start(void);
static void compressor_wait_timer_start(void);
static void compressor_rest_timer_start(void);


static void compressor_work_timer_stop(void);
/*
static void compressor_wait_timer_stop(void);
static void compressor_rest_timer_stop(void);
*/

static void compressor_work_timer_expired(void const *argument);
static void compressor_wait_timer_expired(void const *argument);
static void compressor_rest_timer_expired(void const *argument);

static void compressor_pwr_turn_on();
static void compressor_pwr_turn_off();

static void compressor_task_req_temperature();




static void compressor_work_timer_init()
{
 osTimerDef(compressor_work_timer,compressor_work_timer_expired);
 compressor_work_timer_id=osTimerCreate(osTimer(compressor_work_timer),osTimerOnce,0);
 log_assert(compressor_work_timer_id);
}

static void compressor_work_timer_start(void)
{
 osTimerStart(compressor_work_timer_id,COMPRESSOR_TASK_WORK_TIMEOUT);  
 log_debug("压缩机工作定时器开始.\r\n");
}
static void compressor_work_timer_stop(void)
{
 osTimerStop(compressor_work_timer_id);  
 log_debug("压缩机工作定时器停止.\r\n");  
}

static void compressor_work_timer_expired(void const *argument)
{
  /*首先避免竟态，获取mutex*/
  osMutexWait(compressor.mutex,COMPRESSOR_TASK_MUTEX_WAIT_TIMEOUT);
  
  if(compressor.status == COMPRESSOR_STATUS_WORK){
  compressor_pwr_turn_off();
  compressor.status = COMPRESSOR_STATUS_REST;
  compressor_rest_timer_start();
  log_debug("压缩机到达最长工作时间.关压缩机\r\n");
  }
 /*释放mutex*/ 
 osMutexRelease (compressor.mutex);
}


static void compressor_wait_timer_init()
{
 osTimerDef(compressor_wait_timer,compressor_wait_timer_expired);
 compressor_wait_timer_id=osTimerCreate(osTimer(compressor_wait_timer),osTimerOnce,0);
 log_assert(compressor_wait_timer_id);
}


static void compressor_wait_timer_start(void)
{
 osTimerStart(compressor_wait_timer_id,COMPRESSOR_TASK_WAIT_TIMEOUT);  
 log_debug("压缩机等待定时器开始.\r\n");
}

/*
static void compressor_wait_timer_stop(void)
{
 osTimerStop(compressor_wait_timer_id);  
 log_debug("压缩机等待定时器停止.\r\n");  
}
*/

static void compressor_wait_timer_expired(void const *argument)
{
  /*首先避免竟态，获取mutex*/
  osMutexWait(compressor.mutex,COMPRESSOR_TASK_MUTEX_WAIT_TIMEOUT);
  
  if(compressor.status == COMPRESSOR_STATUS_WAIT){
  /*需要请求当前温度值，以判断是否需要控制压缩机*/
   log_debug("压缩机间隔等待完毕.请求温度.\r\n");
   compressor.status = COMPRESSOR_STATUS_RDY;
   compressor_task_req_temperature();
  }    
 /*释放mutex*/ 
 osMutexRelease (compressor.mutex);
}


static void compressor_rest_timer_init(void)
{
 osTimerDef(compressor_rest_timer,compressor_rest_timer_expired);
 compressor_rest_timer_id=osTimerCreate(osTimer(compressor_rest_timer),osTimerOnce,0);
 log_assert(compressor_rest_timer_id);
}

static void compressor_rest_timer_start(void)
{
 osTimerStart(compressor_rest_timer_id,COMPRESSOR_TASK_REST_TIMEOUT);
 log_debug("压缩机休息定时器开始.\r\n");
}

/*
static void compressor_rest_timer_stop(void)
{
 osTimerStop(compressor_rest_timer_id);
 log_debug("压缩机休息定时器停止.\r\n");
}
*/
static void compressor_rest_timer_expired(void const *argument)
{
  /*首先避免竟态，获取mutex*/
  osMutexWait(compressor.mutex,COMPRESSOR_TASK_MUTEX_WAIT_TIMEOUT);
  
   if(compressor.status == COMPRESSOR_STATUS_REST){
   /*需要请求当前温度值，以判断是否需要控制压缩机*/
   log_debug("压缩机休息完毕.请求温度.\r\n");
   compressor.status = COMPRESSOR_STATUS_RDY;
   compressor_task_req_temperature();
  } 
  
 /*释放mutex*/ 
 osMutexRelease (compressor.mutex);
}



static void compressor_pwr_turn_on()
{
 bsp_compressor_ctrl_on(); 
}

static void compressor_pwr_turn_off()
{
 bsp_compressor_ctrl_off();  
}

/*需要请求当前温度值，以判断是否需要控制压缩机*/
static void compressor_task_req_temperature()
{
   osStatus status;
   t_msg.type=REQ_TEMPERATURE; 
   t_msg.req_q_id = compressor_task_msg_q_id;
   status = osMessagePut(temperature_task_msg_q_id,(uint32_t)&t_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
   if(status !=osOK){
   log_error("put temperature req msg error:%d\r\n",status);
  }
}

void compressor_task(void const *argument)
{
  uint8_t i;
  int16_t t;
  osEvent  os_msg;

  /*定义压缩机互斥体*/
  osMutexDef(compressor_mutex);
  compressor.mutex = osMutexCreate(osMutex(compressor_mutex));
  log_assert(compressor.mutex);
    
  osMessageQDef(compressor_msg_q,6,uint32_t);
  compressor_task_msg_q_id = osMessageCreate(osMessageQ(compressor_msg_q),compressor_task_hdl);
  log_assert(compressor_task_msg_q_id);
  /*开机先关闭压缩机*/
  compressor_pwr_turn_off();
  /*等待任务同步*/
  xEventGroupSync(tasks_sync_evt_group_hdl,TASKS_SYNC_EVENT_COMPRESSOR_TASK_RDY,TASKS_SYNC_EVENT_ALL_TASKS_RDY,osWaitForever);
  log_debug("compressor task sync ok.\r\n");
  
  compressor_work_timer_init();
  compressor_wait_timer_init();
  compressor_rest_timer_init();
  osDelay(COMPRESSOT_TASK_WAIT_RDY_TIMEOUT);
  
  /*清空所有已接收的消息*/
  for(i=0;i<6;i++){
  osMessageGet(compressor_task_msg_q_id,0);
  }
  
   /*释放mutex*/ 
   osMutexRelease (compressor.mutex);
   /*需要请求当前温度值，以判断是否需要控制压缩机*/
   log_debug("压缩机上电待机完毕.请求温度.\r\n");
   compressor_task_req_temperature();
  
  while(1){
  os_msg = osMessageGet(compressor_task_msg_q_id,COMPRESSOR_TASK_MSG_WAIT_TIMEOUT);
  if(os_msg.status == osEventMessage){
  /*首先避免竟态*/
  osMutexWait(compressor.mutex,COMPRESSOR_TASK_MUTEX_WAIT_TIMEOUT);
  
  ptr_msg =(task_msg_t*)os_msg.value.v;
   /*温度消息处理*/
  if(ptr_msg->type == RESPONSE_TEMPERATURE ||ptr_msg->type == BROADCAST_TEMPERATURE){ 
  t=ptr_msg->temperature;   
  if(t == TEMPERATURE_ERR_VALUE_SENSOR    ||\
     t == TEMPERATURE_ERR_VALUE_OVER_HIGH ||\
     t == TEMPERATURE_ERR_VALUE_OVER_LOW){ 
      log_error("温度错误.准备关压缩机.code:%d\r\n",t);
      if(compressor.status == COMPRESSOR_STATUS_WORK){
      compressor_work_timer_stop();
      compressor.status = COMPRESSOR_STATUS_WAIT;
      compressor_pwr_turn_off();
      compressor_wait_timer_start();
      log_error("关闭压缩机->wait\r\n");
      }else{
      log_error("压缩机已是关闭状态.skip.\r\n");
      }
   }else if(t >= COMPRESSOR_WORK_TEMPERATURE && compressor.status == COMPRESSOR_STATUS_RDY){
     compressor.status = COMPRESSOR_STATUS_WORK;
     compressor_pwr_turn_on();
     compressor_work_timer_start();
     log_debug("温度:%d 高于开机温度.开启压缩机.\r\n",t);
    }else if(t <= COMPRESSOR_STOP_TEMPERATURE && compressor.status == COMPRESSOR_STATUS_WORK){
     compressor_work_timer_stop();
     compressor.status = COMPRESSOR_STATUS_WAIT;
     compressor_pwr_turn_off();
     compressor_wait_timer_start();
     log_debug("温度:%d 低于关机温度.关闭压缩机.\r\n",t);
    }
    }
   /*释放mutex*/ 
   osMutexRelease (compressor.mutex);
   }
  
  }
 }

