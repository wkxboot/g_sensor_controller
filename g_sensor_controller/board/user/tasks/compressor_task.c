#include "cmsis_os.h"
#include "tasks_init.h"
#include "board.h"
#include "compressor_task.h"
#include "device_env.h"
#include "stdlib.h"
#include "stdio.h"
#include "log.h"

/*任务句柄*/
osThreadId   compressor_task_hdl;
/*消息句柄*/
osMessageQId compressor_task_msg_q_id;

/*定时器句柄*/
osTimerId    compressor_timer_id;



typedef enum
{
    COMPRESSOR_STATUS_INIT = 0,        /*上电后的状态*/
    COMPRESSOR_STATUS_WORK,            /*压缩机工作状态*/
    COMPRESSOR_STATUS_STOP_RDY,        /*正常关机后的就绪状态,可以随时启动*/
    COMPRESSOR_STATUS_STOP_REST,       /*长时间工作后的停机休息的状态*/
    COMPRESSOR_STATUS_STOP_CONTINUE,   /*长时间工作后且停机休息完毕后的状态,需要继续治冷到达最低限温度值*/
    COMPRESSOR_STATUS_STOP_WAIT,       /*两次开机之间的停机等待状态*/
    COMPRESSOR_STATUS_STOP_FAULT,      /*温度传感器故障停机等待状态*/
}compressor_status_t;

typedef struct
{
    float stop;
    float work;
}compressor_temperature_level_t;

typedef struct
{
    compressor_status_t status;
    int8_t temperature_int;
    float temperature_float;

    int8_t setting;
    float temperature_stop;
    float temperature_work;

    bool   temperature_err;
}compressor_t;

/*压缩机对象实体*/
static compressor_t compressor ={
.status = COMPRESSOR_STATUS_INIT,
.setting = COMPRESSOR_TASK_TEMPERATURE_SETTING_DEFAULT,
.temperature_stop = COMPRESSOR_TASK_TEMPERATURE_SETTING_DEFAULT - COMPRESSOR_TASK_TEMPERATURE_OFFSET,
.temperature_work = COMPRESSOR_TASK_TEMPERATURE_SETTING_DEFAULT + COMPRESSOR_TASK_TEMPERATURE_OFFSET,
.temperature_err = false
};




static void compressor_timer_init(void);

static void compressor_timer_start(uint32_t timeout);
static void compressor_timer_stop(void);
static void compressor_timer_expired(void const *argument);

static void compressor_pwr_turn_on();
static void compressor_pwr_turn_off();


/*
* @brief 压缩机定时器
* @param 无
* @return 无
* @note
*/
static void compressor_timer_init(void)
{
    osTimerDef(compressor_timer,compressor_timer_expired);
    compressor_timer_id=osTimerCreate(osTimer(compressor_timer),osTimerOnce,0);
    log_assert_bool_false(compressor_timer_id);
}

/*
* @brief 定时器启动
* @param timeout 超时时间
* @return 无
* @note
*/
static void compressor_timer_start(uint32_t timeout)
{
    osTimerStart(compressor_timer_id,timeout);  
}

/*
* @brief 工作时间定时器停止
* @param 无
* @return 无
* @note
*/

static void compressor_timer_stop(void)
{
    osTimerStop(compressor_timer_id);  
}

/*
* @brief 定时器回调
* @param argument 回调参数
* @return 无
* @note
*/
static void compressor_timer_expired(void const *argument)
{
    osStatus status;
    static compressor_task_message_t msg;
    /*发送消息*/
    msg.request.type = COMPRESSOR_TASK_MSG_TYPE_TIMER_TIMEOUT;
    status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
    if (status !=osOK) {
        log_error("compressor put timeout msg error:%d\r\n",status);
    } 
}

/*
* @brief 打开压缩机
* @param 无
* @return 无
* @note
*/
static void compressor_pwr_turn_on(void)
{
    bsp_compressor_ctrl_pwr_on(); 
}

/*
* @brief 关闭压缩机
* @param 无
* @return 无
* @note
*/
static void compressor_pwr_turn_off()
{
    bsp_compressor_ctrl_pwr_off();  
}

/*
* @brief 压缩机任务
* @param argument 任务参数
* @return 无
* @note
*/
void compressor_task(void const *argument)
{
    int rc;
    osEvent  os_event;
    osStatus status;
    int8_t setting;
    char *temperature_str;
    char temperature_str_buffer[7];

    compressor_task_message_t req_msg,req_update_msg,rsp_setting_msg,rsp_query_setting_msg;
    
    /*上电先关闭压缩机*/
    compressor_pwr_turn_off();
    /*定时器初始化*/
    compressor_timer_init();
    /*消除编译警告*/
    compressor_timer_stop();

    /*读取温度配置*/
    temperature_str = device_env_get(COMPRESSOR_TASK_TEMPERATURE_ENV_NAME);

    if (temperature_str == NULL) {
        log_info("setting not exsit in flash.default:%d.\r\n",compressor.setting);
    } else {
        setting = atoi(temperature_str);
        if (setting >= COMPRESSOR_TASK_TEMPERATURE_SETTING_MIN || setting <= COMPRESSOR_TASK_TEMPERATURE_SETTING_MAX) {
            log_info("setting:%d exsit in flash.valid.\r\n",setting);
            compressor.setting = setting;
            compressor.temperature_work = setting + COMPRESSOR_TASK_TEMPERATURE_OFFSET > COMPRESSOR_TASK_TEMPERATURE_MAX ? COMPRESSOR_TASK_TEMPERATURE_MAX : setting + COMPRESSOR_TASK_TEMPERATURE_OFFSET;
            compressor.temperature_stop = setting - COMPRESSOR_TASK_TEMPERATURE_OFFSET < COMPRESSOR_TASK_TEMPERATURE_MIN ? COMPRESSOR_TASK_TEMPERATURE_MIN : setting - COMPRESSOR_TASK_TEMPERATURE_OFFSET;
        } else {
            log_info("setting exsit in flash invalid.default:%d.\r\n",compressor.setting);
        }
    }

    /*打印温度参数*/
    log_info("压缩机工作温度范围:%.2fC~%.2fC.\r\n",compressor.temperature_stop,compressor.temperature_work);



    /*上电等待*/
    compressor_timer_start(COMPRESSOR_TASK_PWR_ON_WAIT_TIMEOUT);
  
    while (1) {
    os_event = osMessageGet(compressor_task_msg_q_id,osWaitForever);
    if (os_event.status == osEventMessage) {     
        req_msg = *(compressor_task_message_t*)os_event.value.v;

        /*压缩机定时器超时消息，压缩机根据超时事件更新工作状态*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_TIMER_TIMEOUT){       
            if (compressor.status == COMPRESSOR_STATUS_INIT) {
                /*上电等待完毕*/
                log_info("压缩机上电等待完毕.\r\n");
                compressor.status = COMPRESSOR_STATUS_STOP_RDY; 
                 /*发送消息更新压缩机工作状态*/
                req_update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS;
                status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_update_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
                if (status != osOK) {
                    log_error("compressor put update msg timeout error:%d\r\n",status);
                }            
            } else if (compressor.status == COMPRESSOR_STATUS_WORK) {
                /*压缩机工作时间到达最大时长*/
                log_info("压缩机到达最大工作时长.停机%d分钟.\r\n",COMPRESSOR_TASK_REST_TIMEOUT / (60 * 1000));
                compressor.status = COMPRESSOR_STATUS_STOP_REST;
                /*关闭压缩机*/
                compressor_pwr_turn_off(); 
                /*等待休息完毕*/
                compressor_timer_start(COMPRESSOR_TASK_REST_TIMEOUT);                
            } else if (compressor.status == COMPRESSOR_STATUS_STOP_REST || compressor.status == COMPRESSOR_STATUS_STOP_WAIT || compressor.status == COMPRESSOR_STATUS_STOP_FAULT) {
                if (compressor.status == COMPRESSOR_STATUS_STOP_REST) {
                    /*压缩机休息完毕*/
                    log_info("压缩机休息完毕.\r\n");
                    compressor.status = COMPRESSOR_STATUS_STOP_CONTINUE;
                } else if (compressor.status == COMPRESSOR_STATUS_STOP_WAIT) {
                    /*压缩机等待完毕*/
                    log_info("压缩机等待完毕.\r\n");
                    compressor.status = COMPRESSOR_STATUS_STOP_RDY;
                } else {
                    /*压缩机传感器错误等待完毕*/
                    log_info("压缩机温度错误等待完毕.\r\n");
                    compressor.status = COMPRESSOR_STATUS_STOP_CONTINUE;
                }
                /*发送消息更新压缩机工作状态*/
                req_update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS;
                status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_update_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
                if (status != osOK) {
                    log_error("compressor put update msg timeout error:%d\r\n",status);
                }          
            } else {
                /*压缩机休息完毕*/
                log_warning("压缩机代码内部错误.忽略.\r\n");          
            }
        }
      
        /*温度更新消息处理*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_UPDATE){ 
            /*去除温度错误标志*/
            compressor.temperature_err = false;
            /*缓存温度值*/
            compressor.temperature_int = req_msg.request.temperature_int; 
            compressor.temperature_float = req_msg.request.temperature_float; 
            /*发送消息更新压缩机工作状态*/
            req_update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS;
            status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_update_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
            if (status != osOK) {
                log_error("compressor put update msg timeout error:%d\r\n",status);
            }   
        }

        /*温度错误消息处理*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_ERR) { 
            compressor.temperature_err = true;
            if (compressor.status == COMPRESSOR_STATUS_WORK){
                /*温度异常时，如果在工作,就变更为STOP_FAULT状态*/
                compressor.status = COMPRESSOR_STATUS_STOP_FAULT;
                /*关闭压缩机和工作定时器*/
                compressor_pwr_turn_off(); 
                /*同样打开等待定时器*/ 
                compressor_timer_start(COMPRESSOR_TASK_WAIT_TIMEOUT);  
                log_info("温度错误.关压缩机.等待%d分钟.\r\n",COMPRESSOR_TASK_WAIT_TIMEOUT / (60 * 1000));
            } else {
                log_info("温度错误.压缩机已经停机状态:%d.跳过.\r\n",compressor.status);
            }
        }
 
        /*压缩机根据温度更新工作状态*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS){ 
            /*只有在没有温度错误和等待上电完毕后才处理压缩机的启停*/
            if (compressor.temperature_err == false && compressor.status != COMPRESSOR_STATUS_INIT) {
                if (compressor.temperature_float <= compressor.temperature_stop && compressor.status == COMPRESSOR_STATUS_WORK){                  
                    compressor.status = COMPRESSOR_STATUS_STOP_WAIT;
                    /*关闭压缩机*/
                    compressor_pwr_turn_off(); 
                    /*打开等待定时器*/ 
                    compressor_timer_start(COMPRESSOR_TASK_WAIT_TIMEOUT);
                    log_info("温度:%.2f C低于关机温度:%.2f C.关机等待%d分钟.\r\n",compressor.temperature_float,compressor.temperature_stop,COMPRESSOR_TASK_WAIT_TIMEOUT / (60 * 1000));
                } else if (compressor.temperature_float >= compressor.temperature_work && compressor.status == COMPRESSOR_STATUS_STOP_RDY ) {       
                    /*温度大于开机温度，同时是正常关机状态时，开机*/
                    compressor.status = COMPRESSOR_STATUS_WORK; 
                    /*打开压缩机*/
                    compressor_pwr_turn_on();
                    /*打开工作定时器*/ 
                    compressor_timer_start(COMPRESSOR_TASK_WORK_TIMEOUT); 
                    log_info("温度:%.2f C高于开机温度:%.2f C.正常开压缩机.\r\n",compressor.temperature_float,compressor.temperature_work);
                }else if (compressor.temperature_float > compressor.temperature_stop && compressor.status == COMPRESSOR_STATUS_STOP_CONTINUE) {
                    /*超时关机或者异常关机状态后，温度大于关机温度，继续开机*/ 
                    compressor.status = COMPRESSOR_STATUS_WORK; 
                    /*打开压缩机*/
                    compressor_pwr_turn_on();
                    /*打开工作定时器*/ 
                    compressor_timer_start(COMPRESSOR_TASK_WORK_TIMEOUT); 
                    log_info("温度:%.2f C高于关机温度:%.2f C.继续开压缩机.\r\n",compressor.temperature_float,compressor.temperature_stop);
                } else {
                    log_info("压缩机状态:%d无需处理.\r\n",compressor.status);
                }
            } else {
                log_info("压缩机正在上电等待或者温度错误.跳过.\r\n");
            }
        }
        /*查询压缩机工作温度配置消息*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_QUERY_TEMPERATURE_SETTING){ 
            /*发送消息给通信任务*/
            rsp_query_setting_msg.response.type = COMPRESSOR_TASK_MSG_TYPE_RSP_QUERY_TEMPERATURE_SETTING;   
            rsp_query_setting_msg.response.temperature_setting = compressor.setting ;  
            status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t)&rsp_query_setting_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
            if (status != osOK) {
                log_error("compressor put rsp query setting msg timeout error:%d\r\n",status);
            } 
        }

        /*配置压缩机工作温度区间消息*/
        if (req_msg.request.type == COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_SETTING){ 
            setting = req_msg.request.temperature_setting;
            if (setting < COMPRESSOR_TASK_TEMPERATURE_SETTING_MIN ||\
                setting > COMPRESSOR_TASK_TEMPERATURE_SETTING_MAX) {
                rsp_setting_msg.response.result = COMPRESSOR_TASK_FAIL;
                log_error("温度设置值:%d ± %d C无效.min:%d C max:%d C.\r\n.",
                            setting,
                            COMPRESSOR_TASK_TEMPERATURE_OFFSET,
                            COMPRESSOR_TASK_TEMPERATURE_SETTING_MIN,
                            COMPRESSOR_TASK_TEMPERATURE_SETTING_MAX);
             } else {              
                rsp_setting_msg.response.result = COMPRESSOR_TASK_SUCCESS;
                if (compressor.setting != setting) {
                    /*复制当前温度区间到缓存*/
                    compressor.temperature_work = setting + COMPRESSOR_TASK_TEMPERATURE_OFFSET > COMPRESSOR_TASK_TEMPERATURE_MAX ? COMPRESSOR_TASK_TEMPERATURE_MAX : setting + COMPRESSOR_TASK_TEMPERATURE_OFFSET;;
                    compressor.temperature_stop = setting - COMPRESSOR_TASK_TEMPERATURE_OFFSET < COMPRESSOR_TASK_TEMPERATURE_MIN ? COMPRESSOR_TASK_TEMPERATURE_MIN : setting - COMPRESSOR_TASK_TEMPERATURE_OFFSET; 
                    compressor.setting = setting;
                    snprintf(temperature_str_buffer,7,"%d",setting);
                    rc = device_env_set(COMPRESSOR_TASK_TEMPERATURE_ENV_NAME,temperature_str_buffer);
                    if (rc != 0) {
                        rsp_setting_msg.response.result = COMPRESSOR_TASK_FAIL;
                        log_error("save temperature setting fail.\r\n");
                    } else {                                
                        log_debug("温度设置值:%dC成功.区间%.2fC~%.2fC.\r\n",setting,compressor.temperature_stop,compressor.temperature_work);
                        /*发送消息更新压缩机工作状态*/
                        req_update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS;
                        status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_update_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
                        if (status != osOK) {
                            log_error("compressor put update msg timeout error:%d\r\n",status);
                        }                     
                    }
                } else {
                    log_debug("温度设置值与当前一致.跳过.\r\n");
                }
            }
            /*发送消息给通信任务*/
            rsp_setting_msg.response.type = COMPRESSOR_TASK_MSG_TYPE_RSP_TEMPERATURE_SETTING;     
            status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t)&rsp_setting_msg,COMPRESSOR_TASK_PUT_MSG_TIMEOUT);
            if (status != osOK) {
                log_error("compressor put rsp_setting msg timeout error:%d\r\n",status);
            }                                    
        }                         


    }
    }
 }


