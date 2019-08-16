#ifndef  __COMPRESSOR_TASK_H__
#define  __COMPRESSOR_TASK_H__
#include "stdint.h"


#ifdef  __cplusplus
#define COMPRESSOR_TASK_BEGIN  extern "C" {
#define COMPRESSOR_TASK_END    }
#else
#define COMPRESSOR_TASK_BEGIN  
#define COMPRESSOR_TASK_END   
#endif


COMPRESSOR_TASK_BEGIN

extern osThreadId   compressor_task_hdl;
extern osMessageQId compressor_task_msg_q_id;

void compressor_task(void const *argument);


#define  COMPRESSOR_TASK_WORK_TIMEOUT                 (120*60*1000) /*连续工作时间单位:ms*/
#define  COMPRESSOR_TASK_REST_TIMEOUT                 (5*60*1000)   /*连续工作时间后的休息时间单位:ms*/
#define  COMPRESSOR_TASK_WAIT_TIMEOUT                 (5*60*1000)   /*2次开机的等待时间 单位:ms*/

#define  COMPRESSOR_TASK_PUT_MSG_TIMEOUT               5            /*发送消息超时时间 单位:ms*/

#define  COMPRESSOR_TASK_PWR_ON_WAIT_TIMEOUT          (2*60*1000)   /*压缩机上电后等待就绪的时间 单位:ms*/

#define  COMPRESSOR_TASK_SUCCESS                       0
#define  COMPRESSOR_TASK_FAIL                          1

#define  COMPRESSOR_TASK_TEMPERATURE_OFFSET            2 /*目标温度的浮动值*/
#define  COMPRESSOR_TASK_TEMPERATURE_SETTING_MIN       0 /*目标温度可设置的最低值*/
#define  COMPRESSOR_TASK_TEMPERATURE_SETTING_MAX       28/*目标温度可设置的最高值*/
#define  COMPRESSOR_TASK_TEMPERATURE_SETTING_DEFAULT   6 /*默认目标温度值*/

#define  COMPRESSOR_TASK_TEMPERATURE_MIN               -2/*压缩机能够达到的最低温度*/
#define  COMPRESSOR_TASK_TEMPERATURE_MAX               30/*压缩机能够达到的最高温度*/

#define  COMPRESSOR_TASK_TEMPERATURE_ENV_NAME          "temperature"

enum
{
  COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_UPDATE,
  COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_ERR,
  COMPRESSOR_TASK_MSG_TYPE_UPDATE_STATUS,
  COMPRESSOR_TASK_MSG_TYPE_WORK_TIMEOUT,
  COMPRESSOR_TASK_MSG_TYPE_WAIT_TIMEOUT,
  COMPRESSOR_TASK_MSG_TYPE_REST_TIMEOUT,
  COMPRESSOR_TASK_MSG_TYPE_TIMER_TIMEOUT,
  COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_SETTING,
  COMPRESSOR_TASK_MSG_TYPE_RSP_TEMPERATURE_SETTING,
  COMPRESSOR_TASK_MSG_TYPE_QUERY_TEMPERATURE_SETTING,
  COMPRESSOR_TASK_MSG_TYPE_RSP_QUERY_TEMPERATURE_SETTING,
};

typedef struct
{
    union 
    {
    struct 
    {  
        uint8_t type;/*请求消息类型*/
        int8_t temperature_setting;/*设置的温度值*/
        int16_t temperature_int;/*整数温度值*/
        float temperature_float;/*浮点温度*/
        osMessageQId rsp_message_queue_id;/*回应的消息队列id*/
    }request;
    struct
    {
        uint8_t type;/*回应的消息类型*/
        uint8_t result;/*回应的结果*/
        int8_t temperature_setting;/*回应设置的温度值*/
    }response;
    };
}compressor_task_message_t;/*压缩机任务消息体*/


COMPRESSOR_TASK_END

#endif





