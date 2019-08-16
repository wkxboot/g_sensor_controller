#ifndef  __TEMPERATURE_TASK_H__
#define  __TEMPERATURE_TASK_H__
#include "stdint.h"
#include "stdbool.h"

#ifdef  __cplusplus
#define TEMPERATURE_TASK_BEGIN  extern "C" {
#define TEMPERATURE_TASK_END    }
#else
#define TEMPERATURE_TASK_BEGIN  
#define TEMPERATURE_TASK_END   
#endif


TEMPERATURE_TASK_BEGIN

extern osThreadId   temperature_task_hdl;
extern osMessageQId temperature_task_msg_q_id;

void temperature_task(void const *argument);



#define  TEMPERATURE_TASK_TEMPERATURE_CHANGE_CNT   3 /*连续保持的次数*/

#define  TEMPERATURE_SENSOR_ADC_VALUE_MAX          4096/*温度AD转换最大数值*/      
#define  TEMPERATURE_SENSOR_BYPASS_RES_VALUE       5100/*温度AD转换旁路电阻值*/  
#define  TEMPERATURE_SENSOR_REFERENCE_VOLTAGE      3.30 /*温度传感器参考电压 单位:V*/
#define  TEMPERATURE_SENSOR_SUPPLY_VOLTAGE         3.30 /*温度传感器供电电压 单位:V*/


#define  TEMPERATURE_TASK_MSG_WAIT_TIMEOUT         osWaitForever
#define  TEMPERATURE_TASK_PUT_MSG_TIMEOUT          5    /*发送消息超时时间*/

#define  TEMPERATURE_COMPENSATION_VALUE            0.0  /*温度补偿值,因为温度传感器位置温度与实际温度有误差*/
#define  TEMPERATURE_ALARM_VALUE_MAX               55   /*软件温度高值异常上限 >*/
#define  TEMPERATURE_ALARM_VALUE_MIN               -19  /*软件温度低值异常下限 <*/
#define  TEMPERATURE_ERR_VALUE                     127.0/*错误温度值*/
#define  TEMPERATURE_ACCURATE                      0.1  /*温度精度*/ 
#define  TEMPERATURE_ERR_CNT                       3    /*温度错误确认次数*/ 


#define  ADC_ERR_MAX                               4090
#define  ADC_ERR_MIN                               5

enum
{
    TEMPERATURE_TASK_MSG_TYPE_ADC_COMPLETED,
    TEMPERATURE_TASK_MSG_TYPE_TEMPERATURE,
    TEMPERATURE_TASK_MSG_TYPE_RSP_TEMPERATURE
};

typedef struct
{
    union 
    {
    struct 
    {  
        uint8_t type;/*请求消息类型*/
        uint16_t adc;/*模数转换数值*/
        osMessageQId rsp_message_queue_id;/*回应的消息队列id*/
    }request;
    struct
    {
        uint8_t type;/*回应的消息类型*/
        bool err;/*温度是否错误*/
        int16_t temperature_int;/*回应的温度整数*/
        float temperature_float;/*回应的温度整数*/
    }response;
    };
}temperature_task_message_t;/*温度任务消息体*/


TEMPERATURE_TASK_END

#endif