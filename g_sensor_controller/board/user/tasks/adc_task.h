#ifndef  __ADC_TASK_H__
#define  __ADC_TASK_H__
#include "stdint.h"


#ifdef  __cplusplus
#define ADC_TASK_BEGIN  extern "C" {
#define ADC_TASK_END    }
#else
#define ADC_TASK_BEGIN  
#define ADC_TASK_END   
#endif

ADC_TASK_BEGIN


extern osThreadId   adc_task_hdl;
void adc_task(void const * argument);


#define  ADC_TASK_ADC_VALUE_MAX                (4090)
#define  ADC_TASK_ADC_VALUE_MIN                (5)

#define  ADC_TASK_ADC_ERR_MAX                  (100)   /*ADC取样错误最大次数*/  
#define  ADC_TASK_ADC_ERR_VALUE                (0xFFFF)/*ADC取样错误值*/


#define  ADC_TASK_ADC_SAMPLE_MAX               25/*ADC取样次数*/
#define  ADC_TASK_TEMPERATURE_IDX              0 /*温度取样序号*/
#define  ADC_TASK_PRESSURE_IDX                 1 /*压力取样序号*/

#define  ADC_TASK_INTERVAL                     10 /*ADC取样间隔*/
#define  ADC_TASK_ADC_TIMEOUT                  5  /*ADC取样超时时间*/

#define  ADC_TASK_PUT_MSG_TIMEOUT              5  /*发送消息超时时间*/

#define  ADC_TASK_ADC_COMPLETED_SIGNAL         (1<<0)
#define  ADC_TASK_ADC_ERROR_SIGNAL             (1<<1)
#define  ADC_TASK_ALL_SIGNALS                  ((1<<2)-1)

ADC_TASK_END

#endif