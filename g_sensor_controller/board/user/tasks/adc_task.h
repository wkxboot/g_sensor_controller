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


#define  ADC_TASK_ADC_SAMPLE_MAX               25/*ADC取样次数*/
#define  ADC_TASK_TEMPERATURE_IDX              0 /*温度1取样序号*/
#define  ADC_TASK_TEMPERATURE2_IDX             1 /*温度2取样序号*/

#define  ADC_TASK_INTERVAL                     40 /*ADC取样间隔*/
#define  ADC_TASK_ADC_TIMEOUT                  10  /*ADC取样超时时间*/

#define  ADC_TASK_PUT_MSG_TIMEOUT              5  /*发送消息超时时间*/

#define  ADC_TASK_ADC_COMPLETED_SIGNAL         (1<<0)
#define  ADC_TASK_ADC_ERROR_SIGNAL             (1<<1)
#define  ADC_TASK_ALL_SIGNALS                  ((1<<2)-1)



ADC_TASK_END

#endif