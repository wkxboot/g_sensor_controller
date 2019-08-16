#include "cmsis_os.h"
#include "tasks_init.h"
#include "adc_task.h"
#include "temperature_task.h"
#include "board.h"
#include "fsl_adc.h"
#include "fsl_clock.h"
#include "fsl_power.h"
#include "log.h"


osThreadId   adc_task_hdl;

static volatile uint16_t adc_sample[2];
static volatile uint32_t adc_cusum[2];
static volatile uint16_t adc_average[2];

#define   TEMPERATURE_ADC           ADC0
#define   TEMPERATURE_ADC_IRQ_ID    ADC0_SEQA_IRQn
#define   TEMPERATURE_ADC_CHANNEL   3
#define   TEMPERATURE_ADC_CLK_SRC   kFRO_HF_to_ADC_CLK


static adc_result_info_t gAdcResultInfoStruct;
adc_result_info_t *volatile gAdcResultInfoPtr = &gAdcResultInfoStruct;
/*
* @brief adc模块中断句柄
* @param 无
* @param
* @return 无
* @note
*/
void  ADC0_SEQA_IRQHandler()
{
    if (kADC_ConvSeqAInterruptFlag == (kADC_ConvSeqAInterruptFlag & ADC_GetStatusFlags(TEMPERATURE_ADC))) {
        ADC_GetChannelConversionResult(TEMPERATURE_ADC, TEMPERATURE_ADC_CHANNEL, gAdcResultInfoPtr);
        adc_sample[ADC_TASK_TEMPERATURE_IDX] = gAdcResultInfoPtr->result;
        ADC_ClearStatusFlags(TEMPERATURE_ADC, kADC_ConvSeqAInterruptFlag);
        osSignalSet(adc_task_hdl,ADC_TASK_ADC_COMPLETED_SIGNAL);   
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
   exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif 
}
/*
* @brief adc模块时钟电源配置
* @param 无
* @param
* @return 无
* @note
*/
static int adc_clk_pwr_config(void)
{
    /* SYSCON power. */
    POWER_DisablePD(kPDRUNCFG_PD_VDDA);    /* Power on VDDA. */
    POWER_DisablePD(kPDRUNCFG_PD_ADC0);    /* Power on the ADC converter. */
    POWER_DisablePD(kPDRUNCFG_PD_VD2_ANA); /* Power on the analog power supply. */
    POWER_DisablePD(kPDRUNCFG_PD_VREFP);   /* Power on the reference voltage source. */
    POWER_DisablePD(kPDRUNCFG_PD_TS);      /* Power on the temperature sensor. */
    /* Enable the clock. */
    CLOCK_AttachClk(TEMPERATURE_ADC_CLK_SRC);
    CLOCK_EnableClock(kCLOCK_Adc0); 
    return 0;
}
/*
* @brief adc模块初始化
* @param 无
* @param
* @return 无
* @note
*/
static int adc_converter_init(void)
{
    adc_config_t adcConfigStruct;
    adc_conv_seq_config_t adcConvSeqConfigStruct;

/* Configure the converter. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE) & FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE
    adcConfigStruct.clockMode = kADC_ClockSynchronousMode; /* Using sync clock source. */
#endif                                                     /* FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE */
    adcConfigStruct.clockDividerNumber = 4;
#if defined(FSL_FEATURE_ADC_HAS_CTRL_RESOL) & FSL_FEATURE_ADC_HAS_CTRL_RESOL
    adcConfigStruct.resolution = kADC_Resolution12bit;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_RESOL */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL) & FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL
    adcConfigStruct.enableBypassCalibration = false;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_TSAMP) & FSL_FEATURE_ADC_HAS_CTRL_TSAMP
    adcConfigStruct.sampleTimeNumber = 4U;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_TSAMP */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE) & FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE
    adcConfigStruct.enableLowPowerMode = false;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE */
#if defined(FSL_FEATURE_ADC_HAS_TRIM_REG) & FSL_FEATURE_ADC_HAS_TRIM_REG
    adcConfigStruct.voltageRange = kADC_HighVoltageRange;
#endif /* FSL_FEATURE_ADC_HAS_TRIM_REG */
    ADC_Init(TEMPERATURE_ADC, &adcConfigStruct);

#if !(defined(FSL_FEATURE_ADC_HAS_NO_INSEL) && FSL_FEATURE_ADC_HAS_NO_INSEL)
    /* Use the temperature sensor input to channel 0. */
    ADC_EnableTemperatureSensor(TEMPERATURE_ADC, true);
#endif /* FSL_FEATURE_ADC_HAS_NO_INSEL. */

    ADC_Init(TEMPERATURE_ADC, &adcConfigStruct);

    /* Enable channel TEMPERATURE_ADC_CHANNEL's conversion in Sequence A. */
    adcConvSeqConfigStruct.channelMask =(1U << TEMPERATURE_ADC_CHANNEL); /* Includes channel TEMPERATURE_ADC_CHANNEL. */
    adcConvSeqConfigStruct.triggerMask = 0U;
    adcConvSeqConfigStruct.triggerPolarity = kADC_TriggerPolarityPositiveEdge;
    adcConvSeqConfigStruct.enableSingleStep = false;
    adcConvSeqConfigStruct.enableSyncBypass = false;
    adcConvSeqConfigStruct.interruptMode = kADC_InterruptForEachSequence;
    ADC_SetConvSeqAConfig(TEMPERATURE_ADC, &adcConvSeqConfigStruct);
    ADC_EnableConvSeqA(TEMPERATURE_ADC, true); /* Enable the conversion sequence A. */
    /* Clear the result register. */
    
    ADC_DoSoftwareTriggerConvSeqA(TEMPERATURE_ADC);
    while (!ADC_GetChannelConversionResult(TEMPERATURE_ADC, TEMPERATURE_ADC_CHANNEL, &gAdcResultInfoStruct))
    {
    }
    ADC_GetConvSeqAGlobalConversionResult(TEMPERATURE_ADC, &gAdcResultInfoStruct);
    
    NVIC_SetPriority(TEMPERATURE_ADC_IRQ_ID, 3);
    /* Enable the interrupt. */
    /* Enable the interrupt the for sequence A done. */
    ADC_EnableInterrupts(TEMPERATURE_ADC, kADC_ConvSeqAInterruptEnable);
    EnableIRQ(TEMPERATURE_ADC_IRQ_ID);
  
    return 0;
}
/*
* @brief adc模块启动
* @param 无
* @param
* @return 无
* @note
*/
static int adc_start(void)
{
    ADC_DoSoftwareTriggerConvSeqA(TEMPERATURE_ADC); 
    return 0;
}
/*
* @brief adc模块停止
* @param 无
* @param
* @return 无
* @note
*/
static int adc_stop(void)
{
    return 0;
}

/*
* @brief adc模块校准
* @param 无
* @param
* @return -1 失败
* @return 0  成功
* @note
*/
static int adc_calibration(void)
{
    if (ADC_DoSelfCalibration(TEMPERATURE_ADC) == false) {
        log_error("t adc calibrate err.\r\n");
        return -1;
    }
    return 0;
}
/*
* @brief adc模块复位
* @param 无
* @param
* @return -1 失败
* @return 0  成功
* @note
*/
static int adc_reset(void)
{
    return 0;
}

/*
* @brief adc任务
* @param argument 任务参数
* @param
* @return 无
* @note
*/
void adc_task(void const * argument)
{
    int rc;
    osEvent signals;
    osStatus status;
    temperature_task_message_t adc_completed_msg;
    uint8_t t_sample_cnt = 0;

    adc_stop();
    adc_clk_pwr_config();
    do {
        osDelay(1000);
        rc = adc_calibration();
    }while (rc != 0);

    adc_converter_init();

    while (1) {
        osDelay(ADC_TASK_INTERVAL);
        adc_start();

        signals = osSignalWait(ADC_TASK_ALL_SIGNALS,ADC_TASK_ADC_TIMEOUT);
        if (signals.status == osEventSignal ) {
            if (signals.value.signals & ADC_TASK_ADC_COMPLETED_SIGNAL) {
    
                /*temperature adc calculate*/    
                if (t_sample_cnt < ADC_TASK_ADC_SAMPLE_MAX){
                    adc_cusum[ADC_TASK_TEMPERATURE_IDX] += adc_sample[ADC_TASK_TEMPERATURE_IDX];   
                    t_sample_cnt ++;             
                } else {
                    adc_average[ADC_TASK_TEMPERATURE_IDX] = adc_cusum[ADC_TASK_TEMPERATURE_IDX] / t_sample_cnt;  
                    adc_cusum[ADC_TASK_TEMPERATURE_IDX] = 0;
                    t_sample_cnt = 0;
                    adc_completed_msg.request.type = TEMPERATURE_TASK_MSG_TYPE_ADC_COMPLETED;
                    adc_completed_msg.request.adc = adc_average[ADC_TASK_TEMPERATURE_IDX];  
                    status = osMessagePut(temperature_task_msg_q_id,(uint32_t)&adc_completed_msg,ADC_TASK_PUT_MSG_TIMEOUT);
                    if (status != osOK) {
                        log_error("put temperature msg error:%d\r\n",status);
                    }
                }
            }
    

        if (signals.value.signals & ADC_TASK_ADC_ERROR_SIGNAL) {
            log_error("adc error.reset.\r\n");
            adc_reset();
        }
    }
    }
}

