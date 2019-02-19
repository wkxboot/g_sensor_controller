#ifndef  __TASKS_INIT_H__
#define  __TASKS_INIT_H__

#include "stdint.h"


#ifdef  __cplusplus
#define TASKS_BEGIN  extern "C" {
#define TASKS_END    }
#else
#define TASKS_BEGIN  
#define TASKS_END   
#endif

TASKS_BEGIN

extern EventGroupHandle_t tasks_sync_evt_group_hdl;
void tasks_init();


#define  TASKS_SYNC_EVENT_SCALE_TASK_RDY            (1<<0)
#define  TASKS_SYNC_EVENT_PROTOCOL_TASK_RDY         (1<<1)
#define  TASKS_SYNC_EVENT_DOOR_LOCK_TASK_RDY        (1<<2)
#define  TASKS_SYNC_EVENT_TEMPERATURE_TASK_RDY      (1<<3)
#define  TASKS_SYNC_EVENT_ADC_TASK_RDY              (1<<4)
#define  TASKS_SYNC_EVENT_CPU_TASK_RDY              (1<<5)
#define  TASKS_SYNC_EVENT_COMPRESSOR_TASK_RDY       (1<<6)
#define  TASKS_SYNC_EVENT_ALL_TASKS_RDY             ((1<<6)-1)


typedef enum
{
    REQ_NET_WEIGHT,
    REQ_REMOVE_TAR_WEIGHT,
    REQ_CALIBRATION_ZERO,
    REQ_CALIBRATION_FULL,
    RSP_NET_WEIGHT,
    RSP_REMOVE_TAR_WEIGHT,
    RSP_CALIBRATION_ZERO,
    RSP_CALIBRATION_FULL
}task_msg_type_t;


typedef struct
{
    uint32_t type:8;
    uint32_t value:16;
    uint32_t reserved:8;
}task_msg_t;



TASKS_END




#endif