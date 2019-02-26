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

typedef enum
{
    REQ_CONFIGRATION,
    REQ_NET_WEIGHT,
    REQ_REMOVE_TARE_WEIGHT,
    REQ_CALIBRATION_ZERO,
    REQ_CALIBRATION_FULL,
    REQ_FIRMWARE_VERSION,
    RSP_CONFIGRATION,
    RSP_NET_WEIGHT,
    RSP_REMOVE_TARE_WEIGHT,
    RSP_CALIBRATION_ZERO,
    RSP_CALIBRATION_FULL,
    RSP_FIRMWARE_VERSION
}task_msg_type_t;


typedef struct
{
    uint32_t type:8;
    uint32_t value:16;
    uint32_t reserved:8;
}task_msg_t;


TASKS_END


#endif