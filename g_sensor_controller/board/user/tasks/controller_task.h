#ifndef  __CONTROLLER_H__
#define  __CONTROLLER_H__


extern osThreadId   controller_task_hdl;
extern osMessageQId controller_task_msg_q_id;
void controller_task(void const * argument);



#define  CONTROLLER_TASK_SERIAL_PORT                 0
#define  CONTROLLER_TASK_SERIAL_BAUDRATES            115200
#define  CONTROLLER_TASK_SERIAL_DATABITS             8
#define  CONTROLLER_TASK_SERIAL_STOPBITS             1


#define  CONTROLLER_TASK_RX_BUFFER_SIZE              64
#define  CONTROLLER_TASK_TX_BUFFER_SIZE              64


#define  CONTROLLER_TASK_SCALE_CNT_MAX               8
#define  CONTROLLER_TASK_CONTROLLER_ADDR             1
#define  SCALE_TASK_HARD_CONFIGRATION_ADDR           0x20000



#define  SCALE_CNT_MAX                               8
#define  CONTROLLER_TASK_SCALE_DEFAULT_ADDR          1

typedef struct
{
    uint8_t cnt;
    uint8_t value[SCALE_CNT_MAX];
}scale_addr_configration_t;

typedef struct
{
    int handle;
    uint8_t port;
    uint32_t baud_rates;
    uint8_t data_bits;
    uint8_t stop_bits;
    uint8_t addr;
    uint8_t default_addr;
    osMessageQId msg_q_id;
    osMessageQId net_weight_msg_q_id;
    osMessageQId remove_tare_weight_msg_q_id;
    osMessageQId calibration_zero_msg_q_id;
    osMessageQId calibration_full_msg_q_id;
    osThreadId   task_hdl;
}scale_task_configration_t;

typedef struct
{
    uint8_t cnt;
    /*电子秤任务配置信息*/
    scale_task_configration_t task[SCALE_CNT_MAX];
}scale_contex_t;



#endif