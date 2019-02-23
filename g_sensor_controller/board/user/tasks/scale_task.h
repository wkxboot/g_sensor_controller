#ifndef  __SCALE_TASK_H__
#define  __SCALE_TASK_H__

extern osThreadId   scale_task_hdl;
extern osMessageQId scale_task_msg_q_id;
void scale_task(void const * argument);

#define  SCALE_TASK_SCALE_CNT_MAX             8
#define  SCALE_TASK_HARD_CONFIGRATION_ADDR    0x20000

#define  SCALE_TASK_RX_BUFFER_SIZE            32
#define  SCALE_TASK_TX_BUFFER_SIZE            32
#define  SCALE_TASK_FRAME_SIZE_MAX            20

#define  SCALE_TASK_SERIAL_BAUDRATES          115200
#define  SCALE_TASK_SERIAL_DATABITS           8
#define  SCALE_TASK_SERIAL_STOPBITS           1

#define  SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE    osWaitForever
#define  SCALE_TASK_MSG_PUT_TIMEOUT_VALUE     5
#define  SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE   5

#define  SCALE_TASK_FRAME_TIMEOUT_VALUE       2

#define  SCALE_TASK_SUCCESS                   1
#define  SCALE_TASK_FAILURE                   2
/*数字变送器协议*/
/*协议定义开始*/

/*协议ADU*/
#define  SCALE_TASK_ADU_HEAD_OFFSET             0
#define  SCALE_TASK_ADU_HEAD0_VALUE             'M'
#define  SCALE_TASK_ADU_HEAD1_VALUE             'L'
#define  SCALE_TASK_ADU_HEAD_LEN                2
#define  SCALE_TASK_ADU_PDU_LEN_OFFSET          2
#define  SCALE_TASK_ADU_PDU_OFFSET              3
/*协议PDU*/
#define  SCALE_TASK_PDU_SCALE_ADDR_OFFSET       0
#define  SCALE_TASK_PDU_CODE_OFFSET             1
#define  SCALE_TASK_PDU_VALUE_OFFSET            2
/*协议CRC*/
#define  SCALE_TASK_ADU_CRC_LEN                 2

/*操作码*/
#define  SCALE_TASK_PDU_CODE_NET_WEIGHT         0x00  
#define  SCALE_TASK_PDU_CODE_REMOVE_TARE_WEIGHT 0x01
#define  SCALE_TASK_PDU_CODE_CALIBRATION_ZERO   0x02 
#define  SCALE_TASK_PDU_CODE_CALIBRATION_FULL   0x03   

#define  SCALE_TASK_PDU_CODE_MAX                SCALE_TASK_PDU_CODE_CALIBRATION_FULL
/*操作结果码*/
#define  SCALE_TASK_SUCCESS_VALUE               0x00
#define  SCALE_TASK_FAILURE_VALUE               0x01
#define  SCALE_TASK_ERR_NET_WEIGHT              0x7FFF
/*协议定义结束*/


typedef struct
{
    osMutexId mutex;
    uint8_t cnt;
    uint8_t index;
    int16_t net_weight[SCALE_TASK_SCALE_CNT_MAX];

}scale_task_net_weight_msg_t;

typedef struct
{
    uint8_t cnt;
    uint8_t scale_addr[SCALE_TASK_SCALE_CNT_MAX];
}scale_task_configration_msg_t;

typedef struct
{
    uint8_t cnt;
    uint8_t index;
    uint8_t opt_value[SCALE_TASK_SCALE_CNT_MAX];
}scale_task_opt_msg_t;

typedef struct
{
    int handle;
    uint8_t addr;
    uint8_t port;
    uint32_t baud_rates;
    uint8_t data_bits;
    uint8_t stop_bits;
}scale_configration_t;


typedef struct
{
    uint8_t cnt;
    scale_configration_t scale[SCALE_TASK_SCALE_CNT_MAX];
}scale_task_information_t;

typedef struct
{
    uint8_t cnt;
    uint8_t addr[SCALE_TASK_SCALE_CNT_MAX];
}scale_hard_configration_t;


#endif