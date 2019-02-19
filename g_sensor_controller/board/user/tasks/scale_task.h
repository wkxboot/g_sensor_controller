#ifndef  __SCALE_TASK_H__
#define  __SCALE_TASK_H__



extern osThreadId   scale_task_hdl;
extern osMessageQId scale_task_msg_q_id;
void scale_task(void const * argument);



#define  SCALE_TASK_RX_BUFFER_SIZE            32
#define  SCALE_TASK_TX_BUFFER_SIZE            32

#define  SCALE_TASK_SERIAL_PORT               2
#define  SCALE_TASK_SERIAL_BAUDRATES          38400
#define  SCALE_TASK_SERIAL_DATABITS           8
#define  SCALE_TASK_SERIAL_STOPBITS           1

#define  SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE        osWaitForever
#define  SCALE_TASK_MSG_PUT_TIMEOUT_VALUE         5






/*称的协议*/
#define  SCALE_TASK_SET_SENSOR_TIMEOUT        1700
#define  SCALE_TASK_GET_NET_WEIGHT_TIMEOUT    800
#define  SCALE_TASK_SEND_TIMEOUT              10
#define  SCALE_TASK_CHARACTER_TIMEOUT         3
#define  SCALE_TASK_RECV_BUFFER_SIZE          20


#define  SCALE_TASK_GET_NET_WEIGHT_LEN         7
#define  SCALE_TASK_SET_SENSOR_LEN             8

#define  SCALE_TASK_SOF_OFFSET                 0
#define  SCALE_TASK_LEN_OFFSET                 1
#define  SCALE_TASK_ADDR_OFFSET                2
#define  SCALE_TASK_CMD_OFFSET                 3
#define  SCALE_TASK_SENSOR_VALUE_OFF           5
#define  SCALE_TASK_NET_WEIGHT_OFFSET          5


#define  SCALE_TASK_SOF_LEN                     1
#define  SCALE_TASK_LEN_LEN                     1
#define  SCALE_TASK_CRC_LEN                     2

#define  SCALE_TASK_SOF_VALUE                   0x05
#define  SCALE_TASK_ADDR_VALUE                  0x00
#define  SCALE_TASK_CMD_SET_SENSOR_VALUE        0x00E2
#define  SCALE_TASK_CMD_GET_NET_WEIGHT_VALUE    0x01E2
#define  SCALE_TASK_STATUS_OK_VALUE             0x00
#define  SCALE_TASK_STATUS_ERR_VALUE            0x01



#define  SCALTE_TASK_SET_SENSOR_FAILURE         0x11
#define  SCALTE_TASK_SET_SENSOR_SUCCESS         0x22

#define  SCALE_TASK_SCALE_CNT                   4
#define  SCALE_TASK_MANUFACTURE_ID              0x1100

#endif