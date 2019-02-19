#ifndef  __SCALE_TASK_H__
#define  __SCALE_TASK_H__



extern osThreadId   scale_task_hdl;
extern osMessageQId scale_task_msg_q_id;
void scale_task(void const * argument);



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

#define  SCALE_TASK_SCALE_CNT_MAX             5

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
#define  SCALE_TASK_PDU_CODE_REMOVE_TAR_WEIGHT  0x01
#define  SCALE_TASK_PDU_CODE_CALIBRATION_ZERO   0x02 
#define  SCALE_TASK_PDU_CODE_CALIBRATION_FULL   0x03   

#define  SCALE_TASK_PDU_CODE_MAX                SCALE_TASK_PDU_CODE_CALIBRATION_FULL
/*操作结果码*/
#define  SCALE_TASK_SUCCESS_VALUE               0x00
#define  SCALE_TASK_FAILURE_VALUE               0x01
/*协议定义结束*/

#endif