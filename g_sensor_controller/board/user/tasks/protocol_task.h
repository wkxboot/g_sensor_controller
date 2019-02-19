#ifndef  __PROTOCOL_H__
#define  __PROTOCOL_H__


extern osThreadId   protocol_task_hdl;
extern osMessageQId protocol_task_msg_q_id;
void protocol_task(void const * argument);


#define  PROTOCOL_TASK_SERIAL_PORT                 4
#define  PROTOCOL_TASK_SERIAL_BAUDRATES            115200
#define  PROTOCOL_TASK_SERIAL_DATABITS             8
#define  PROTOCOL_TASK_SERIAL_STOPBITS             1

#define  PROTOCOL_TASK_SEND_TIMEOUT                5

#define  PROTOCOL_TASK_RX_BUFFER_SIZE              64
#define  PROTOCOL_TASK_TX_BUFFER_SIZE              64

#define  PROTOCOL_TASK_SCALE_CNT_MAX               5
#define  PROTOCOL_TASK_CONTROLLER_ADDR             1


/*协议定义开始*/

/*协议ADU*/
#define  PROTOCOL_TASK_ADU_HEAD_OFFSET             0
#define  PROTOCOL_TASK_ADU_HEAD0_VALUE             'M'
#define  PROTOCOL_TASK_ADU_HEAD1_VALUE             'L'
#define  PROTOCOL_TASK_ADU_HEAD_LEN                2
#define  PROTOCOL_TASK_ADU_PDU_LEN_OFFSET          2
#define  PROTOCOL_TASK_ADU_PDU_OFFSET              3
/*协议PDU*/
#define  PROTOCOL_TASK_PDU_CONTROLLER_ADDR_OFFSET  0
#define  PROTOCOL_TASK_PDU_CODE_OFFSET             1
#define  PROTOCOL_TASK_PDU_SCALE_ADDR_OFFSET       2
#define  PROTOCOL_TASK_PDU_VALUE_OFFSET            3
/*协议CRC*/
#define  PROTOCOL_TASK_ADU_CRC_LEN                 2

/*操作码*/
#define  PROTOCOL_TASK_PDU_CODE_CONFIGRATION       0x00  
#define  PROTOCOL_TASK_PDU_CODE_NET_WEIGHT         0x01   
#define  PROTOCOL_TASK_PDU_CODE_REMOVE_TAR_WEIGHT  0x02
#define  PROTOCOL_TASK_PDU_CODE_CALIBRATION_ZERO   0x03  
#define  PROTOCOL_TASK_PDU_CODE_CALIBRATION_FULL   0x04    
#define  PROTOCOL_TASK_PDU_CODE_FIRMWARE_VERSION   0x05
#define  PROTOCOL_TASK_PDU_CODE_MAX                PROTOCOL_TASK_PDU_CODE_FIRMWARE_VERSION
/*操作结果码*/
#define  PROTOCOL_TASK_SUCCESS_VALUE               0x00
#define  PROTOCOL_TASK_FAILURE_VALUE               0x01
/*协议定义结束*/




#define  PROTOCOL_TASK_FRAME_SIZE_MAX               64




#define  PROTOCOL_TASK_WAIT_TIMEOUT_VALUE          osWaitForever
#define  PROTOCOL_TASK_FRAME_TIMEOUT_VALUE         2

#define  PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE       5
#define  PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE      1800


#endif