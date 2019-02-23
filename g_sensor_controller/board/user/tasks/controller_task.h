#ifndef  __CONTROLLER_H__
#define  __CONTROLLER_H__


extern osThreadId   controller_task_hdl;
extern osMessageQId controller_task_cfg_msg_q_id;
extern osMessageQId controller_task_net_weight_msg_q_id;
extern osMessageQId controller_task_remove_tare_weight_msg_q_id;
extern osMessageQId controller_task_calibration_zero_msg_q_id;
extern osMessageQId controller_task_calibration_full_msg_q_id;

void controller_task(void const * argument);


#define  CONTROLLER_TASK_SERIAL_PORT                 4
#define  CONTROLLER_TASK_SERIAL_BAUDRATES            115200
#define  CONTROLLER_TASK_SERIAL_DATABITS             8
#define  CONTROLLER_TASK_SERIAL_STOPBITS             1

#define  CONTROLLER_TASK_SEND_TIMEOUT                5

#define  CONTROLLER_TASK_RX_BUFFER_SIZE              64
#define  CONTROLLER_TASK_TX_BUFFER_SIZE              64

#define  CONTROLLER_TASK_SCALE_CNT_MAX               5
#define  CONTROLLER_TASK_CONTROLLER_ADDR             1


/*协议定义开始*/

/*协议ADU*/
#define  CONTROLLER_TASK_ADU_HEAD_OFFSET             0
#define  CONTROLLER_TASK_ADU_HEAD0_VALUE             'M'
#define  CONTROLLER_TASK_ADU_HEAD1_VALUE             'L'
#define  CONTROLLER_TASK_ADU_HEAD_LEN                2
#define  CONTROLLER_TASK_ADU_PDU_LEN_OFFSET          2
#define  CONTROLLER_TASK_ADU_PDU_OFFSET              3
/*协议PDU*/
#define  CONTROLLER_TASK_PDU_CONTROLLER_ADDR_OFFSET  0
#define  CONTROLLER_TASK_PDU_CODE_OFFSET             1
#define  CONTROLLER_TASK_PDU_SCALE_ADDR_OFFSET       2
#define  CONTROLLER_TASK_PDU_VALUE_OFFSET            3
/*协议CRC*/
#define  CONTROLLER_TASK_ADU_CRC_LEN                 2

/*操作码*/
#define  CONTROLLER_TASK_PDU_CODE_CONFIGRATION       0x00  
#define  CONTROLLER_TASK_PDU_CODE_NET_WEIGHT         0x01   
#define  CONTROLLER_TASK_PDU_CODE_REMOVE_TAR_WEIGHT  0x02
#define  CONTROLLER_TASK_PDU_CODE_CALIBRATION_ZERO   0x03  
#define  CONTROLLER_TASK_PDU_CODE_CALIBRATION_FULL   0x04    
#define  CONTROLLER_TASK_PDU_CODE_FIRMWARE_VERSION   0x05
#define  CONTROLLER_TASK_PDU_CODE_MAX                CONTROLLER_TASK_PDU_CODE_FIRMWARE_VERSION
/*操作结果码*/
#define  CONTROLLER_TASK_SUCCESS_VALUE               0x00
#define  CONTROLLER_TASK_FAILURE_VALUE               0x01
/*协议定义结束*/




#define  CONTROLLER_TASK_FRAME_SIZE_MAX               64




#define  CONTROLLER_TASK_WAIT_TIMEOUT_VALUE          osWaitForever
#define  CONTROLLER_TASK_FRAME_TIMEOUT_VALUE         2

#define  CONTROLLER_TASK_MSG_PUT_TIMEOUT_VALUE       5
#define  CONTROLLER_TASK_MSG_WAIT_TIMEOUT_VALUE      1800




#endif