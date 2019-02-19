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


/*协议定义开始*/
#define  PROTOCOL_TASK_CMD_ADDR_OFFSET             0
#define  PROTOCOL_TASK_CMD_ADDR_LEN                1
#define  PROTOCOL_TASK_CMD_ADDR_VALUE              0x01


#define  PROTOCOL_TASK_CMD_CODE_OFFSET             1
#define  PROTOCOL_TASK_CMD_CODE_LEN                1
#define  PROTOCOL_TASK_CMD_NET_WEIGHT              0x03   
#define  PROTOCOL_TASK_CMD_SCALE_CNT               0x04  
#define  PROTOCOL_TASK_CMD_DOOR_STATUS             0x11  
#define  PROTOCOL_TASK_CMD_UNLOCK                  0x21    
#define  PROTOCOL_TASK_CMD_LOCK                    0x22 
#define  PROTOCOL_TASK_CMD_LOCK_STATUS             0x23  
#define  PROTOCOL_TASK_CMD_TEMPERATURE             0x41   
#define  PROTOCOL_TASK_CMD_MANUFACTURE_ID          0x51    
 
#define  PROTOCOL_TASK_CMD_PARAM_OFFSET             2
#define  PROTOCOL_TASK_CMD_NET_WEIGHT_PARAM_LEN     1
#define  PROTOCOL_TASK_CMD_DOOR_STATUS_PARAM_LEN    0
#define  PROTOCOL_TASK_CMD_LOCK_STATUS_PARAM_LEN    0
#define  PROTOCOL_TASK_CMD_TEMPERATURE_PARAM_LEN    0
#define  PROTOCOL_TASK_CMD_MANUFACTURE_ID_PARAM_LEN 0
#define  PROTOCOL_TASK_CMD_SCALE_CNT_PARAM_LEN      0
#define  PROTOCOL_TASK_CMD_UNLOCK_PARAM_LEN         0
#define  PROTOCOL_TASK_CMD_LOCK_PARAM_LEN           0
#define  PROTOCOL_TASK_CMD_CRC_LEN                  2

#define  PROTOCOL_TASK_DOOR_OPEN_STATUS             0x01
#define  PROTOCOL_TASK_DOOR_CLOSE_STATUS            0x00
#define  PROTOCOL_TASK_LOCK_OPEN_STATUS             0x01
#define  PROTOCOL_TASK_LOCK_CLOSE_STATUS            0x00
#define  PROTOCOL_TASK_UNLOCK_SUCCESS               0x01
#define  PROTOCOL_TASK_UNLOCK_FAILURE               0x00
#define  PROTOCOL_TASK_LOCK_SUCCESS                 0x01
#define  PROTOCOL_TASK_LOCK_FAILURE                 0x00


#define  PROTOCOL_TASK_SCALE_CNT_MAX                20
#define  PROTOCOL_TASK_T_ERR_VALUE                  0x7F
/*协议定义结束*/




#define  PROTOCOL_TASK_FRAME_SIZE_MAX               64




#define  PROTOCOL_TASK_FRAME_TIMEOUT_VALUE         osWaitForever
#define  PROTOCOL_TASK_CHARACTER_TIMEOUT_VALUE     3

#define  PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE       5
#define  PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE      1800


#endif