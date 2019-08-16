#ifndef  __SCALE_TASK_H__
#define  __SCALE_TASK_H__

extern osThreadId   scale_task_hdl;
void scale_task(void const * argument);


#define  SCALE_TASK_RX_BUFFER_SIZE            32
#define  SCALE_TASK_TX_BUFFER_SIZE            32
#define  SCALE_TASK_FRAME_SIZE_MAX            20

#define  SCALE_TASK_SERIAL_BAUDRATES          115200
#define  SCALE_TASK_SERIAL_DATABITS           8
#define  SCALE_TASK_SERIAL_STOPBITS           1


#define  SCALE_TASK_NET_WEIGHT_ERR_VALUE      0x7FFF
#define  SCALE_TASK_SUCCESS                   0xAA
#define  SCALE_TASK_FAIL                      0xBB

#define  SCALE_TASK_PUT_MSG_TIMEOUT           5

enum
{
    SCALE_TASK_MSG_TYPE_NET_WEIGHT,
    SCALE_TASK_MSG_TYPE_REMOVE_TARE_WEIGHT,
    SCALE_TASK_MSG_TYPE_CALIBRATION_ZERO_WEIGHT,
    SCALE_TASK_MSG_TYPE_CALIBRATION_FULL_WEIGHT,
    SCALE_TASK_MSG_TYPE_RSP_NET_WEIGHT,
    SCALE_TASK_MSG_TYPE_RSP_REMOVE_TARE_WEIGHT,
    SCALE_TASK_MSG_TYPE_RSP_CALIBRATION_ZERO_WEIGHT,
    SCALE_TASK_MSG_TYPE_RSP_CALIBRATION_FULL_WEIGHT
};


typedef struct
{
    union 
    {
    struct 
    {  
        uint8_t type;/*请求消息类型*/
        uint8_t addr;/*电子称请求地址*/
        uint8_t index;/*电子秤队列号*/
        int16_t weight;/*请求的校准值*/
        osMessageQId rsp_message_queue_id;/*回应的消息队列id*/
    }request;
    struct
    {
        uint8_t type;/*回应的消息类型*/
        uint8_t addr;/*电子称回应的地址*/
        uint8_t index;/*电子秤队列号*/
        uint8_t result;/*回应的操作结果*/
        int16_t weight;/*回应的净重值*/
        uint32_t flag;
    }response;
    };
}scale_task_message_t;/*电子秤任务消息体*/

    
#define  SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE    osWaitForever
#endif