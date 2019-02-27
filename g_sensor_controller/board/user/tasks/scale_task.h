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
#define  SCALE_TASK_SUCCESS                   0xaa
#define  SCALE_TASK_FAILURE                   0xbb

#define  SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE    osWaitForever
#endif