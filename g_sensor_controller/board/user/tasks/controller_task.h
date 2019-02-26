#ifndef  __CONTROLLER_H__
#define  __CONTROLLER_H__


extern osThreadId   controller_task_hdl;
extern osMessageQId controller_task_msg_q_id;
void controller_task(void const * argument);



#define  CONTROLLER_TASK_SERIAL_PORT                 3
#define  CONTROLLER_TASK_SERIAL_BAUDRATES            115200
#define  CONTROLLER_TASK_SERIAL_DATABITS             8
#define  CONTROLLER_TASK_SERIAL_STOPBITS             1


#define  CONTROLLER_TASK_RX_BUFFER_SIZE              64
#define  CONTROLLER_TASK_TX_BUFFER_SIZE              64


#define  CONTROLLER_TASK_SCALE_CNT_MAX               5
#define  CONTROLLER_TASK_CONTROLLER_ADDR             1
#define  SCALE_TASK_HARD_CONFIGRATION_ADDR           0x20000

#endif