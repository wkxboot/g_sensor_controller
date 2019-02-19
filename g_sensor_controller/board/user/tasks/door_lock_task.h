#ifndef  __DOOR_LOCK_TASK_H__
#define  __DOOR_LOCK_TASK_H__



extern osThreadId   door_lock_task_hdl;
extern osMessageQId door_lock_task_msg_q_id;
void door_lock_task(void const * argument);


#define  DOOR_LOCK_TASK_MSG_WAIT_TIMEOUT_VALUE       osWaitForever
#define  DOOR_LOCK_TASK_PUT_MSG_TIMEOUT_VALUE        5
#define  DOOR_LOCK_TASK_LOCK_TIMEOUT_VALUE           1700
#define  DOOR_LOCK_TASK_SENSOR_TIMER_TIMEOUT_VALUE   10
/*状态稳定时间*/
#define  LOCK_DOOR_TASK_STATUS_STABLE_TIME           100



#define  DOOR_LOCK_TASK_DOOR_OPEN_STATUS      1
#define  DOOR_LOCK_TASK_DOOR_CLOSE_STATUS     2

#define  DOOR_LOCK_TASK_LOCK_OPEN_STATUS      3
#define  DOOR_LOCK_TASK_LOCK_CLOSE_STATUS     4

#define  DOOR_LOCK_TASK_UNLOCK_SUCCESS        5
#define  DOOR_LOCK_TASK_UNLOCK_FAILURE        6

#define  DOOR_LOCK_TASK_LOCK_SUCCESS          7
#define  DOOR_LOCK_TASK_LOCK_FAILURE          8










#endif