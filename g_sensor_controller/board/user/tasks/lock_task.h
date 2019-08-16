#ifndef  __LOCK_TASK_H__
#define  __LOCK_TASK_H__



extern osThreadId   lock_task_hdl;
extern osMessageQId lock_task_msg_q_id;
void lock_task(void const * argument);


#define  LOCK_TASK_MSG_WAIT_TIMEOUT                 osWaitForever
#define  LOCK_TASK_PUT_MSG_TIMEOUT                  5
#define  LOCK_TASK_LOCK_TIMEOUT                     980
#define  LOCK_TASK_UNLOCK_TIMEOUT                   980
#define  LOCK_TASK_LOCK_CONTROLLER_TIMER_TIMEOUT    10

/*状态稳定时间*/
#define  LOCK_TASK_STATUS_HOLD_ON_TIME              100
/*手动开门保持时间*/
//#define  LOCK_TASK_MANUAL_UNLOCK_TIME             5000

/*锁任务状态和结果定义*/
#define  LOCK_TASK_STATUS_DOOR_OPEN                  1
#define  LOCK_TASK_STATUS_DOOR_CLOSE                 2

#define  LOCK_TASK_STATUS_LOCK_LOCKED                3
#define  LOCK_TASK_STATUS_LOCK_UNLOCKED              4

#define  LOCK_TASK_SUCCESS                           5
#define  LOCK_TASK_FAIL                              6

/*锁任务消息*/

enum
{
    LOCK_TASK_MSG_TYPE_LOCK_LOCK,
    LOCK_TASK_MSG_TYPE_UNLOCK_LOCK,
    LOCK_TASK_MSG_TYPE_LOCK_STATUS,
    LOCK_TASK_MSG_TYPE_DOOR_STATUS,
    LOCK_TASK_MSG_TYPE_RSP_LOCK_LOCK_RESULT,
    LOCK_TASK_MSG_TYPE_RSP_UNLOCK_LOCK_RESULT,
    LOCK_TASK_MSG_TYPE_RSP_LOCK_STATUS,
    LOCK_TASK_MSG_TYPE_RSP_DOOR_STATUS,
    LOCK_TASK_MSG_TYPE_MANUAL_LOCK_LOCK,
    LOCK_TASK_MSG_TYPE_MANUAL_UNLOCK_LOCK,
    LOCK_TASK_MSG_TYPE_DEBUG_LOCK_LOCK,
    LOCK_TASK_MSG_TYPE_DEBUG_UNLOCK_LOCK
};

typedef struct
{
    union 
    {
    struct 
    {  
        uint8_t type;/*请求消息类型*/
        osMessageQId rsp_message_queue_id;/*回应的消息队列id*/
    }request;
    struct
    {
        uint8_t type;/*回应的消息类型*/
        uint8_t result;/*回应的操作结果*/
        uint8_t status;/*回应的状态值*/
    }response;
    };
}lock_task_message_t;/*锁任务任务消息体*/






#endif