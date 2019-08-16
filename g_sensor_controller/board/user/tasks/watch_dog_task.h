#ifndef  __WATCH_DOG_TASK_H__
#define  __WATCH_DOG_TASK_H__

/*任务句柄*/
extern osThreadId   watch_dog_task_hdl;

/*
* @brief 看门狗任务
* @param argument 任务参数
* @param
* @return 无
* @note
*/
void watch_dog_task(void const * argument);


#define  WATCH_DOG_TASK_INTERVAL            100








#endif