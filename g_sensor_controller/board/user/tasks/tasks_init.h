#ifndef  __TASKS_INIT_H__
#define  __TASKS_INIT_H__

#include "stdint.h"


#ifdef  __cplusplus
#define TASKS_BEGIN  extern "C" {
#define TASKS_END    }
#else
#define TASKS_BEGIN  
#define TASKS_END   
#endif

TASKS_BEGIN

void tasks_init(void);


TASKS_END


#endif