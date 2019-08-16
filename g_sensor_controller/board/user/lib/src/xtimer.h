#ifndef  __XTIMER_H__
#define  __XTIMER_H__

#ifdef  __cplusplus
#define  XTIMER_BEGIN extern "C" {
#define  XTIMER_END   }
#else
#define  XTIMER_BEGIN
#define  XTIMER_END
#endif
 
XTIMER_BEGIN

 /** @brief 定时器定义*/
typedef struct
{
    uint8_t dir; /**< 定时器增长方向*/
    uint32_t timeout;/**< 定时器定时值*/
    uint32_t start;/**< 定时器初始值*/
}xtimer_t;


/**
* @brief 定时器初始化
* @details
* @param timer 定时器指针
* @param dir 定时器增长方向
* @param timeout 定时器超时时间
* @return 初始化结果
* @retval 0 成功
* @retval -1 失败
* @attention
* @note
*/
int xtimer_init(xtimer_t *timer,uint8_t dir,uint32_t timeout);

/**
* @brief 定时器当前值
* @details
* @param timer 定时器指针
* @return 定时器当前值
* @attention 如果是向上增长，当流逝的时间大于超时值时，返回超时值；
*            如果时向下增长，当流逝的时间大于超时值时，返回0 。
* @note
*/
uint32_t xtimer_value(xtimer_t *timer);



XTIMER_END

#endif
