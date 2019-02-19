#ifndef  __SERIAL_H__
#define  __SERIAL_H__

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "circle_buffer.h"

#ifdef __IAR_SYSTEMS_ICC__
#include <intrinsics.h>
#endif

#ifdef  __cplusplus
# define SERIAL_BEGIN  extern "C" {
# define SERIAL_END    }
#else
# define SERIAL_BEGIN
# define SERIAL_END
#endif

SERIAL_BEGIN

#define  SERIAL_PRIORITY_BITS                       4
#define  SERIAL_PRIORITY_HIGH                       5
#define  SERIAL_MAX_INTERRUPT_PRIORITY             (SERIAL_PRIORITY_HIGH << (8 - SERIAL_PRIORITY_BITS))


typedef struct 
{
    int (*init)(uint8_t port,uint32_t bauds,uint8_t data_bit,uint8_t stop_bit);
    int (*deinit)(uint8_t port);
    void (*enable_txe_it)(uint8_t port);
    void (*disable_txe_it)(uint8_t port);
    void (*enable_rxne_it)(uint8_t port);
    void (*disable_rxne_it)(uint8_t port);
}serial_hal_driver_t;


typedef struct
{
    int                 handle;
    uint8_t             port;
    uint32_t            baud_rates;
    uint8_t             data_bits;
    uint8_t             stop_bits;
    bool                registered;
    bool                init;
    bool                full;
    bool                empty;
    serial_hal_driver_t *driver;
    circle_buffer_t     recv;
    circle_buffer_t     send;
}serial_t;

/*
* @brief  从串口非阻塞的读取指定数量的数据
* @param handle 串口句柄
* @param dst 数据目的地址
* @param size 期望读取的数量
* @return < 0 读取错误
* @return >= 0 实际读取的数量
* @note 可重入
*/
int serial_read(int handle,char *dst,int size);


/*
* @brief  从串口非阻塞的写入指定数量的数据
* @param handle 串口句柄
* @param src 数据源地址
* @param size 期望写入的数量
* @return < 0 写入错误
* @return >= 0 实际写入的数量
* @note 可重入
*/
int serial_write(int handle,const char *src,int size);

/*
* @brief  串口刷新
* @param handle 串口句柄
* @return < 0 失败
* @return >=0 刷新的接收缓存数量
* @note 
*/
int serial_flush(int handle);

/*
* @brief  打开串口
* @param handle 串口句柄
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int serial_open(int handle,uint8_t port,uint32_t baud_rates,uint8_t data_bits,uint8_t stop_bits);

/*
* @brief  关闭串口
* @param handle 串口句柄
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int serial_close(int handle);

/*
* @brief  串口等待数据
* @param handle 串口句柄
* @param timeout 超时时间
* @return < 0 失败
* @return = 0 等待超时
* @return > 0 等待的数据量
* @note 
*/
int serial_select(int handle,uint32_t timeout);

/*
* @brief  串口等待数据发送完毕
* @param handle 串口句柄
* @param timeout 超时时间
* @return < 0 失败
* @return = 0 等待发送超时
* @return > 0 实际发送的数据量
* @note 
*/
int serial_complete(int handle,uint32_t timeout);

/*
* @brief  串口注册硬件驱动
* @param handle 串口句柄
* @param driver 硬件驱动指针
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int serial_register_hal_driver(int handle,serial_hal_driver_t *driver);

/*
* @brief  串口中断发送routine
* @param handle 串口句柄
* @param byte_send 从发送循环缓存中取出的将要发送的一个字节
* @return < 0 失败
* @return = 0 成功
* @note 
*/
int isr_serial_get_byte_to_send(int handle,char *byte_send);

/*
* @brief  串口中断接收routine
* @param handle 串口句柄
* @param byte_send 把从串口读取的一个字节放入接收循环缓存
* @return = 0 失败
* @return = 0 成功
* @note 
*/
int isr_serial_put_byte_from_recv(int handle,char recv_byte);

/*
* @brief  串口创建
* @param handle 串口句柄
* @param rx_size 接收循环缓存容量
* @param tx_size 发送循环缓存容量
* @return = 0 成功
* @return < 0 失败
* @note     rx_size和tx_size必须是2的x次方
*/
int serial_create(int *handle,uint32_t rx_size,uint32_t tx_size);

/*
* @brief  串口销毁
* @param handle 串口句柄
* @return = 0 成功
* @note 
*/
int serial_destroy(int handle);





#define  SERIAL_MALLOC(x)         pvPortMalloc((x))
#define  SERIAL_FREE(x)           vPortFree((x))



/*
*  serial critical configuration for IAR EWARM
*/

#ifdef __ICCARM__
  #if (defined (__ARM6M__) && (__CORE__ == __ARM6M__))             
    #define SERIAL_ENTER_CRITICAL()                            \
    {                                                          \
    unsigned int pri_mask;                                     \
    pri_mask = __get_PRIMASK();                                \
    __set_PRIMASK(1);
    
   #define SERIAL_EXIT_CRITICAL()                              \
    __set_PRIMASK(pri_mask);                                   \
    }
  #elif ((defined (__ARM7EM__) && (__CORE__ == __ARM7EM__)) || (defined (__ARM7M__) && (__CORE__ == __ARM7M__)))

   #define SERIAL_ENTER_CRITICAL()                             \
   {                                                           \
   unsigned int base_pri;                                      \
   base_pri = __get_BASEPRI();                                 \
   __set_BASEPRI(SERIAL_MAX_INTERRUPT_PRIORITY);   

   #define SERIAL_EXIT_CRITICAL()                              \
   __set_BASEPRI(base_pri);                                    \
  }
  #endif
#endif


  
#ifdef __CC_ARM
  #if (defined __TARGET_ARCH_6S_M)
    #define SERIAL_ENTER_CRITICAL()                                             
   {                                                            \
    unsigned int pri_mask;                                      \
    register unsigned char PRIMASK __asm( "primask");           \
    pri_mask = PRIMASK;                                         \
    PRIMASK = 1u;                                               \
    __schedule_barrier();

    #define SERIAL_EXIT_CRITICAL()                              \
    PRIMASK = pri_mask;                                         \
     __schedule_barrier();                                      \
    }
  #elif (defined(__TARGET_ARCH_7_M) || defined(__TARGET_ARCH_7E_M))

    #define SERIAL_ENTER_CRITICAL()                             \
    {                                                           \
     unsigned int base_pri;                                     \
     register unsigned char BASEPRI __asm( "basepri");          \
      base_pri = BASEPRI;                                       \
      BASEPRI = SERIAL_MAX_INTERRUPT_PRIORITY;                  \
      __schedule_barrier();

    #define SERIAL_EXIT_CRITICAL()                              \
     BASEPRI = base_pri;                                        \
     __schedule_barrier();                                      \
    }
  #endif
#endif  
  
  
  



SERIAL_END



#endif

