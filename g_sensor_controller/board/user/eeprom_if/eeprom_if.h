#ifndef  __EEPROM_IF_H__
#define  __EEPROM_IF_H__
#include "stdint.h"
#include "fsl_eeprom.h"

#ifdef __cplusplus
    extern "C" {
#endif


/*
* @brief eeprom 初始化
* @param 无
* @param
* @return 0：成功 其他：失败
* @note
*/
int eeprom_if_init(void);

/*
* @brief eeprom 编程
* @param addr 开始地址
* @param src 数据源地址
* @param size 数量
* @return 0：成功 -1：失败
* @note
*/
int eeprom_if_write(int addr,uint8_t *src,int size);
/*
* @brief eeprom 读取
* @param addr 开始地址
* @param dst 存储地址
* @param size 数量
* @return 0：成功 其他：失败
* @note
*/
int eeprom_if_read(int addr,uint8_t *dst,int size);



#define  EEPROM_PRIORITY_BITS                   3
#define  EEPROM_PRIORITY_HIGH                   2  

#define  EEPROM_MAX_INTERRUPT_PRIORITY          (EEPROM_PRIORITY_HIGH << (8 - EEPROM_PRIORITY_BITS))


#ifdef __ICCARM__

#if (defined (__ARM6M__) && (__CORE__ == __ARM6M__))             
#define EEPROM_ENTER_CRITICAL()                                \
{                                                              \
    unsigned int pri_mask;                                     \
    pri_mask = __get_PRIMASK();                                \
    __set_PRIMASK(1);
    
#define EEPROM_EXIT_CRITICAL()                                 \
    __set_PRIMASK(pri_mask);                                   \
}
#elif ((defined (__ARM7EM__) && (__CORE__ == __ARM7EM__)) || (defined (__ARM7M__) && (__CORE__ == __ARM7M__)))

#define EEPROM_ENTER_CRITICAL()                                \
{                                                              \
   unsigned int base_pri;                                      \
   base_pri = __get_BASEPRI();                                 \
   __set_BASEPRI(EEPROM_MAX_INTERRUPT_PRIORITY);   

#define EEPROM_EXIT_CRITICAL()                                 \
   __set_BASEPRI(base_pri);                                    \
}
  #endif
#endif

/*
*  serial critical configuration for KEIL
*/ 
#ifdef __CC_ARM
#if (defined __TARGET_ARCH_6S_M)
#define EEPROM_ENTER_CRITICAL()                                             
 {                                                              \
    unsigned int pri_mask;                                      \
    register unsigned char PRIMASK __asm( "primask");           \
    pri_mask = PRIMASK;                                         \
    PRIMASK = 1u;                                               \
    __schedule_barrier();

#define EEPROM_EXIT_CRITICAL()                                  \
    PRIMASK = pri_mask;                                         \
    __schedule_barrier();                                       \
}
#elif (defined(__TARGET_ARCH_7_M) || defined(__TARGET_ARCH_7E_M))

#define EEPROM_ENTER_CRITICAL()                                 \
 {                                                              \
     unsigned int base_pri;                                     \
     register unsigned char BASEPRI __asm( "basepri");          \
      base_pri = BASEPRI;                                       \
      BASEPRI = EEPROM_MAX_INTERRUPT_PRIORITY;                  \
      __schedule_barrier();

#define EEPROM_EXIT_CRITICAL()                                  \
     BASEPRI = base_pri;                                        \
     __schedule_barrier();                                      \
}
#endif
#endif  



#ifdef __cplusplus
    }
#endif


#endif