#ifndef  __NV_FLASH_H__
#define  __NV_FLASH_H__
#include "stdint.h"
#include "fsl_flashiap.h"

#ifdef __cplusplus
    extern "C" {
#endif

#define  FLASH_PAGE_SIZE                          FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES
#define  FLASH_SECTOR_SIZE                        FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES             
#define  NV_FLASH_PRIORITY_BITS                   3
#define  NV_FLASH_PRIORITY_HIGH                   2  

#define  NV_FLASH_MAX_INTERRUPT_PRIORITY          (NV_FLASH_PRIORITY_HIGH << (8 - NV_FLASH_PRIORITY_BITS))

/*
* @brief flash_if_init 数据区域初始化
* @param 无
* @return 0 成功 -1 失败
* @note
*/
int flash_if_init(void);

/*
* @brief flash_if_read 数据读取
* @param addr 地址
* @param dst 数据保存的地址
* @param size 数据量
* @return 0 成功 -1 失败
* @note
*/
int flash_if_read(uint32_t addr,uint8_t *dst,uint32_t size);

/*
* @brief 擦除指定地址数据(页擦除)
* @param addr 擦除开始地址
* @param size 擦除数据量
* @return 0：成功 -1：失败
* @note
*/
int flash_if_erase(uint32_t addr,uint32_t size);

/*
* @brief flash_if_write 数据编程写入
* @param addr 读取偏移地址
* @param src 数据保存的地址
* @param size 读取的数据量
* @return 0 成功 -1 失败
* @note
*/
int flash_if_write(uint32_t addr,uint8_t *src,uint32_t size);


#ifdef __ICCARM__

#if (defined (__ARM6M__) && (__CORE__ == __ARM6M__))             
#define NV_FLASH_ENTER_CRITICAL()                              \
{                                                              \
    unsigned int pri_mask;                                     \
    pri_mask = __get_PRIMASK();                                \
    __set_PRIMASK(1);
    
#define NV_FLASH_EXIT_CRITICAL()                               \
    __set_PRIMASK(pri_mask);                                   \
}
#elif ((defined (__ARM7EM__) && (__CORE__ == __ARM7EM__)) || (defined (__ARM7M__) && (__CORE__ == __ARM7M__)))

#define NV_FLASH_ENTER_CRITICAL()                              \
{                                                              \
   unsigned int base_pri;                                      \
   base_pri = __get_BASEPRI();                                 \
   __set_BASEPRI(NV_FLASH_MAX_INTERRUPT_PRIORITY);   

#define NV_FLASH_EXIT_CRITICAL()                               \
   __set_BASEPRI(base_pri);                                    \
}
  #endif
#endif


/*
*  serial critical configuration for KEIL
*/ 
#ifdef __CC_ARM
#if (defined __TARGET_ARCH_6S_M)
#define NV_FLASH_ENTER_CRITICAL()                                             
 {                                                              \
    unsigned int pri_mask;                                      \
    register unsigned char PRIMASK __asm( "primask");           \
    pri_mask = PRIMASK;                                         \
    PRIMASK = 1u;                                               \
    __schedule_barrier();

#define NV_FLASH_EXIT_CRITICAL()                                \
    PRIMASK = pri_mask;                                         \
    __schedule_barrier();                                       \
}
#elif (defined(__TARGET_ARCH_7_M) || defined(__TARGET_ARCH_7E_M))

#define NV_FLASH_ENTER_CRITICAL()                               \
 {                                                              \
     unsigned int base_pri;                                     \
     register unsigned char BASEPRI __asm( "basepri");          \
      base_pri = BASEPRI;                                       \
      BASEPRI = NV_FLASH_MAX_INTERRUPT_PRIORITY;           \
      __schedule_barrier();

#define NV_FLASH_EXIT_CRITICAL()                                \
     BASEPRI = base_pri;                                        \
     __schedule_barrier();                                      \
}
#endif
#endif  





#ifdef __cplusplus
    }
#endif




#endif