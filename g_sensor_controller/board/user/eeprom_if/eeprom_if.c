#include "board.h"
#include "eeprom_if.h"


#define EEPROM_IF_SOURCE_CLOCK           kCLOCK_BusClk
#define EEPROM_IF_CLK_FREQ               CLOCK_GetFreq(kCLOCK_BusClk)
#define EEPROM_IF                        EEPROM

#define FSL_FEATURE_EEPROM_PAGE_SIZE    (FSL_FEATURE_EEPROM_SIZE / FSL_FEATURE_EEPROM_PAGE_COUNT)


/*
* @brief eeprom 初始化
* @param 无
* @param
* @return 0：成功 其他：失败
* @note
*/
int eeprom_if_init(void)
{
    eeprom_config_t config;
    uint32_t sourceClock_Hz = 0;

    /* Init EEPROM */
    EEPROM_GetDefaultConfig(&config);
    config.autoProgram = kEEPROM_AutoProgramDisable;
    sourceClock_Hz = EEPROM_IF_CLK_FREQ;
    EEPROM_Init(EEPROM_IF, &config, sourceClock_Hz);
    
    return 0;
}

/*
* @brief eeprom 编程
* @param addr 开始地址
* @param src 数据源地址
* @param size 数量
* @return 0：成功 -1：失败
* @note
*/
int eeprom_if_write(int addr,uint8_t *src,int size)
{
    if (addr < FSL_FEATURE_EEPROM_BASE_ADDRESS) {
        return -1;
    }
    EEPROM_ENTER_CRITICAL();
    EEPROM_Write(EEPROM_IF,addr - FSL_FEATURE_EEPROM_BASE_ADDRESS,src,size);
    EEPROM_EXIT_CRITICAL();

    return 0;
}

/*
* @brief eeprom 读取
* @param addr 开始地址
* @param dst 存储地址
* @param size 数量
* @return 0：成功 其他：失败
* @note
*/
int eeprom_if_read(int addr,uint8_t *dst,int size)
{
    for (int i = 0;i < size; i++) {
        dst[i] = *((uint8_t *)(addr + i));
    }

    return 0;
}