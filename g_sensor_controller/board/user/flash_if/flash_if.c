#include "flash_if.h"
#include "log.h"


/*
* @brief flash_if_get_sector 获取当前地址扇区号
* @param addr 地址
* @return 扇区号
* @note
*/
static uint32_t flash_if_get_sector(uint32_t addr)
{
    return (addr / FLASH_SECTOR_SIZE);
}

/*
* @brief flash_if_get_page 获取当前地址页号
* @param addr 地址
* @return 页号
* @note
*/
static uint32_t flash_if_get_page(uint32_t addr)
{
    return  (addr / FLASH_PAGE_SIZE);
}



/*
* @brief flash_if_init 数据区域初始化
* @param 无
* @return 0 成功 -1 失败
* @note
*/
int flash_if_init(void)
{

    return 0;
}


/*
* @brief flash_if_read 数据读取
* @param addr 地址
* @param dst 数据目的地址
* @param size 数据量
* @return 0 成功 -1 失败
* @note
*/
int flash_if_read(uint32_t addr,uint8_t *dst,uint32_t size)
{
    for (uint32_t i = 0; i < size; i++ ) {
        dst[i] = *((uint8_t *)addr + i);
    }

    return 0;
}

/*
* @brief 擦除指定地址数据
* @param addr 擦除开始地址
* @param size 擦除数据量
* @return 0：成功 -1：失败
* @note
*/
int flash_if_erase(uint32_t addr,uint32_t size) 
{
    status_t status;

    uint32_t start_page,end_page;
    uint32_t start_sector,end_sector;

    start_page = flash_if_get_page(addr);
    end_page = flash_if_get_page(addr + size - 1);

    start_sector = flash_if_get_sector(addr);
    end_sector = flash_if_get_sector(addr + size - 1);

    log_debug("prepare erase...\r\n");

    NV_FLASH_ENTER_CRITICAL();
    status = FLASHIAP_PrepareSectorForWrite(start_sector,end_sector);
    NV_FLASH_EXIT_CRITICAL();
    if (status != kStatus_Success) {
        log_error("prepare status:%d err.\r\n",status);
        return -1;
    }

    NV_FLASH_ENTER_CRITICAL();
    /*如果是跨扇区删除，使用扇区删除，否则使用页删除*/
    if (end_sector > start_sector) {
        log_debug("erase sector from %d to %d.\r\n",start_sector,end_sector);
        status = FLASHIAP_EraseSector(start_sector,end_sector,SystemCoreClock);
    } else {
        log_debug("erase page from %d to %d.\r\n",start_page,end_page);
        status = FLASHIAP_ErasePage(start_page,end_page,SystemCoreClock);
    }
    NV_FLASH_EXIT_CRITICAL();

    if (status != kStatus_Success) {
        log_error("erase status:%d err.\r\n",status);
        return -1;
    }

    log_debug("erase done.\r\n");
    return 0;
}


/*
* @brief flash_if_write 数据编程写入
* @param addr 地址
* @param src 数据源
* @param size 写入的数据量
* @return 0 成功 -1 失败
* @note
*/
int flash_if_write(uint32_t addr,uint8_t *src,uint32_t size)
{
    status_t status;

    uint32_t start_page,end_page;
    uint32_t start_sector,end_sector;

    start_page = flash_if_get_page(addr);
    end_page = flash_if_get_page(addr + size - 1);



    start_sector = flash_if_get_sector(addr);
    end_sector = flash_if_get_sector(addr + size - 1);

    log_debug("prepare write...\r\n");

    NV_FLASH_ENTER_CRITICAL();
    status = FLASHIAP_PrepareSectorForWrite(start_sector,end_sector);
    NV_FLASH_EXIT_CRITICAL();
    if (status != kStatus_Success) {
        log_error("prepare status:%d err.\r\n",status);
        return -1;
    }
    log_debug("write sector from %d to %d or page from %d to %d.size:%d bytes.\r\n",start_sector,end_sector,start_page,end_page,size);

    NV_FLASH_ENTER_CRITICAL();
    status = FLASHIAP_CopyRamToFlash(addr,(uint32_t*)src,size,SystemCoreClock);
    NV_FLASH_EXIT_CRITICAL();

    if (status != kStatus_Success) {
        log_error("write status:%d err.\r\n",status);
        return -1;
    }

    log_debug("write done.\r\n");
    return 0;
}