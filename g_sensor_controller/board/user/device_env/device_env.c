#include "device_env.h"
#if  DEVICE_ENV_USE_EEPROM > 0
#include "eeprom_if.h"
#else
#include "flash_if.h"
#endif
#include "crc16.h"
#include "log.h"

/*****************************************************************************
*  设备环境                                                          
*  Copyright (C) 2019 wkxboot 1131204425@qq.com.                             
*                                                                            
*                                                                            
*  This program is free software; you can redistribute it and/or modify      
*  it under the terms of the GNU General Public License version 3 as         
*  published by the Free Software Foundation.                                
*                                                                            
*  @file     device_env.c                                                   
*  @brief    设备环境                                                                                                                                                                                             
*  @author   wkxboot                                                      
*  @email    1131204425@qq.com                                              
*  @version  v1.0.0                                                  
*  @date     2019/5/5                                            
*  @license  GNU General Public License (GPL)                                
*                                                                            
*                                                                            
*****************************************************************************/
#define  DEVICE_ENV_DATA_SIZE_LIMIT   (DEVICE_ENV_SIZE_LIMIT - 2)


typedef struct
{
    uint16_t crc;
    uint8_t  data_region[DEVICE_ENV_DATA_SIZE_LIMIT];
}device_env_t;

/*内部静态环境变量*/
static device_env_t device_env;

#if DEVICE_ENV_USE_EEPROM > 0
/*内部静态备份环境变量*/
static device_env_t device_env_backup;
#endif

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static char device_env_get_char(int index) 
{
    return *((char *)(&device_env.data_region[0] + index));
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static char * device_env_get_addr(int index)
{
    return (char *)(&device_env.data_region[0] + index);
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int device_env_match(char *name,int index2) 
{
    while (*name == device_env_get_char(index2 ++)) {
        if (*name ++ == '=') {
            return (index2);
        }
    }
    if (*name == '\0' && device_env_get_char(index2 - 1) == '=') {
        return (index2);
    }

    return (-1);
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static void device_env_crc_update(void)
{
    device_env.crc = calculate_crc16(device_env.data_region,DEVICE_ENV_DATA_SIZE_LIMIT);
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int device_env_crc_check(device_env_t *env) 
{
    log_assert_null_ptr(env);
    
    return env->crc == calculate_crc16(env->data_region,DEVICE_ENV_DATA_SIZE_LIMIT) ? 0 : -1;
}

/*
* @brief 写入非易存储体
* @param addr 读取地址
* @param dst 保存地址
* @param size 数据量
* @return 0：成功 -1：失败
* @note
*/
static int device_env_read(uint32_t addr,uint8_t *dst,uint32_t size) 
{
    int rc;
    
#if DEVICE_ENV_USE_EEPROM > 0
    rc = eeprom_if_read(addr,dst,size);
#else
    rc = flash_if_read(addr,dst,size);
#endif
    if (rc != 0) {
        log_error("env read err.code:%d.\r\n",rc);
        return -1;
    }
    
    return 0;
}

/*
* @brief 写入非易存储体
* @param addr 写入地址
* @param src 数据地址
* @param size 数据量
* @return 0：成功 -1：失败
* @note
*/
static int device_env_write(uint32_t addr,uint8_t *src,uint32_t size) 
{
    int rc;
#if DEVICE_ENV_USE_EEPROM > 0
    rc = eeprom_if_write(addr,src,size);
#else
    rc = flash_if_erase(addr,size);
    if (rc != 0) {
        return -1;
    }
    rc = flash_if_write(addr,src,size);
#endif

    if (rc != 0) {
        log_error("env write err.code:%d.\r\n",rc);
        return -1;
    }  
  
    return 0;
}

/*
* @brief 环境变量值和备份环境变量保存到非易存储体
* @param 无
* @return 0：成功 -1：失败
* @note
*/
static int device_env_do_save(void) 
{
    int rc;

    log_debug("do save...\r\n");
    /*保存环境变量*/
    rc = device_env_write(DEVICE_ENV_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
    if (rc != 0) {
        log_error("do save err.\r\n");
        return -1;     
    }
#if DEVICE_ENV_USE_BACKUP > 0 
    /*保存环境变量备份*/
    rc = device_env_write(DEVICE_ENV_BACKUP_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
    if (rc != 0) {
        log_error("do backup save err.\r\n");
        return -1;     
    }    
#endif

    log_debug("do save ok.\r\n");
    return 0;
}

/*
* @brief 环境变量初始化
* @param 无
* @return 0：成功 -1：失败
* @note
*/
int device_env_init(void) 
{
    int rc;
#if DEVICE_ENV_USE_BACKUP > 0 
    int rc_backup;
#endif
    
    log_info("device env init...\r\n");
    /*初始化接口*/
#if DEVICE_ENV_USE_EEPROM > 0
    rc = eeprom_if_init();
#else 
    rc = flash_if_init();
#endif
    
    if (rc != 0) {
        log_error("env if init err.\r\n");
        return -1;
    }
    log_debug("read env...\r\n");    
    /*读取环境变量*/
    rc = device_env_read(DEVICE_ENV_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
    if (rc != 0) {
        log_error("read env err.\r\n");
        return -1;
    }
    /*对比校验值*/
    rc = device_env_crc_check(&device_env);

#if DEVICE_ENV_USE_BACKUP > 0 
    /*读取备份环境变量*/
    log_debug("read backup env...\r\n");
    rc_backup = device_env_read(DEVICE_ENV_BACKUP_BASE_ADDR,(uint8_t*)&device_env_backup,sizeof(device_env_t));
    if (rc_backup != 0) {
        log_error("read backup env err.\r\n");
        return -1;
    }   
    /*对比校验值*/
    rc_backup = device_env_crc_check(&device_env_backup);
    
    /*如果两者都是有效的*/
    if (rc == 0 && rc_backup == 0) {
        log_info("all env ok.\r\n");
        /*检查两者是否一致 如果不一致 那么环境变量应该是最新的*/
        if (device_env.crc != device_env_backup.crc) {
            device_env_backup = device_env;
            /*同步备份环境变量*/
            rc_backup = device_env_write(DEVICE_ENV_BACKUP_BASE_ADDR,(uint8_t*)&device_env_backup,sizeof(device_env_t));
            if (rc_backup != 0) {
               log_error("sync backup env err.\r\n");
               return -1;
            }
            log_info("sync backup env ok.\r\n");
        }
        
    /*如果环境变量是有效的而备份环境变量是无效的 那么就从环境变量恢复到备份环境变量*/
    } else if (rc == 0 && rc_backup != 0) {
        device_env_backup = device_env;
        rc_backup = device_env_write(DEVICE_ENV_BACKUP_BASE_ADDR,(uint8_t*)&device_env_backup,sizeof(device_env_t));
        if (rc_backup != 0) {
           log_error("recovery backup env err.\r\n");
           return -1;
        }
        log_info("recovery backup env ok.\r\n");
        
    /*如果备份环境变量是有效的而环境变量是无效的 那么就从备份环境变量恢复到环境变量*/
    } else if (rc != 0 && rc_backup == 0) {
        device_env = device_env_backup;
        rc_backup = device_env_write(DEVICE_ENV_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
        if (rc_backup != 0) {
           log_error("recovery env err.\r\n");
           return -1;
        }            
        log_info("recovery env ok.\r\n");  
        
    /*两者都是无效的 环境变量初始化为0*/  
    } else {
        memset(&device_env,0x00,sizeof(device_env_t));
        log_info("all env bad.clear env.\r\n");  
    }
#else
    if (rc == 0) {
        log_info("env ok.");
    } else {
        memset(&device_env,0x00,sizeof(device_env_t));
        log_info("env bad.cleared.\r\n");          
    }    
#endif

    log_info("device env init ok.\r\n");
    return 0;
}

/*
* @brief 环境变量完全清除
* @param 无
* @return 0：成功 -1：失败
* @note
*/
int device_env_clear(void) 
{
    int rc;

    log_info("clear env...\r\n");

    memset(&device_env,0x00,sizeof(device_env_t));
    /*写入环境变量*/
    rc = device_env_write(DEVICE_ENV_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
    if (rc != 0) {
        log_error("clear env err.\r\n");
        return -1;
    }

#if DEVICE_ENV_USE_BACKUP > 0  
    /*写入环境变量备份*/
    rc = device_env_write(DEVICE_ENV_BACKUP_BASE_ADDR,(uint8_t*)&device_env,sizeof(device_env_t));
    if (rc != 0) {
        log_error("clear backup env err.\r\n");
        return -1;
    }
#endif

    log_info("clear env ok.\r\n");

    return 0;
}

/*
* @brief 获取对应名称的环境变量值
* @param name 环境变量名
* @return 环境变量值 or null
* @note
*/
char *device_env_get(char *name) 
{
    int i, next;
    for (i=0; device_env_get_char(i) != '\0'; i = next + 1) {
        int val;
        for (next = i; device_env_get_char(next) != '\0'; ++ next) {
            if (next >= DEVICE_ENV_SIZE_LIMIT) {
                return (NULL);
            }
        }
        if ((val = device_env_match((char *)name, i)) < 0) {
            continue;
        }

        return ((char *)device_env_get_addr(val));
    }

    return (NULL);
}

/*
* @brief 设置环境变量值
* @param name 环境变量名
* @param value 环境变量值
* @return 0：成功 -1：失败
* @note
*/
int device_env_set(char *name,char *value) 
{
    int  len, oldval;

    char *env, *next = NULL;

    char *env_data = device_env_get_addr(0);

    if (strchr(name, '=')) {
        log_error("Error: illegal character '=' in variable name \"%s\"\n", name);
        return -1;
    }

    /*
     * search if variable with this name already exists
     */
    oldval = -1;
    for ( env = env_data; *env; env = next + 1) {
        for (next = env; *next; ++ next);
        if ((oldval = device_env_match((char *)name, env - env_data)) >= 0)
            break;
    }

    /*
     * Delete any existing definition
     */
    if (oldval >= 0) {      
        if (*++next == '\0') {
            if (env > env_data) {
                env--;
            } else {
                *env = '\0';
            }
        } else {
            while(1) {
                *env = *next++;
                if ((*env == '\0') && (*next == '\0')) {
                    break;
                }
                ++env;
            }
        }
        *++env = '\0';
    }
    /* Delete only ? */
    if (value == NULL) {
        device_env_crc_update();
        return device_env_do_save();
    }

    /*
     * Append new definition at the end
     */
    for (env = env_data; *env || *(env + 1); ++ env);

    if (env > env_data) {
        ++ env;
    }
    /*
     * Overflow when:
     * "name" + "=" + "val" +"\0\0"  > ENV_SIZE - (env-env_data)
     */
    len = strlen(name) + 2;
    /* add '=' for first arg, ' ' for all others */
    len += strlen(value) + 1;

    if (len > (&env_data[DEVICE_ENV_DATA_SIZE_LIMIT] - env)) {
        log_error("Error: environment overflow, \"%s\" deleted\n", name);
        return 1;
    }
    while ((*env = *name++) != '\0') {
        env++;
    }


    char *val = value;

    *env = '=';
    while ((*++env = *val++) != '\0');
   

    /* end is marked with double '\0' */
    *++env = '\0';

    /* Update CRC */
    device_env_crc_update();
    return device_env_do_save();
}
