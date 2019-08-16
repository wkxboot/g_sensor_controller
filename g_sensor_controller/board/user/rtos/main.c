/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "cmsis_os.h"
#include "board.h"
#include "tasks_init.h"
#include "device_env.h"
#include "firmware_version.h"
#include "log.h"


/*
* @brief 硬件延时
* @param 无
* @param
* @return 无
* @note
*/
void hal_delay(void)
{
    volatile uint32_t sleep = 50000000;

    while (sleep --);
}


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

uint32_t log_time()
{
    return osKernelSysTick();
}

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{ 
    bsp_board_init();
    log_init();
    log_info("g sensor firmware version: %s.\r\n",FIRMWARE_VERSION_STR);

    /*环境变量初始化*/
    device_env_init();

#if 0
    /*查看环境变量*/
    flag = device_env_get(ENV_BOOTLOADER_FLAG_NAME);
    /*第一次运行*/
    if (flag == NULL) {
        log_info("first boot.set init...\r\n");
        device_env_set(ENV_BOOTLOADER_FLAG_NAME,ENV_BOOTLOADER_INIT);
        log_info("done.reboot...\r\n");
        /*主动复位*/
        hal_delay();
        __NVIC_SystemReset();
    }

    if (flag) {
        /*如果是更新后第一次运行 就置为OK*/
        if (strcmp(flag,ENV_BOOTLOADER_COMPLETE) == 0) {
            log_info("update first boot.set ok...\r\n");
            device_env_set(ENV_BOOTLOADER_FLAG_NAME,ENV_BOOTLOADER_OK);
            log_info("done.\r\n");
        }

    }
#endif

    tasks_init();
    
    /* Start scheduler */
    osKernelStart();


    while (1) {

    }
}
