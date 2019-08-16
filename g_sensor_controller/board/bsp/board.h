/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
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

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "fsl_wwdt.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/
#define  BSP_UNLOCK_SW_STATUS_PRESS_LEVEL   0
#define  BSP_UNLOCK_SW_STATUS_RELEASE_LEVEL 1

#define  BSP_LOCK_UNLOCKED_LEVEL            0
#define  BSP_LOCK_LOCKED_LEVEL              1

#define  BSP_HOLE_OPEN_LEVEL                1
#define  BSP_HOLE_CLOSE_LEVEL               0

#define  BSP_DOOR_OPEN_LEVEL                1
#define  BSP_DOOR_CLOSE_LEVEL               0


   
#define  BSP_DOOR_STATUS_OPEN              0x11
#define  BSP_DOOR_STATUS_CLOSE             0x22
  
#define  BSP_LOCK_STATUS_UNLOCKED          0x33
#define  BSP_LOCK_STATUS_LOCKED            0x44

#define  BSP_HOLE_STATUS_OPEN              0x55
#define  BSP_HOLE_STATUS_CLOSE             0x66

#define  BSP_UNLOCK_SW_STATUS_PRESS        0x77
#define  BSP_UNLOCK_SW_STATUS_RELEASE      0x88

  
int bsp_board_init(void);
/*开压缩机*/
void bsp_compressor_ctrl_pwr_on(void);
/*关压缩机*/
void bsp_compressor_ctrl_pwr_off(void);
/*开锁*/
void bsp_lock_ctrl_open(void);
/*关锁*/
void bsp_lock_ctrl_close(void);
/*门窗传感器状态*/
uint8_t bsp_door_sensor_status(void);
/*锁传感器状态*/
uint8_t bsp_lock_sensor_status(void);
/*锁空接近传感器状态*/
uint8_t bsp_hole_sensor_status(void);
/*手动开锁按键状态*/
uint8_t bsp_unlock_sw_status();



#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
