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

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/
#define  DATA_STREAM_LED_GPIO    GPIO
#define  DATA_STREAM_LED_PORT    1
#define  DATA_STREAM_LED_PIN     12
 
#define  SYS_LED_GPIO            GPIO
#define  SYS_LED_PORT            0
#define  SYS_LED_PIN             7    

#define  LOCK_CTRL_GPIO          GPIO
#define  LOCK_CTRL_PORT          0
#define  LOCK_CTRL_PIN           31U

#define  LOCK_SENSOR_GPIO        GPIO
#define  LOCK_SENSOR_PORT        1
#define  LOCK_SENSOR_PIN         6U

#define  LOCK_HOLE_SENSOR_GPIO   GPIO
#define  LOCK_HOLE_SENSOR_PORT   0
#define  LOCK_HOLE_SENSOR_PIN    8U
  
#define  DOOR_SENSOR_GPIO        GPIO
#define  DOOR_SENSOR_PORT        1
#define  DOOR_SENSOR_PIN         19U

#define  BSP_LOCK_OPEN_LEVEL          0
#define  BSP_LOCK_CLOSE_LEVEL         1

#define  BSP_LOCK_HOLE_OPEN_LEVEL     1
#define  BSP_LOCK_HOLE_CLOSE_LEVEL    0
#define  BSP_DOOR_OPEN_LEVEL          1
#define  BSP_DOOR_CLOSE_LEVEL         0

#define  BSP_LOCK_STATUS_OPEN         0x33
#define  BSP_LOCK_STATUS_CLOSE        0x44

#define  BSP_LOCK_HOLE_STATUS_OPEN    0x55
#define  BSP_LOCK_HOLE_STATUS_CLOSE   0x66
   
#define  BSP_DOOR_STATUS_OPEN         0x11
#define  BSP_DOOR_STATUS_CLOSE        0x22
  
  
  
int bsp_board_init();
void bsp_data_stream_led_toggle();
void bsp_data_stream_led_on();
void bsp_data_stream_led_off();
void bsp_sys_led_toggle();

void bsp_lock_ctrl_open();
void bsp_lock_ctrl_close();
uint8_t bsp_door_sensor_status();
uint8_t bsp_lock_sensor_status();
uint8_t bsp_lock_hole_sensor_status();

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
