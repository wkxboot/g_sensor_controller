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

#include <stdint.h>
#include "fsl_common.h"
#include "clock_config.h"
#include "board.h"



/*压缩机开关控制引脚初始化*/
static void bsp_compressor_ctrl_pin_init(void)
{
    gpio_pin_config_t pin;
    pin.pinDirection = kGPIO_DigitalOutput;
    pin.outputLogic = 0;
    GPIO_PinInit(BOARD_INITPINS_COMPRESSOR_CTRL_GPIO,BOARD_INITPINS_COMPRESSOR_CTRL_PORT,BOARD_INITPINS_COMPRESSOR_CTRL_PIN,&pin);
}

/*开压缩机*/
void bsp_compressor_ctrl_pwr_on(void)
{
    GPIO_PinWrite(BOARD_INITPINS_COMPRESSOR_CTRL_GPIO,BOARD_INITPINS_COMPRESSOR_CTRL_PORT,BOARD_INITPINS_COMPRESSOR_CTRL_PIN,1);
}

/*关压缩机*/
void bsp_compressor_ctrl_pwr_off()
{
    GPIO_PinWrite(BOARD_INITPINS_COMPRESSOR_CTRL_GPIO,BOARD_INITPINS_COMPRESSOR_CTRL_PORT,BOARD_INITPINS_COMPRESSOR_CTRL_PIN,0);
}


/*锁开关控制引脚初始化*/
static void bsp_lock_ctrl_pin_init()
{
    gpio_pin_config_t pin;
    pin.pinDirection = kGPIO_DigitalOutput;
    pin.outputLogic = 1;
    GPIO_PinInit(BOARD_INITPINS_LOCK_CTRL_GPIO,BOARD_INITPINS_LOCK_CTRL_PORT,BOARD_INITPINS_LOCK_CTRL_PIN,&pin);
}

/*开锁*/
void bsp_lock_ctrl_open(void)
{
  GPIO_PinWrite(BOARD_INITPINS_LOCK_CTRL_GPIO,BOARD_INITPINS_LOCK_CTRL_PORT,BOARD_INITPINS_LOCK_CTRL_PIN,0);
}

/*关锁*/
void bsp_lock_ctrl_close()
{
    GPIO_PinWrite(BOARD_INITPINS_LOCK_CTRL_GPIO,BOARD_INITPINS_LOCK_CTRL_PORT,BOARD_INITPINS_LOCK_CTRL_PIN,1);
}

/*手动开锁按键*/
static void bsp_unlock_sw_pin_init()
{
    gpio_pin_config_t pin;
    pin.pinDirection = kGPIO_DigitalInput;
    pin.outputLogic = 1U;
    GPIO_PinInit(BOARD_INITPINS_UNLOCK_SW_GPIO,BOARD_INITPINS_UNLOCK_SW_PORT,BOARD_INITPINS_UNLOCK_SW_PIN,&pin);
}

/*手动开锁按键状态*/
uint8_t bsp_unlock_sw_status()
{
    uint8_t pin_level,pin_level_main,status;
    pin_level = GPIO_PinRead(BOARD_INITPINS_UNLOCK_SW_GPIO,BOARD_INITPINS_UNLOCK_SW_PORT,BOARD_INITPINS_UNLOCK_SW_PIN);
    pin_level_main = GPIO_PinRead(BOARD_INITPINS_UNLOCK_SW_MAIN_GPIO,BOARD_INITPINS_UNLOCK_SW_MAIN_PORT,BOARD_INITPINS_UNLOCK_SW_MAIN_PIN);
    /*使用2个按键，只要有一个按键按下就认为是有效按压*/
    if (pin_level == BSP_UNLOCK_SW_STATUS_PRESS_LEVEL || pin_level_main == BSP_UNLOCK_SW_STATUS_PRESS_LEVEL) {
        status = BSP_UNLOCK_SW_STATUS_PRESS;
    } else {
        status = BSP_UNLOCK_SW_STATUS_RELEASE;
    }
    return status;
}

/*锁舌传感器*/
static void bsp_lock_sensor_pin_init()
{
    gpio_pin_config_t pin;
    pin.pinDirection = kGPIO_DigitalInput;
    pin.outputLogic = 1;
    GPIO_PinInit(BOARD_INITPINS_LOCK_SENSOR_GPIO,BOARD_INITPINS_LOCK_SENSOR_PORT,BOARD_INITPINS_LOCK_SENSOR_PIN,&pin);
}

/*锁孔内传感器*/
static void bsp_hole_sensor_pin_init()
{
    gpio_pin_config_t pin;
    pin.pinDirection = kGPIO_DigitalInput;
    pin.outputLogic = 1;
    GPIO_PinInit(BOARD_INITPINS_HOLE_SENSOR_GPIO,BOARD_INITPINS_HOLE_SENSOR_PORT,BOARD_INITPINS_HOLE_SENSOR_PIN,&pin);
}

/*门磁传感器*/
static void bsp_door_sensor_pin_init(void)
{
    gpio_pin_config_t pin;
    pin.pinDirection = kGPIO_DigitalInput;
    pin.outputLogic = 1;
    GPIO_PinInit(BOARD_INITPINS_DOOR_SENSOR_GPIO,BOARD_INITPINS_DOOR_SENSOR_PORT,BOARD_INITPINS_DOOR_SENSOR_PIN,&pin);
}

/*锁舌传感器相关*/
uint8_t bsp_lock_sensor_status()
{
    uint8_t pin_level,status;
    pin_level = GPIO_PinRead(BOARD_INITPINS_LOCK_SENSOR_GPIO,BOARD_INITPINS_LOCK_SENSOR_PORT,BOARD_INITPINS_LOCK_SENSOR_PIN);
    if (pin_level == BSP_LOCK_UNLOCKED_LEVEL) {
        status = BSP_LOCK_STATUS_UNLOCKED;
    } else {
        status = BSP_LOCK_STATUS_LOCKED;
    }
    return status;
}

/*锁孔传感器相关*/
uint8_t bsp_hole_sensor_status()
{
    uint8_t pin_level,status;
    pin_level = GPIO_PinRead(BOARD_INITPINS_HOLE_SENSOR_GPIO,BOARD_INITPINS_HOLE_SENSOR_PORT,BOARD_INITPINS_HOLE_SENSOR_PIN);
    if (pin_level == BSP_HOLE_OPEN_LEVEL) {
        status = BSP_HOLE_STATUS_OPEN;
    } else {
        status = BSP_HOLE_STATUS_CLOSE;
    }
    return status;
}
/*门磁传感器*/
uint8_t bsp_door_sensor_status()
{
    uint8_t pin_level,status;
    pin_level = GPIO_PinRead(BOARD_INITPINS_DOOR_SENSOR_GPIO,BOARD_INITPINS_DOOR_SENSOR_PORT,BOARD_INITPINS_DOOR_SENSOR_PIN);
    if (pin_level == BSP_DOOR_OPEN_LEVEL) {
        status = BSP_DOOR_STATUS_OPEN;
    } else {
        status = BSP_DOOR_STATUS_CLOSE;
    }
    return status;
}

/*板级初始化*/
int bsp_board_init(void)
{
    BOARD_InitBootPins();
    BOARD_BootClockPLL180M();
    
    bsp_compressor_ctrl_pin_init();
    bsp_unlock_sw_pin_init();
    bsp_lock_ctrl_pin_init();
    bsp_lock_sensor_pin_init();
    bsp_hole_sensor_pin_init();
    bsp_door_sensor_pin_init();
    
    return 0;
}

