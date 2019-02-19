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
#include "pin_mux.h"

/*数据灯*/
static void bsp_data_stream_led_pin_init()
{
  gpio_pin_config_t pin;
  pin.pinDirection = kGPIO_DigitalOutput;
  pin.outputLogic = 0;
  GPIO_PortInit(DATA_STREAM_LED_GPIO, DATA_STREAM_LED_PORT);
  GPIO_PinInit(DATA_STREAM_LED_GPIO,DATA_STREAM_LED_PORT,DATA_STREAM_LED_PIN,&pin);
}
/*系统运行灯*/
static void bsp_sys_led_pin_init()
{
  gpio_pin_config_t pin;
  pin.pinDirection = kGPIO_DigitalOutput;
  pin.outputLogic = 1;
  GPIO_PortInit(SYS_LED_GPIO, SYS_LED_PORT);
  GPIO_PinInit(SYS_LED_GPIO,SYS_LED_PORT,SYS_LED_PIN,&pin);
}
/*锁开关控制端*/
static void bsp_lock_ctrl_pin_init()
{
  gpio_pin_config_t pin;
  pin.pinDirection = kGPIO_DigitalOutput;
  pin.outputLogic = 0;
  GPIO_PortInit(LOCK_CTRL_GPIO, LOCK_CTRL_PORT);
  GPIO_PinInit(LOCK_CTRL_GPIO,LOCK_CTRL_PORT,LOCK_CTRL_PIN,&pin);
}
/*锁舌传感器*/
static void bsp_lock_sensor_pin_init()
{
  gpio_pin_config_t pin;
  pin.pinDirection = kGPIO_DigitalInput;
  pin.outputLogic = 1;
  GPIO_PortInit(LOCK_SENSOR_GPIO, LOCK_SENSOR_PORT);
  GPIO_PinInit(LOCK_SENSOR_GPIO,LOCK_SENSOR_PORT,LOCK_SENSOR_PIN,&pin);
}

/*锁孔内传感器*/
static void bsp_lock_hole_sensor_pin_init()
{
  gpio_pin_config_t pin;
  pin.pinDirection = kGPIO_DigitalInput;
  pin.outputLogic = 1;
  GPIO_PortInit(LOCK_HOLE_SENSOR_GPIO, LOCK_HOLE_SENSOR_PORT);
  GPIO_PinInit(LOCK_HOLE_SENSOR_GPIO,LOCK_HOLE_SENSOR_PORT,LOCK_HOLE_SENSOR_PIN,&pin);
}
/*门磁传感器*/
static void bsp_door_sensor_pin_init()
{
  gpio_pin_config_t pin;
  pin.pinDirection = kGPIO_DigitalInput;
  pin.outputLogic = 1;
  GPIO_PortInit(DOOR_SENSOR_GPIO, DOOR_SENSOR_PORT);
  GPIO_PinInit(DOOR_SENSOR_GPIO,DOOR_SENSOR_PORT,DOOR_SENSOR_PIN,&pin);
}
/*锁相关*/
void bsp_lock_ctrl_open()
{
 GPIO_PortSet(LOCK_CTRL_GPIO,LOCK_CTRL_PORT,(1U<<LOCK_CTRL_PIN));
}

void bsp_lock_ctrl_close()
{
 GPIO_PortClear( LOCK_CTRL_GPIO,LOCK_CTRL_PORT,(1U<<LOCK_CTRL_PIN));
}
/*锁舌传感器相关*/
uint8_t bsp_lock_sensor_status()
{
 uint8_t pin_level,status;
 pin_level = GPIO_PinRead(LOCK_SENSOR_GPIO,LOCK_SENSOR_PORT,LOCK_SENSOR_PIN);
 if(pin_level == BSP_LOCK_OPEN_LEVEL){
   status = BSP_LOCK_STATUS_OPEN;
 }else{
   status = BSP_LOCK_STATUS_CLOSE;
 }
 return status;
}

/*锁孔传感器相关*/
uint8_t bsp_lock_hole_sensor_status()
{
 uint8_t pin_level,status;
 pin_level = GPIO_PinRead(LOCK_HOLE_SENSOR_GPIO,LOCK_HOLE_SENSOR_PORT,LOCK_HOLE_SENSOR_PIN);
 if(pin_level == BSP_LOCK_HOLE_OPEN_LEVEL){
   status = BSP_LOCK_HOLE_STATUS_OPEN;
 }else{
   status = BSP_LOCK_HOLE_STATUS_CLOSE;
 }
 return status;
}
/*门磁传感器*/
uint8_t bsp_door_sensor_status()
{
 uint8_t pin_level,status;
 pin_level = GPIO_PinRead(DOOR_SENSOR_GPIO,DOOR_SENSOR_PORT,DOOR_SENSOR_PIN);
 if(pin_level == BSP_DOOR_OPEN_LEVEL){
   status = BSP_DOOR_STATUS_OPEN;
 }else{
   status = BSP_DOOR_STATUS_CLOSE;
 }
 return status;
}

/*数据灯相关*/
void bsp_data_stream_led_toggle()
{
 GPIO_PortToggle(DATA_STREAM_LED_GPIO,DATA_STREAM_LED_PORT,(1<<DATA_STREAM_LED_PIN));
}

void bsp_data_stream_led_on()
{
 GPIO_PortSet(DATA_STREAM_LED_GPIO,DATA_STREAM_LED_PORT,(1U<<DATA_STREAM_LED_PIN));
}

void bsp_data_stream_led_off()
{
 GPIO_PortClear( DATA_STREAM_LED_GPIO,DATA_STREAM_LED_PORT,(1U<<DATA_STREAM_LED_PIN));
}

void bsp_sys_led_toggle()
{
 GPIO_PortToggle(SYS_LED_GPIO,SYS_LED_PORT,(1<<SYS_LED_PIN));
}

/*板级初始化*/
int bsp_board_init()
{
bsp_data_stream_led_pin_init();
bsp_sys_led_pin_init();
bsp_lock_ctrl_pin_init();
bsp_lock_sensor_pin_init();
bsp_lock_hole_sensor_pin_init();
bsp_door_sensor_pin_init();

BOARD_InitBootPins();
BOARD_BootClockPLL180M();

return 0;
}


