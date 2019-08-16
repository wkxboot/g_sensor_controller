#ifndef __PROCESS_BAR_H__
#define __PROCESS_BAR_H__
#include "stdio.h"
#include "stdint.h"
#include <stdlib.h>
#include <string.h>

#define PROGRESS_BAR_NUM_STYLE  0
#define PROGRESS_BAR_CHR_STYLE  1


typedef struct {
    char tip_char;   /*tip char*/
    char *tip_title; /*tip string*/
    uint8_t style;   /*progress style*/
    float max_value; /*maximum value*/
    char *buffer;
    uint16_t buffer_size;
} progress_bar_t;

/*
* @brief 进度条初始化
* @param progress_bar 进度条指针
* @param tip_title 进度条名称
* @param tip_char 进度条字符样式
* @param buffer 进度条样式缓存
* @param buffer_size 进度条样式缓存大小
* @param max_value 进度条表示的最大值
* @param style 进度条类型
* @return 无
* @note
*/
void progress_bar_init(progress_bar_t *progress_bar,char *tip_title,char tip_char,char *buffer,uint16_t buffer_size,float max_value,uint8_t style);

/*
* @brief 进度条显示
* @param progress_bar 进度条指针
* @param value 进度条当前值
* @return 无
* @note
*/
void progress_bar_show(progress_bar_t *progress_bar, float value);


#endif    /*ifndef*/

