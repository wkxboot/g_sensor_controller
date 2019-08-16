/**
 * linux terminal progress bar (no thread safe).
 *     @package progress.c
 *
 * @author chenxin <chenxin619315@gmail.com>
 */
#include "progress_bar.h"
#include "log.h"


#if  PROCESS_BAR_DEBUG  >  0
#define  PROCESS_BAR_ASSERT(x)  { if ((x) == 0) {while (1);}}
#else
#define  PROCESS_BAR_ASSERT(x)
#endif

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
void progress_bar_init(progress_bar_t *progress_bar,char *tip_title,char tip_char,char *buffer,uint16_t buffer_size,float max_value,uint8_t style)
{
    PROCESS_BAR_ASSERT(progress_bar);
    PROCESS_BAR_ASSERT(tip_title);
    PROCESS_BAR_ASSERT(tip_char);
    PROCESS_BAR_ASSERT(max != 0);

    progress_bar->tip_char = tip_char;
    progress_bar->tip_title = tip_title;
    progress_bar->style = style;
    progress_bar->max_value = max_value;
    progress_bar->buffer =buffer;
    progress_bar->buffer_size = buffer_size;
    
    memset(progress_bar->buffer, 0x00, buffer_size);
}

/*
* @brief 进度条显示
* @param progress_bar 进度条指针
* @param value 进度条当前值
* @return 无
* @note
*/
void progress_bar_show(progress_bar_t *progress_bar, float value)
{
    uint8_t num_progress;
    uint16_t char_progress;   
    float percent;

    if (value > progress_bar->max_value) {
        value = progress_bar->max_value;
    }
    
    percent = value / progress_bar->max_value;
    
    num_progress = (uint8_t)(percent * 100.0);
    char_progress = (uint16_t)(percent * progress_bar->buffer_size);
    
    if (progress_bar->style == PROGRESS_BAR_NUM_STYLE) {
        log_info("\033[?25l%s%d%%\033[?25h\r",
        progress_bar->tip_title,num_progress);
    }

    if (progress_bar->style == PROGRESS_BAR_CHR_STYLE) {
        memset(progress_bar->buffer,progress_bar->tip_char,char_progress);
        memset(progress_bar->buffer + char_progress,0x20,progress_bar->buffer_size - char_progress);
        progress_bar->buffer[progress_bar->buffer_size] = 0;
        log_info("\033[?25l%s[%s] %d%%\033[?25h\r",progress_bar->tip_title,progress_bar->buffer,num_progress);
    }

}


