#ifndef   __LOG_H__
#define   __LOG_H__
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"
#include "string.h"

#ifdef __cplusplus
    extern "C" {
#endif	
  
#define  LOG_LEVEL_OFF             0U
#define  LOG_LEVEL_ERROR           1U
#define  LOG_LEVEL_WARNING         2U
#define  LOG_LEVEL_INFO            3U
#define  LOG_LEVEL_DEBUG           4U
#define  LOG_LEVEL_ARRAY           5U
#define  LOG_LEVEL_LOWEST          5U

#define  LOG_COLOR_BLACK           "[2;30m"
#define  LOG_COLOR_RED             "[2;31m"
#define  LOG_COLOR_GREEN           "[2;32m"
#define  LOG_COLOR_YELLOW          "[2;33m"
#define  LOG_COLOR_BLUE            "[2;34m"
#define  LOG_COLOR_MAGENTA         "[2;35m"
#define  LOG_COLOR_CYAN            "[2;36m"
#define  LOG_COLOR_WHITE           "[2;37m"


/******************************************************************************/
/*    配置开始                                                                */
/******************************************************************************/
#define  LOG_PRINTF_BUFFER_SIZE    1560
#define  LOG_LEVEL_GLOBLE_DEFAULT  LOG_LEVEL_DEBUG 
#define  LOG_USE_RTT               1
#define  LOG_USE_SERIAL            0
#define  LOG_USE_COLORS            1
#define  LOG_USE_TIMESTAMP         1   

#define  LOG_ERROR_COLOR           LOG_COLOR_RED
#define  LOG_WARNING_COLOR         LOG_COLOR_MAGENTA
#define  LOG_INFO_COLOR            LOG_COLOR_GREEN
#define  LOG_DEBUG_COLOR           LOG_COLOR_YELLOW
#define  LOG_ARRAY_COLOR           LOG_COLOR_CYAN

/******************************************************************************/
/*    配置结束                                                                */
/******************************************************************************/

#if  LOG_USE_COLORS ==  0

#undef LOG_USE_COLORS
#undef LOG_WARNING_COLOR
#undef LOG_INFO_COLOR
#undef LOG_DEBUG_COLOR 
#undef LOG_ARRAY_COLOR

#define LOG_USE_COLORS
#define LOG_WARNING_COLOR
#define LOG_INFO_COLOR
#define LOG_DEBUG_COLOR 
#define LOG_ARRAY_COLOR

#endif 

#if     LOG_USE_TIMESTAMP  >   0
#define LOG_TIME_VALUE         log_time()
#define LOG_TIME_FORMAT        "[%8d]"

#else
#define LOG_TIME_VALUE         0
#define LOG_TIME_FORMAT        "[%1d]"
#endif

#define  LOG_FILE_NAME_FORMAT  " %s | "
#define  LOG_LINE_NUM_FORMAT   "%d "


#define LOG_ERROR_PREFIX_FORMAT       "\r\n"LOG_ERROR_COLOR   LOG_TIME_FORMAT   "[error]"   LOG_FILE_NAME_FORMAT LOG_LINE_NUM_FORMAT "\r\n"
#define LOG_WARNING_PREFIX_FORMAT     "\r\n"LOG_WARNING_COLOR LOG_TIME_FORMAT   "[warning]" LOG_FILE_NAME_FORMAT LOG_LINE_NUM_FORMAT "\r\n"
#define LOG_INFO_PREFIX_FORMAT        "\r\n"LOG_INFO_COLOR    LOG_TIME_FORMAT   "[info]"    LOG_FILE_NAME_FORMAT LOG_LINE_NUM_FORMAT "\r\n"
#define LOG_DEBUG_PREFIX_FORMAT       "\r\n"LOG_DEBUG_COLOR   LOG_TIME_FORMAT   "[debug]"   LOG_FILE_NAME_FORMAT LOG_LINE_NUM_FORMAT "\r\n"
#define LOG_ARRAY_PREFIX_FORMAT       "\r\n"LOG_ARRAY_COLOR   LOG_TIME_FORMAT   "[array]"   LOG_FILE_NAME_FORMAT LOG_LINE_NUM_FORMAT "\r\n"

#define LOG_PREFIX_VALUE              LOG_TIME_VALUE,__FILE__,__LINE__ 


/*
* @brief 终端日志初始化
* @param 无
* @return 无
* @note
*/
void log_init(void);


/*
* @brief 日志时间
* @param 无
* @return 
* @note 日志时间
*/
uint32_t log_time(void);


/*
* @brief 终端读取输入
* @param dst 读取数据存储的目的地址
* @param size 期望读取的数量
* @return 实际读取的数量
* @note
*/

uint32_t log_read(char *dst,uint32_t size);

/*
* @brief 设置日志全局输出等级
* @param lelvel 日志等级
* @return = 0 成功
* @return < 0 失败
* @note level >= LOG_LEVEL_OFF && level <=LOG_LEVEL_ARRAY
*/
int log_set_level(uint8_t level);

/*
* @brief 终端日志输出
* @param level 输出等级
* @param format 格式化字符串
* @param ... 可变参数
* @return 实际写入的数量
* @note
*/
int log_printf(uint8_t level,const char *format,...);

/*
* @brief 日志array输出
* @param format格式化字符串
* @param ...可变参数
* @return 无
* @note 
*/
#define  log_array(format,arg...)                                                           \
{                                                                                           \
   log_printf(LOG_LEVEL_ARRAY,LOG_ARRAY_PREFIX_FORMAT format,LOG_PREFIX_VALUE,##arg);     \
}

/*
* @brief 日志debug输出
* @param format格式化字符串
* @param ...可变参数
* @return 无
* @note 
*/
#define  log_debug(format,arg...)                                                           \
{                                                                                           \
   log_printf(LOG_LEVEL_DEBUG,LOG_DEBUG_PREFIX_FORMAT format,LOG_PREFIX_VALUE,##arg);     \
}

/*
* @brief 日志info输出
* @param format格式化字符串
* @param ...可变参数
* @return 无
* @note 
*/
#define  log_info(format,arg...)                                                            \
{                                                                                           \
   log_printf(LOG_LEVEL_INFO,LOG_INFO_PREFIX_FORMAT format,LOG_PREFIX_VALUE,##arg);       \
}

/*
* @brief 日志warning输出
* @param format格式化字符串
* @param ...可变参数
* @return 无
* @note 
*/
#define  log_warning(format,arg...)                                                        \
{                                                                                          \
   log_printf(LOG_LEVEL_WARNING,LOG_WARNING_PREFIX_FORMAT format,LOG_PREFIX_VALUE,##arg);\
}

/*
* @brief 日志error输出
* @param format格式化字符串
* @param ...可变参数
* @return 无
* @note   
*/
#define  log_error(format,arg...)                                                           \
{                                                                                           \
   log_printf(LOG_LEVEL_ERROR,LOG_ERROR_PREFIX_FORMAT format,LOG_PREFIX_VALUE,##arg);     \
}

/*
* @brief 日志断言
* @param 无
* @return 
* @note 
*/ 
void log_assert_handler(int line,char *file_name);


/*
* @brief 终端日志断言
* @param expr 断言变量
* @return 无
* @note
*/
#define log_assert(expr)                                                  \
{                                                                         \
    if ((void *)(expr) == 0) {                                            \
        log_assert_handler(__LINE__,__FILE__);	                          \
    }                                                                     \
}




#ifdef __cplusplus
    }
#endif	

#endif