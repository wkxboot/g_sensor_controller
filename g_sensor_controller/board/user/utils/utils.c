#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"
#include "utils.h"
#include "cmsis_os.h"


/*字节转换成HEX字符串*/
 void bytes_to_hex_str(const char *src,char *dest,uint16_t src_len)
 {
    char temp;
    char hex_digital[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    for (uint16_t i = 0; i < src_len; i++){  
        temp = src[i];  
        dest[2 * i] = hex_digital[(temp >> 4) & 0x0f ];  
        dest[2 * i + 1] = hex_digital[temp & 0x0f];  
    }
    dest[src_len * 2] = '\0';
}


/* 函数：utils_timer_init
*  功能：自定义定时器初始化
*  参数：timer 定时器指针
*  参数：timeout 超时时间
*  参数：up 定时器方向-向上计数-向下计数
*  返回: 0：成功 其他：失败
*/ 
int utils_timer_init(utils_timer_t *timer,uint32_t timeout,bool up)
{
    if(timer == NULL){
        return -1;
    }

    timer->up = up;
    timer->start = osKernelSysTick();  
    timer->value = timeout;  

    return 0;
}

/* 函数：utils_timer_value
*  功能：定时器现在的值
*  返回：>=0：现在时间值 其他：失败
*/ 
int utils_timer_value(utils_timer_t *timer)
{
    uint32_t time_elapse;

    if (timer == NULL){
        return -1;
    }  
    time_elapse = osKernelSysTick() - timer->start; 

    if (timer->up == true) {
        return  timer->value > time_elapse ? time_elapse : timer->value;
    }

    return  timer->value > time_elapse ? timer->value - time_elapse : 0; 
}

/* 函数：utils_get_str_addr_by_num
*  功能：获取字符串中第num个str的地址
*  参数：buffer 字符串地址
*  参数：str    查找的字符串
*  参数：num    第num个str
*  参数：addr   第num个str的地址
*  返回：0：成功 其他：失败
*/ 
int utils_get_str_addr_by_num(char *buffer,const char *str,const uint8_t num,char **addr)
{
    uint8_t cnt = 0;
    uint16_t str_size;
 
    char *search,*temp;
 
    temp = buffer;
    str_size = strlen(str);
 
    while (cnt < num ){
        search = strstr(temp,str);
        if (search == NULL) {
            return -1;
    }
    temp = search + str_size;
    cnt ++;
    }
    *addr = search;
 
    return 0; 
}

/* 函数：utils_get_str_value_by_num
*  功能：获取字符串中第num和第num+1个str之间的字符串
*  参数：buffer 字符串地址
*  参数：dst    存储地址
*  参数：str    字符串
*  参数：num    第num个字符串
*  返回：0：成功 其他：失败
*/
int utils_get_str_value_by_num(char *buffer,char *dst,const char *str,uint8_t num)
{
    int rc;
    uint16_t str_size,str_start_size;
 

    char *addr_start,*addr_end;
 
    rc = utils_get_str_addr_by_num(buffer,str,num,&addr_start);
    if (rc != 0) {
        return -1;
    }
    /*先查找原来的字符串，没有就查询结束符*/
    rc = utils_get_str_addr_by_num(buffer,str,num + 1,&addr_end);
    if (rc != 0) {
        rc = utils_get_str_addr_by_num(addr_start,"\r\n",1,&addr_end);
        if (rc != 0) {
            return -1;
        }
    }
 
    str_start_size = strlen(str);
    if (addr_end < addr_start + str_start_size) {
        return -1;
    }
 
    str_size = addr_end - addr_start - str_start_size;
 
    memcpy(dst,addr_start + str_start_size,str_size);
    dst[str_size] = '\0';
 
    return 0; 
}