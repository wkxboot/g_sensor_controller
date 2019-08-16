#ifndef  __XSTRING_H__
#define  __XSTRING_H__
 
#ifdef  __cplusplus
#define  XSTRING_BEGIN extern "C" {
#define  XSTRING_END   }
#else
#define  XSTRING_BEGIN
#define  XSTRING_END
#endif
 
XSTRING_BEGIN
#include "stdint.h"
#include "stdio.h"

#ifndef   IS_POWER_OF_TWO
#define   IS_POWER_OF_TWO(A)   (((A) != 0) && ((((A) - 1) & (A)) == 0))
#endif

#ifndef   MIN
#define   MIN(A,B)             ((A) > (B) ? (B) :(A))
#endif

#ifndef   MAX
#define   MAX(A,B)             ((A) > (B) ? (A) :(B))
#endif


/**
* @brief 转成HEX字符串
* @param
* @param
* @return 
* @note
*/
int xstring_hex_to_string(char *dst,const char *src,uint16_t src_len);

/**
* @brief 字符串转浮点数
* @param s字符串
* @param 
* @return 浮点数
* @note
*/
double xstring_atof(char *s);

XSTRING_END

   
#endif