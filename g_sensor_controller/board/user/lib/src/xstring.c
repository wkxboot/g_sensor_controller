#include "xstring.h"

/**
* @brief 转成HEX字符串
* @param
* @param
* @return 
* @note
*/
int xstring_hex_to_string(char *dst,const char *src,uint16_t src_len)
 {
    char temp;

    if (src == NULL || dst == NULL) {
        return -1;
    }

    char hex_digital[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    for (uint16_t i = 0; i < src_len; i++){  
        temp = src[i];  
        dst[2 * i] = hex_digital[(temp >> 4) & 0x0f ];  
        dst[2 * i + 1] = hex_digital[temp & 0x0f];  
    }
    dst[src_len * 2] = '\0';

    return 0;
}

/**
* @brief 判断是否是数字字符
* @param s字符串
* @param 
* @return 浮点数
* @note
*/
static uint8_t is_digit(char c)
{
    if (c >= '0' && c <= '9') {
        return 1;
    }

    return 0;
}

/**
* @brief 字符串转浮点数
* @param s字符串
* @param 
* @return 浮点数
* @note
*/
double xstring_atof(char *s)
{ 
    double tf;
    int nc,i,neg;

    tf = 0.0;
    if (s[0] == '-') { 
        neg = 1;
    } else { 
        neg = 0;
    }

    /* search for decimal point */
    for (nc=neg;( is_digit(s[nc]) && (s[nc] != '.') && (s[nc] != '\0')); nc++) ;
    for (i=neg; ( (is_digit(s[i]) || (s[i] == '.')) && (s[i] != '\0') ) ;i++) {   
    switch((nc-i))
    {
      case 0:break; /* we're at the decimal point */
      case 1:tf += (s[i] - '0');  break; /* one's place */
      case 2:tf += (s[i] - '0') * 10; break;
      case 3:tf += (s[i] - '0') * 100; break;
      case 4:tf += (s[i] - '0') * 1000; break;
      case 5:tf += (s[i] - '0') * 10000; break;
      case 6:tf += (s[i] - '0') * 100000; break;
      case 7:tf += (s[i] - '0') * 1000000; break;
      case 8:tf += (s[i] - '0') * 10000000; break;
      case 9:tf += (s[i] - '0') * 100000000.0; break;
      case 10:tf += (s[i] - '0') * 1000000000.0; break;
      case 11:tf += (s[i] - '0') * 10000000000.0;  break; 
      case 12:tf += (s[i] - '0') * 100000000000.0; break;
      case 13:tf += (s[i] - '0') * 1000000000000.0; break;
      case 14:tf += (s[i] - '0') * 10000000000000.0; break;
      case 15:tf += (s[i] - '0') * 100000000000000.0; break;
      case 16:tf += (s[i] - '0') * 1000000000000000.0; break;
      case 17:tf += (s[i] - '0') * 10000000000000000.0; break;
      case 18:tf += (s[i] - '0') * 100000000000000000.0; break;
      case 19:tf += (s[i] - '0') * 1000000000000000000.0; break;
      case -1:tf += (s[i] - '0') * 0.1; break;
      case -2:tf += (s[i] - '0') * 0.01; break;
      case -3:tf += (s[i] - '0') * 0.001; break;
      case -4:tf += (s[i] - '0') * 0.0001; break;
      case -5:tf += (s[i] - '0') * 0.00001; break;
      case -6:tf += (s[i] - '0') * 0.000001; break;
      case -7:tf += (s[i] - '0') * 0.0000001; break;
      case -8:tf += (s[i] - '0') * 0.00000001; break;
      case -9:tf += (s[i] - '0') * 0.000000001; break;
      case -10:tf += (s[i] - '0') * 0.0000000001; break;
      case -11:tf += (s[i] - '0') * 0.00000000001; break;
      case -12:tf += (s[i] - '0') * 0.000000000001; break;
      case -13:tf += (s[i] - '0') * 0.0000000000001; break;
      case -14:tf += (s[i] - '0') * 0.00000000000001; break;
      case -15:tf += (s[i] - '0') * 0.000000000000001; break;
      case -16:tf += (s[i] - '0') * 0.0000000000000001; break;
      case -17:tf += (s[i] - '0') * 0.00000000000000001; break;
      case -18:tf += (s[i] - '0') * 0.000000000000000001; break;
      case -19:tf += (s[i] - '0') * 0.0000000000000000001; break;
      default : break;
    }
  }

    if (neg) {
        tf *= -1.0;
    }

    return(tf);
}
