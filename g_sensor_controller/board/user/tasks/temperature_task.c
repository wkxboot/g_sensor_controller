#include "cmsis_os.h"
#include "tasks_init.h"
#include "stdio.h"
#include "stdbool.h"
#include "adc_task.h"
#include "compressor_task.h"
#include "temperature_task.h"
#include "log.h"

/*任务句柄*/
osThreadId   temperature_task_hdl;
/*消息句柄*/
osMessageQId temperature_task_msg_q_id;


/**/
static int16_t const t_r_map[][2]={
  {-22,21180},{-21,20010},{-20,18900},{-19,17870},{-18,16900},{-17,15980},{-16,15120},{-15,14310},{-14,13550},{-13,12830},
  {-12,12160},{-11,11520},{-10,10920},{-9,10350} ,{-8,9820}  ,{-7,9316}  ,{-6,8841}  ,{-5,8392}  ,{-4,7968}  ,{-3 ,7568},
  {-2 ,7190} ,{-1,6833}  ,{0,6495}   ,{1,6175 }  ,{2,5873 }  ,{3,5587 }  ,{4,5315}   ,{5,5060}   ,{6,4818}   ,{7,4589}  ,
  {8,4372}   ,{9,4167}   ,{10,3972}  ,{11,3788}  ,{12,3613}  ,{13,3447}  ,{14,3290}  ,{15,3141}  ,{16,2999}  ,{17,2865} ,
  {18,2737}  ,{19,2616}  ,{20,2501}  ,{21,2391}  ,{22,2287}  ,{23,2188}  ,{24,2094}  ,{25,2005}  ,{26,1919}  ,{27,1838} ,
  {28,1761}  ,{29,1687}  ,{30,1617}  ,{31,1550}  ,{32,1486}  ,{33,1426}  ,{34,1368}  ,{35,1312}  ,{36,1259}  ,{37,1209} ,
  {38,1161}  ,{39,1115}  ,{40,1071}  ,{41,1029}  ,{42,989}   ,{43,951}   ,{44,914}   ,{45,879}   ,{46,845}   ,{47,813}  ,
  {48,783}   ,{49,753}   ,{50,725}   ,{51,698}   ,{52,672}   ,{53,647}   ,{54,624}   ,{55,601}   ,{56,579}   ,{57,559}  ,
  {58,539}   ,{59,520}   ,{60,502}   ,{61,484}   ,{62,467}   ,{63,451}   ,{64,435}   ,{65,421}   ,{66,406}   ,{67,392}
};

#define  TR_MAP_IDX_MIN        2
#define  TR_MAP_IDX_MAX        82

typedef struct
{
    float value_float;
    int16_t value_int;
    int8_t dir;
    uint8_t err_cnt;
    bool err;
    bool change;
}temperature_t;

/*温度对象实体*/
static temperature_t   temperature;


 /*
 * @brief 获取温度传感器电阻值
 * @param adc adc数值
 * @return 电阻值
 * @note
 */             
static uint32_t get_r(const uint16_t adc)
{
    float t_sensor_r;
    t_sensor_r = (TEMPERATURE_SENSOR_SUPPLY_VOLTAGE * TEMPERATURE_SENSOR_ADC_VALUE_MAX * TEMPERATURE_SENSOR_BYPASS_RES_VALUE)/(adc * TEMPERATURE_SENSOR_REFERENCE_VOLTAGE)-TEMPERATURE_SENSOR_BYPASS_RES_VALUE;
    return (uint32_t)t_sensor_r;
}

 /*
 * @brief 获取浮点温度值
 * @param r 温度传感器电阻值
 * @param idx 阻温映射表下标号
 * @return 浮点温度值
 * @note
 */ 
static float calcaulate_float_t(uint32_t r,uint8_t idx)
{
    uint32_t r1,r2;

    float t;

    r1 = t_r_map[idx][1];
    r2 = t_r_map[idx + 1][1];

    t = t_r_map[idx][0] + (r1 - r) * 1.0 /(r1 - r2) + TEMPERATURE_COMPENSATION_VALUE;
  
    log_debug("temperature: %.2f C.\r\n",t);
   
    return t;  
}

/*
* @brief 获取四舍五入整数温度值
* @param t_float 浮点温度值
* @return 整形四舍五入温度值
* @note
*/
static int16_t calculate_approximate_t(float t_float)
{
    int16_t t = (int16_t)t_float;

    t_float -= t;

    if(t_float >= 0.5 ) {
        t += 1; 
    } else if (t_float <= -0.5){
        t -= 1;
    }
  
    return t;  
}

/*
* @brief 获取实际浮点
* @param adc adc数值
* @return 整形四舍五入温度值
* @note
*/
float get_float_temperature_by_adc(uint16_t adc)
{
    uint32_t r; 
    uint8_t mid;
    int low = TR_MAP_IDX_MIN;  
    int high =TR_MAP_IDX_MAX; 
 
    if (adc >= ADC_ERR_MAX ||
        adc <= ADC_ERR_MIN ){
        log_error("传感器短路或者开路错误.\r\n");
        return TEMPERATURE_ERR_VALUE;
    }

    r = get_r(adc);

    if (r <= t_r_map[TR_MAP_IDX_MAX][1]){
        log_error("NTC 小于最高温度阻值！r=%d\r\n",r); 
        return TEMPERATURE_ERR_VALUE;
    }else if (r >= t_r_map[TR_MAP_IDX_MIN][1]){
        log_error("NTC 大于最低温度阻值！r=%d\r\n",r); 
        return TEMPERATURE_ERR_VALUE;
    }

    while (low <= high) {  
        mid = (low + high) / 2;  
        if (r > t_r_map[mid][1]) {
            if (r <= t_r_map[mid-1][1]){
                /*返回指定精度的温度*/
                return calcaulate_float_t(r, mid - 1);
            } else {
                high = mid - 1;  
            }
        } else {
            if (r > t_r_map[mid+1][1]) {
                /*返回带有温度补偿值的温度*/
                return calcaulate_float_t(r, mid);
            } else {
                low = mid + 1;   
            }
        }  
    }

    return TEMPERATURE_ERR_VALUE;
}


/*
* @brief 温度传感器任务
* @param argument 任务参数
* @return 
* @note
*/
void temperature_task(void const *argument)
{

    osEvent  os_event;
    osStatus status;
    temperature_task_message_t req_msg,rsp_msg;
    compressor_task_message_t update_msg;
    uint16_t bypass_r_adc;
    int16_t t_int;
    float t_float;

    temperature.value_int = 0;
    temperature.value_float = 0.0;

    while (1) {
        os_event = osMessageGet(temperature_task_msg_q_id,TEMPERATURE_TASK_MSG_WAIT_TIMEOUT);
        if (os_event.status == osEventMessage){
            req_msg = *(temperature_task_message_t*)os_event.value.v;
 
            /*温度ADC转换完成消息处理*/
            if (req_msg.request.type == TEMPERATURE_TASK_MSG_TYPE_ADC_COMPLETED){
                bypass_r_adc = req_msg.request.adc;  
             
                t_float = get_float_temperature_by_adc(bypass_r_adc);
              
                /*判断是否在报警范围*/ 
                if (t_float == TEMPERATURE_ERR_VALUE || (t_float > TEMPERATURE_ALARM_VALUE_MAX || t_float < TEMPERATURE_ALARM_VALUE_MIN)) {
                    if (temperature.err == false) {  
                        temperature.err_cnt ++; 
                        if (temperature.err_cnt >= TEMPERATURE_ERR_CNT) {
                            temperature.err = true;
                            temperature.change = true;
                        }
                    }
                } else {  
                    t_int = calculate_approximate_t(t_float);
                    /*温度由错误变为正常，需要发送温度变化消息*/
                    if ( temperature.err == true) {
                        temperature.err = false;
                        temperature.err_cnt = 0;
                        temperature.change = true;
                    }
                    /*温度精度*/
                    if (t_float - temperature.value_float >= TEMPERATURE_ACCURATE) {
                        temperature.dir += 1;    
                    }else if(t_float - temperature.value_float <= -TEMPERATURE_ACCURATE) {
                        temperature.dir -= 1;      
                    } else {
                        temperature.dir = 0; 
                    }
                    /*温度正常变化 当满足条件时 接受数据变化*/
                    if (temperature.dir >= TEMPERATURE_TASK_TEMPERATURE_CHANGE_CNT ||
                        temperature.dir <= -TEMPERATURE_TASK_TEMPERATURE_CHANGE_CNT){
                        temperature.value_int = t_int;
                        temperature.value_float = t_float;
                        temperature.change = true;
                    }
                }
            }
    
            if (temperature.change == true) {
         
                if (temperature.err == true){
                    /*压缩机温度错误消息*/
                    update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_ERR;
                    log_error("temperature err.\r\n");
                }else{
                    /*压缩机温度更新消息*/
                    update_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_UPDATE;
                    update_msg.request.temperature_int = temperature.value_int;
                    update_msg.request.temperature_float = temperature.value_float;
                    log_info("teperature change to:%.2f C.\r\n",temperature.value_float);
                }
                temperature.change = false;       
                temperature.dir = 0;    
                status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&update_msg,TEMPERATURE_TASK_PUT_MSG_TIMEOUT);
                if (status !=osOK) {
                    log_error("put compressor t msg error:%d\r\n",status); 
                } 
            }
        }

        /*温度查询消息处理*/
        if (req_msg.request.type == TEMPERATURE_TASK_MSG_TYPE_TEMPERATURE){
            rsp_msg.response.type = TEMPERATURE_TASK_MSG_TYPE_RSP_TEMPERATURE;
            if (temperature.err == true) {
                rsp_msg.response.err = true;
            } else {
                rsp_msg.response.err = false;
                rsp_msg.response.temperature_int = temperature.value_int;
                rsp_msg.response.temperature_float = temperature.value_float;
            }
            status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t)&rsp_msg,TEMPERATURE_TASK_PUT_MSG_TIMEOUT);
           if (status !=osOK) {
               log_error("put temperature msg error:%d\r\n",status); 
           }          
        }

    }
}