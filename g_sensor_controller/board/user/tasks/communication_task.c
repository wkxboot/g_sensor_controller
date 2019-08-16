#include "board.h"
#include "cmsis_os.h"
#include "xstring.h"
#include "xtimer.h"
#include "xuart.h"
#include "nxp_cm4_uart_hal_driver.h"
#include "firmware_version.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "lock_task.h"
#include "temperature_task.h"
#include "compressor_task.h"
#include "communication_task.h"
#include "fymodem.h"
#include "device_env.h"
#include "crc16.h"
#include "md5.h"
#include "log.h"

osThreadId   communication_task_hdl;
osMessageQId communication_task_msg_q_id;
/*通信串口句柄*/
static xuart_handle_t communication_uart_handle;
static uint8_t comm_recv_buffer[COMMUNICATION_TASK_RX_BUFFER_SIZE];
static uint8_t comm_send_buffer[COMMUNICATION_TASK_TX_BUFFER_SIZE];


extern xuart_hal_driver_t xuart_hal_driver;
/*通信任务上文实体*/
static communication_task_contex_t communication_task_contex;

/*通信协议部分*/
#define  CONTROLLER_ADDR                            1
/*ADU*/
#define  ADU_SIZE_MAX                               30
#define  ADU_HEAD_OFFSET                            0
#define  ADU_HEAD_SIZE                              2
#define  ADU_HEAD0_VALUE                            'M'
#define  ADU_HEAD1_VALUE                            'L'
#define  ADU_PDU_SIZE_REGION_OFFSET                 2
#define  ADU_PDU_SIZE_REGION_SIZE                   1
#define  ADU_PDU_OFFSET                             3
#define  ADU_CRC_SIZE                               2
/*PDU CODE*/ 
#define  PDU_ADDR_OFFSET                            0
#define  PDU_ADDR_SIZE                              1
#define  PDU_CODE_OFFSET                            1
#define  PDU_CODE_SIZE                              1
#define  PDU_DATA_OFFSET                            2

#define  CODE_SCALE_ADDR_CONFIGRATION               0
#define  CODE_QUERY_NET_WEIGHT                      1
#define  CODE_REMOVE_TARE_WEIGHT                    2
#define  CODE_CALIBRATION_ZERO                      3
#define  CODE_CALIBRATION_FULL                      4
#define  CODE_FIRMWARE_VERSION                      5
#define  CODE_QUERY_TEMPERATURE                     6
#define  CODE_SET_TEMPERATURE                       7 
/*保留*/
#define  CODE_UNLOCK_LOCK                           8
#define  CODE_LOCK_LOCK                             9
#define  CODE_QUERY_DOOR_STATUS                     10
#define  CODE_QUERY_LOCK_STATUS                     11  
#define  CODE_QUERY_MANUFACTURER_HARDWARE_VER       12 
#define  CODE_NOTIFY_UPDATE                         13
#define  CODE_MAX                                   CODE_NOTIFY_UPDATE

/*PDU DATA*/
#define  DATA_SCALE_ADDR_CONFIGRATION_SIZE          2 
#define  DATA_QUERY_NET_WEIGHT_SIZE                 3 
#define  DATA_REMOVE_TARE_SIZE                      3 
#define  DATA_CALIBRATION_SIZE                      5 
#define  DATA_QUERY_TEMPERATURE_SIZE                2
#define  DATA_SET_TEMPERATURE_SIZE                  3
#define  DATA_QUERY_SOFTWARE_VER_SIZE               2
/*保留*/
#define  DATA_QUERY_DOOR_STATUS_SIZE                2
#define  DATA_LOCK_LOCK_SIZE                        2
#define  DATA_UNLOCK_LOCK_SIZE                      2
#define  DATA_QUERY_LOCK_STATUS_SIZE                2
#define  DATA_QUERY_HARDWARE_VER_SIZE               2
#define  DATA_NOTIFY_UPDATE_SIZE                    22
/*数据值*/
#define  DATA_SCALE_ADDR_OFFSET                     0
#define  DATA_CALIBRATION_WEIGHT_OFFSET             1
#define  DATA_SCALE_CNT_OFFSET                      0
#define  DATA_TEMPERATURE_OFFSET                    0
#define  DATA_FILE_SIZE_OFFSET                      0
#define  DATA_FILE_MD5_OFFSET                       4
#define  DATA_STATUS_OFFSET                         0

/*协议操作值定义*/
#define  DATA_NET_WEIGHT_ERR_VALUE                  0x7FFF
#define  DATA_TEMPERATURE_ERR_VALUE                 0x7F
#define  DATA_RESULT_REMOVE_TARE_SUCCESS            0x00
#define  DATA_RESULT_REMOVE_TARE_FAIL               0x01
#define  DATA_RESULT_CALIBRATION_SUCCESS            0x00
#define  DATA_RESULT_CALIBRATION_FAIL               0x01
#define  DATA_RESULT_SET_TEMPERATURE_SUCCESS        0x00
#define  DATA_RESULT_SET_TEMPERATURE_FAIL           0x01
/*保留*/
#define  DATA_STATUS_DOOR_OPEN                      0x00
#define  DATA_STATUS_DOOR_CLOSE                     0x01
#define  DATA_STATUS_DOOR_ERR                       0xFF
#define  DATA_STATUS_LOCK_UNLOCKED                  0x00
#define  DATA_STATUS_LOCK_LOCKED                    0x01
#define  DATA_STATUS_LOCK_ERR                       0xFF
#define  DATA_RESULT_LOCK_SUCCESS                   0x00
#define  DATA_RESULT_LOCK_FAIL                      0x01
#define  DATA_RESULT_UNLOCK_SUCCESS                 0x00
#define  DATA_RESULT_UNLOCK_FAIL                    0x01
#define  DATA_MANUFACTURER_CHANGHONG_ID             0x0101

/*协议时间*/
#define  ADU_WAIT_TIMEOUT                           osWaitForever
#define  ADU_FRAME_TIMEOUT                          3
#define  ADU_QUERY_WEIGHT_TIMEOUT                   40
#define  ADU_REMOVE_TARE_TIMEOUT                    510
#define  ADU_CALIBRATION_ZERO_TIMEOUT               510
#define  ADU_CALIBRATION_FULL_TIMEOUT               510
#define  ADU_LOCK_RSP_TIMEOUT                       990
#define  ADU_UNLOCK_RSP_TIMEOUT                     990
#define  ADU_QUERY_DOOR_STATUS_TIMEOUT              40
#define  ADU_QUERY_LOCK_STATUS_TIMEOUT              40
#define  ADU_QUERY_TEMPERATURE_TIMEOUT              20
#define  ADU_QUERY_TEMPERATURE_SETTING_TIMEOUT      20
#define  ADU_SEND_TIMEOUT                           5

/*
* @brief 计算发送缓存CRC并填充到发送缓存
* @param adu 回应缓存
* @param size 当前数据缓存长度
* @return 加上crc后的数据长度
* @note
*/

static uint8_t adu_add_crc16(uint8_t *adu,uint8_t size)
{
    uint16_t crc16;
    crc16 = calculate_crc16(adu,size);

    adu[size ++] = crc16 & 0xFF;
    adu[size ++] = (crc16 >> 8) & 0xFF ;
    return size;
}


/*
* @brief 读取电子称地址配置
* @param config 硬件配置指针
* @return 0 成功
* @return -1 失败
* @note
*/
static int communication_read_scale_addr_configration(scale_addr_configration_t *addr)
{
    addr->cnt = SCALE_CNT_MAX;
    for (uint8_t i = 0;i < addr->cnt;i ++) {
        addr->value[i] = 10 * i + 11;
    }
    return 0;
}
/*
* @brief 查找地址在配置表中对应的标号
* @param addr 传感器地址
* @return -1 失败
* @return >=0 对应的标号
* @note
*/
static int find_scale_task_contex_index(const communication_task_contex_t *contex,uint8_t addr)
{   
    /*指定电子秤任务*/
    for (uint8_t i = 0;i < contex->cnt;i ++) {
        if (addr == contex->scale_task_contex[i].internal_addr) {
            return i;
        }
    }

    return -1;
}


/*
* @brief 请求电子秤数量
* @param contex 电子秤上下文
* @return -1 失败
* @return  > 0 配置电子秤数量
* @note
*/
static int query_scale_cnt(communication_task_contex_t *contex,uint8_t *value)
{
    for (uint8_t i = 0;i < contex->cnt;i ++) {
        value[i] = contex->scale_task_contex[i].internal_addr;
    }
    return contex->cnt;
}

/*
* @brief 请求净重值
* @param contex 通信任务任务上下文
* @param addr 电子秤地址
* @param value 净重量值指针
* @return -1 失败
* @return  0 成功
* @note
*/
static int query_net_weight(const communication_task_contex_t *contex,const uint8_t addr,int16_t *value)
{
    int rc;
    uint32_t flags = 0;
    uint8_t cnt;
    osStatus status;
    osEvent os_event;

    scale_task_message_t req_msg[SCALE_CNT_MAX],rsp_msg;
    xtimer_t timer;

    xtimer_init(&timer,0,ADU_QUERY_WEIGHT_TIMEOUT);
    /*全部电子秤任务*/
    if (addr == 0) {      
        /*发送消息*/
        for (uint8_t i = 0;i < contex->cnt;i ++) {
            req_msg[i].request.type = SCALE_TASK_MSG_TYPE_NET_WEIGHT;
            req_msg[i].request.rsp_message_queue_id = contex->net_weight_rsp_msg_q_id; 
            req_msg[i].request.addr = contex->scale_task_contex[i].internal_addr;
            req_msg[i].request.index = i;
            flags |= contex->scale_task_contex[i].flag;
            status = osMessagePut(contex->scale_task_contex[i].msg_q_id,(uint32_t)&req_msg[i],xtimer_value(&timer));
            log_assert_bool_false(status == osOK);
        }
        cnt = contex->cnt;
    } else {/*指定电子秤任务*/
        rc = find_scale_task_contex_index(contex,addr);
        if (rc < 0) {
            log_error("scale addr:%d invlaid.\r\n",addr);
            return -1;
        }    
        req_msg[0].request.type = SCALE_TASK_MSG_TYPE_NET_WEIGHT;
        req_msg[0].request.rsp_message_queue_id = contex->net_weight_rsp_msg_q_id;
        req_msg[0].request.addr = addr;
        req_msg[0].request.index = 0;
        flags |= contex->scale_task_contex[rc].flag;
        status = osMessagePut(contex->scale_task_contex[rc].msg_q_id,(uint32_t)&req_msg[0],xtimer_value(&timer));
        log_assert_bool_false(status == osOK);
        cnt = 1;
    }
    /*等待消息*/
    while (flags != 0 && xtimer_value(&timer) > 0) {
        os_event = osMessageGet(contex->net_weight_rsp_msg_q_id,xtimer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(scale_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != SCALE_TASK_MSG_TYPE_RSP_NET_WEIGHT) {     
                log_error("comm net weight rsp type:%d err.\r\n",rsp_msg.response.type);
                continue;
            }
            value[rsp_msg.response.index] = rsp_msg.response.weight;
            flags ^= rsp_msg.response.flag;
        }
    }
        
    if (flags != 0) {
        log_error("net weight query timeout err.flags:%d.\r\n",flags);
        return -1;
    }

    return cnt;
}


/*
* @brief 去除皮重
* @param contex 电子秤任务上下文
* @param addr 电子秤地址
* @return -1 失败
* @return  0 成功
* @note
*/
static int remove_tare_weight(const communication_task_contex_t *contex,const uint8_t addr)
{
    int rc;
    uint32_t flags = 0;
    osStatus status;
    osEvent os_event;

    scale_task_message_t req_msg[SCALE_CNT_MAX],rsp_msg;
    xtimer_t timer;
    bool success = true;

    xtimer_init(&timer,0,ADU_REMOVE_TARE_TIMEOUT);
    /*全部电子秤任务*/
    if (addr == 0) {      
        /*发送消息*/
        for (uint8_t i = 0;i < contex->cnt;i ++) { 
            req_msg[i].request.type = SCALE_TASK_MSG_TYPE_REMOVE_TARE_WEIGHT;
            req_msg[i].request.rsp_message_queue_id = contex->remove_tare_rsp_msg_q_id;
            req_msg[i].request.addr = contex->scale_task_contex[i].internal_addr;
            req_msg[i].request.index = i;
            flags |= contex->scale_task_contex[i].flag;
            status = osMessagePut(contex->scale_task_contex[i].msg_q_id,(uint32_t)&req_msg[i],xtimer_value(&timer));
            log_assert_bool_false(status == osOK);
        }
    } else {/*指定电子秤任务*/
        rc = find_scale_task_contex_index(contex,addr);
        if (rc < 0) {
            log_error("scale addr:%d invlaid.\r\n",addr);
            return -1;
        }   
        req_msg[0].request.type = SCALE_TASK_MSG_TYPE_REMOVE_TARE_WEIGHT;
        req_msg[0].request.rsp_message_queue_id = contex->remove_tare_rsp_msg_q_id; 
        req_msg[0].request.addr = addr;
        req_msg[0].request.index = 0;
        flags |= contex->scale_task_contex[rc].flag;
        status = osMessagePut(contex->scale_task_contex[rc].msg_q_id,(uint32_t)&req_msg[0],xtimer_value(&timer));
        log_assert_bool_false(status == osOK);
    }
    /*等待消息*/
    while (flags != 0 && xtimer_value(&timer) > 0) {
        os_event = osMessageGet(contex->remove_tare_rsp_msg_q_id,xtimer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(scale_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != SCALE_TASK_MSG_TYPE_RSP_REMOVE_TARE_WEIGHT) {     
                log_error("comm remove tare weight rsp type:%d err.\r\n",rsp_msg.response.type);
                continue;
            }
            if (rsp_msg.response.result == SCALE_TASK_FAIL) {
                success = false;
            }
            flags ^= rsp_msg.response.flag;
        }
    }
        
    if (flags != 0) {
        log_error("comm remove tare weight timeout err.flags:%d.\r\n",flags);
        return -1;
    }

    return success == true ? 0 : -1;
}

/*
* @brief 0点校准
* @param contex 通信任务上下文
* @param addr 电子秤地址
* @param weight 校准重力值
* @return -1 失败
* @return  0 成功
* @note
*/
static int calibration_zero(const communication_task_contex_t *contex,const uint8_t addr,const int16_t weight)
{
    int rc;
    uint32_t flags = 0;
    osStatus status;
    osEvent os_event;

    scale_task_message_t req_msg[SCALE_CNT_MAX],rsp_msg;
    xtimer_t timer;
    bool success = true;

    if (weight != 0) {
        log_error("calibration zero weight:%d != 0 err.\r\n",weight);
        return -1;
    }
    xtimer_init(&timer,0,ADU_CALIBRATION_ZERO_TIMEOUT);

    /*全部电子秤任务*/
    if (addr == 0) {      
        /*发送消息*/
        for (uint8_t i = 0;i < contex->cnt;i ++) { 
            req_msg[i].request.type = SCALE_TASK_MSG_TYPE_CALIBRATION_ZERO_WEIGHT;    
            req_msg[i].request.rsp_message_queue_id = contex->calibration_zero_rsp_msg_q_id;
            req_msg[i].request.weight = weight;
            req_msg[i].request.addr = contex->scale_task_contex[i].internal_addr;
            req_msg[i].request.index = i;
            flags |= contex->scale_task_contex[i].flag;
            status = osMessagePut(contex->scale_task_contex[i].msg_q_id,(uint32_t)&req_msg[i],xtimer_value(&timer));
            log_assert_bool_false(status == osOK);
        }
    } else {/*指定电子秤任务*/
        rc = find_scale_task_contex_index(contex,addr);
        if (rc < 0) {
            log_error("scale addr:%d invlaid.\r\n",addr);
            return -1;
        }    
        req_msg[0].request.type = SCALE_TASK_MSG_TYPE_CALIBRATION_ZERO_WEIGHT;    
        req_msg[0].request.rsp_message_queue_id = contex->calibration_zero_rsp_msg_q_id;
        req_msg[0].request.weight = weight;
        req_msg[0].request.addr = addr;
        req_msg[0].request.index = 0;
        flags |= contex->scale_task_contex[rc].flag;
        status = osMessagePut(contex->scale_task_contex[rc].msg_q_id,(uint32_t)&req_msg[0],xtimer_value(&timer));
        log_assert_bool_false(status == osOK);
    }
    /*等待消息*/
    while (flags != 0 && xtimer_value(&timer) > 0) {
        os_event = osMessageGet(contex->calibration_zero_rsp_msg_q_id,xtimer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(scale_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != SCALE_TASK_MSG_TYPE_RSP_CALIBRATION_ZERO_WEIGHT) {     
                log_error("comm calibration zero weight rsp type:%d err.\r\n",rsp_msg.response.type);
                continue;
            }
            if (rsp_msg.response.result == SCALE_TASK_FAIL) {
                success = false;
            }
            flags ^= rsp_msg.response.flag;
        }
    }
        
    if (flags != 0) {
        log_error("comm calibration zero timeout err.flags:%d.\r\n",flags);
        return -1;
    }

    return success == true ? 0 : -1;
}

/*
* @brief 增益校准
* @param contex 通信任务上下文
* @param addr 电子秤地址
* @param weight 校准重力值
* @return -1 失败
* @return  0 成功
* @note
*/
static int calibration_full(const communication_task_contex_t *contex,const uint8_t addr,const int16_t weight)
{
    int rc;
    uint32_t flags = 0;
    osStatus status;
    osEvent os_event;

    scale_task_message_t req_msg[SCALE_CNT_MAX],rsp_msg;
    xtimer_t timer;
    bool success = true;

    if (weight <= 0) {
        log_error("calibration full weight:%d <= 0 err.\r\n",weight);
        return -1;
    }
    xtimer_init(&timer,0,ADU_CALIBRATION_FULL_TIMEOUT);

    /*全部电子秤任务*/
    if (addr == 0) {      
        /*发送消息*/
        for (uint8_t i = 0;i < contex->cnt;i ++) { 
            req_msg[i].request.type = SCALE_TASK_MSG_TYPE_CALIBRATION_FULL_WEIGHT;    
            req_msg[i].request.rsp_message_queue_id = contex->calibration_full_rsp_msg_q_id;
            req_msg[i].request.weight = weight;
            req_msg[i].request.addr = contex->scale_task_contex[i].internal_addr;
            req_msg[i].request.index = i;
            flags |= contex->scale_task_contex[i].flag;
            status = osMessagePut(contex->scale_task_contex[i].msg_q_id,(uint32_t)&req_msg[i],xtimer_value(&timer));
            log_assert_bool_false(status == osOK);
        }
    } else {/*指定电子秤任务*/
        rc = find_scale_task_contex_index(contex,addr);
        if (rc < 0) {
            log_error("scale addr:%d invlaid.\r\n",addr);
            return -1;
        }  
        req_msg[0].request.type = SCALE_TASK_MSG_TYPE_CALIBRATION_FULL_WEIGHT;    
        req_msg[0].request.rsp_message_queue_id = contex->calibration_full_rsp_msg_q_id;  
        req_msg[0].request.weight = weight;
        req_msg[0].request.addr = addr;
        req_msg[0].request.index = 0;
        flags |= contex->scale_task_contex[rc].flag;
        status = osMessagePut(contex->scale_task_contex[rc].msg_q_id,(uint32_t)&req_msg[0],xtimer_value(&timer));
        log_assert_bool_false(status == osOK);
    }
    /*等待消息*/
    while (flags != 0 && xtimer_value(&timer) > 0) {
        os_event = osMessageGet(contex->calibration_full_rsp_msg_q_id,xtimer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(scale_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != SCALE_TASK_MSG_TYPE_RSP_CALIBRATION_FULL_WEIGHT) {     
                log_error("comm calibration full weight rsp type:%d err.\r\n",rsp_msg.response.type);
                continue;
            }
            if (rsp_msg.response.result == SCALE_TASK_FAIL) {
                success = false;
            }
            flags ^= rsp_msg.response.flag;
        }
    }
        
    if (flags != 0) {
        log_error("comm calibration full timeout err.flags:%d.\r\n",flags);
        return -1;
    }

    return success == true ? 0 : -1;       
}

/*
* @brief 查询门状态
* @param contex 通信任务上下文
* @param door_status 门状态指针
* @return -1 失败
* @return  0 成功
* @note
*/

static int query_door_status(communication_task_contex_t *contex,uint8_t *door_status)
{
    osStatus status;
    osEvent os_event;

    lock_task_message_t req_msg,rsp_msg;
    xtimer_t timer;

    req_msg.request.type = LOCK_TASK_MSG_TYPE_DOOR_STATUS;    
    req_msg.request.rsp_message_queue_id = contex->query_door_status_rsp_msg_q_id;
    xtimer_init(&timer,0,ADU_QUERY_DOOR_STATUS_TIMEOUT);
    
    /*发送消息*/
    status = osMessagePut(lock_task_msg_q_id,(uint32_t)&req_msg,xtimer_value(&timer));
    log_assert_bool_false(status == osOK);

    /*等待消息*/
    while (xtimer_value(&timer) > 0) {
        os_event = osMessageGet(contex->query_door_status_rsp_msg_q_id,xtimer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(lock_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != LOCK_TASK_MSG_TYPE_RSP_DOOR_STATUS) {     
                log_error("comm query door status rsp type:%d err.\r\n",rsp_msg.response.type);
                continue;
            }

            *door_status = rsp_msg.response.status;
            return 0;
        }
    }
        
    log_error("comm query door status timeout err.\r\n");
    return -1;
}

/*
* @brief 查询锁状态
* @param contex 通信任务上下文
* @param lock_status 锁状态指针
* @return -1 失败
* @return  0 成功
* @note
*/
static int query_lock_status(communication_task_contex_t *contex,uint8_t *lock_status)
{
    osStatus status;
    osEvent os_event;

    lock_task_message_t req_msg,rsp_msg;
    xtimer_t timer;

    req_msg.request.type = LOCK_TASK_MSG_TYPE_LOCK_STATUS;    
    req_msg.request.rsp_message_queue_id = contex->query_lock_status_rsp_msg_q_id;
    xtimer_init(&timer,0,ADU_QUERY_LOCK_STATUS_TIMEOUT);
    
    /*发送消息*/
    status = osMessagePut(lock_task_msg_q_id,(uint32_t)&req_msg,xtimer_value(&timer));
    log_assert_bool_false(status == osOK);

    /*等待消息*/
    while (xtimer_value(&timer) > 0) {
        os_event = osMessageGet(contex->query_lock_status_rsp_msg_q_id,xtimer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(lock_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != LOCK_TASK_MSG_TYPE_RSP_LOCK_STATUS) {     
                log_error("comm query lock status rsp type:%d err.\r\n",rsp_msg.response.type);
                continue;
            }

            *lock_status = rsp_msg.response.status;
            return 0;
        }
    }
        
    log_error("comm query door status timeout err.\r\n");
    return -1;
}

/*
* @brief 开锁
* @param contex 通信任务上下文
* @param result 结果指针
* @return -1 失败
* @return  0 成功
* @note
*/

static int unlock_lock(communication_task_contex_t *contex)
{
    osStatus status;
    osEvent os_event;

    lock_task_message_t req_msg,rsp_msg;
    xtimer_t timer;

    req_msg.request.type = LOCK_TASK_MSG_TYPE_UNLOCK_LOCK;    
    req_msg.request.rsp_message_queue_id = contex->unlock_lock_rsp_msg_q_id;
    xtimer_init(&timer,0,ADU_UNLOCK_RSP_TIMEOUT);
    
    /*发送消息*/
    status = osMessagePut(lock_task_msg_q_id,(uint32_t)&req_msg,xtimer_value(&timer));
    log_assert_bool_false(status == osOK);

    /*等待消息*/
    while (xtimer_value(&timer) > 0) {
        os_event = osMessageGet(contex->unlock_lock_rsp_msg_q_id,xtimer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(lock_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != LOCK_TASK_MSG_TYPE_RSP_UNLOCK_LOCK_RESULT) {     
                log_error("comm unlock lock rsp type:%d err.\r\n",rsp_msg.response.type);
                continue;
            }

            return rsp_msg.response.result == LOCK_TASK_SUCCESS ? 0 : -1;          
        }
    }
        
    log_error("comm unlock lock timeout err.\r\n");
    return -1;
}

/*
* @brief 关锁
* @param contex 通信任务上下文
* @return -1 失败
* @return  0 成功
* @note
*/
static int lock_lock(communication_task_contex_t *contex)
{
    osStatus status;
    osEvent os_event;

    lock_task_message_t req_msg,rsp_msg;
    xtimer_t timer;

    req_msg.request.type = LOCK_TASK_MSG_TYPE_LOCK_LOCK;    
    req_msg.request.rsp_message_queue_id = contex->lock_lock_rsp_msg_q_id;
    xtimer_init(&timer,0,ADU_LOCK_RSP_TIMEOUT);
    
    /*发送消息*/
    status = osMessagePut(lock_task_msg_q_id,(uint32_t)&req_msg,xtimer_value(&timer));
    log_assert_bool_false(status == osOK);

    /*等待消息*/
    while (xtimer_value(&timer) > 0) {
        os_event = osMessageGet(contex->lock_lock_rsp_msg_q_id,xtimer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(lock_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != LOCK_TASK_MSG_TYPE_RSP_LOCK_LOCK_RESULT) {     
                log_error("comm lock lock rsp type:%d err.\r\n",rsp_msg.response.type);
                continue;
            }

            return rsp_msg.response.result == LOCK_TASK_SUCCESS ? 0 : -1;
        }
    }
        
    log_error("comm lock lock timeout err.\r\n");
    return -1;
}

/*
* @brief 查询温度值
* @param contex 通信任务上下文
* @param temperature 温度指针
* @return -1 失败
* @return  0 成功
* @note
*/
static int query_temperature(communication_task_contex_t *contex,int8_t *temperature)
{
    osStatus status;
    osEvent os_event;

    temperature_task_message_t req_msg,rsp_msg;
    xtimer_t timer;

    req_msg.request.type = TEMPERATURE_TASK_MSG_TYPE_TEMPERATURE;    
    req_msg.request.rsp_message_queue_id = contex->query_temperature_rsp_msg_q_id;
    xtimer_init(&timer,0,ADU_QUERY_TEMPERATURE_TIMEOUT);
    
    /*发送消息*/
    status = osMessagePut(temperature_task_msg_q_id,(uint32_t)&req_msg,xtimer_value(&timer));
    log_assert_bool_false(status == osOK);

    /*等待消息*/
    while (xtimer_value(&timer) > 0) {
        os_event = osMessageGet(contex->query_temperature_rsp_msg_q_id,xtimer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(temperature_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != TEMPERATURE_TASK_MSG_TYPE_RSP_TEMPERATURE) {     
                log_error("comm query temperature rsp type:%d err.\r\n",rsp_msg.response.type);
                continue;
            }
            if (rsp_msg.response.err == false) {
                *temperature = rsp_msg.response.temperature_int;
            } else {
                *temperature = DATA_TEMPERATURE_ERR_VALUE;
            }
            return 0;
        }
    }
        
    log_error("comm query temperature timeout err.\r\n");
    return -1;
}

/*
* @brief 查询温度设置值
* @param contex 通信任务上下文
* @param setting 温度设置指针
* @return -1 失败
* @return  0 成功
* @note
*/
static int query_temperature_setting(communication_task_contex_t *contex,int8_t *setting)
{
    osStatus status;
    osEvent os_event;

    compressor_task_message_t req_msg,rsp_msg;
    xtimer_t timer;

    req_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_QUERY_TEMPERATURE_SETTING;    
    req_msg.request.rsp_message_queue_id = contex->query_temperature_setting_rsp_msg_q_id;
    xtimer_init(&timer,0,ADU_QUERY_TEMPERATURE_SETTING_TIMEOUT);
    
    /*发送消息*/
    status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_msg,xtimer_value(&timer));
    log_assert_bool_false(status == osOK);

    /*等待消息*/
    while (xtimer_value(&timer) > 0) {
        os_event = osMessageGet(contex->query_temperature_setting_rsp_msg_q_id,xtimer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(compressor_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != COMPRESSOR_TASK_MSG_TYPE_RSP_QUERY_TEMPERATURE_SETTING) {     
                log_error("comm query temperature setting rsp type:%d err.\r\n",rsp_msg.response.type);
                continue;
            }

            *setting = rsp_msg.response.temperature_setting;
            return 0;
        }
    }
        
    log_error("comm query temperature setting timeout err.\r\n");
    return -1;
}

/*
* @brief 设置压缩机温度控制区间
* @param contex 通信任务上下文
* @param setting 温度设置值
* @return -1 失败
* @return  0 成功
* @note
*/
static int temperature_setting(communication_task_contex_t *contex,int8_t setting)
{
    osStatus status;
    osEvent os_event;

    compressor_task_message_t req_msg,rsp_msg;
    xtimer_t timer;

    req_msg.request.type = COMPRESSOR_TASK_MSG_TYPE_TEMPERATURE_SETTING;    
    req_msg.request.rsp_message_queue_id = contex->temperature_setting_rsp_msg_q_id;
    req_msg.request.temperature_setting = setting;
    xtimer_init(&timer,0,ADU_QUERY_TEMPERATURE_SETTING_TIMEOUT);
    
    /*发送消息*/
    status = osMessagePut(compressor_task_msg_q_id,(uint32_t)&req_msg,xtimer_value(&timer));
    log_assert_bool_false(status == osOK);

    /*等待消息*/
    while (xtimer_value(&timer) > 0) {
        os_event = osMessageGet(contex->temperature_setting_rsp_msg_q_id,xtimer_value(&timer));
        if (os_event.status == osEventMessage ){
            rsp_msg = *(compressor_task_message_t *)os_event.value.v;
            if (rsp_msg.response.type != COMPRESSOR_TASK_MSG_TYPE_RSP_TEMPERATURE_SETTING) {     
                log_error("comm temperature setting rsp type:%d err.\r\n",rsp_msg.response.type);
                continue;
            }

            return rsp_msg.response.result == COMPRESSOR_TASK_SUCCESS ? 0 : -1;
        }
    }
        
    log_error("comm set temperature level timeout err.\r\n");
    return -1;
}

/*
* @brief 查询厂家ID和硬件版本
* @param contex 通信任务上下文
* @param manufacturer 厂家id指针
* @return -1 失败
* @return  0 成功
* @note
*/

static int query_manufacturer_and_hardware_version(communication_task_contex_t *contex,uint16_t *manufacturer)
{
    *manufacturer = contex->manufacturer_id;
    return 0;
}
/*
* @brief 查询软件版本
* @param contex 通信任务上下文
* @param manufacturer软件版本指针
* @return -1 失败
* @return  0 成功
* @note
*/

static int query_software_version(communication_task_contex_t *contex,uint32_t *software_version)
{
    *software_version = contex->software_version;
    return 0;
}
/*
* @brief 处理升级
* @param contex 通信任务上下文
* @param manufacturer软件版本指针
* @return -1 失败
* @return  0 成功
* @note
*/

static int process_update(application_update_t *update,uint32_t timeout)
{
#define  SIZE_STR_BUFFER               7

    char file_name[FYMODEM_FILE_NAME_MAX_LENGTH + 1];
    char md5_value[16];
    char md5_str_buffer[33];
    char size_str_buffer[SIZE_STR_BUFFER];

    int size = 0;

    log_info("start process update...\r\n");
    size = fymodem_receive(&communication_uart_handle,APPLICATION_UPDATE_BASE_ADDR,APPLICATION_SIZE_LIMIT,file_name,timeout);
    if (size > 0) {
        log_info("update file_name:%s.\r\n",file_name);
        if (size != update->size) {
            log_error("file ymodem get size:%d != notify size:%d.\r\n",size,update->size);
            return -1;
        }
        /*计算MD5*/
        md5((char *)APPLICATION_UPDATE_BASE_ADDR,size,md5_value);
        xstring_hex_to_string(md5_str_buffer,md5_value,16);

        if (strcmp(md5_str_buffer,update->md5_str) != 0) {
            log_error("file md5 calculate:%s != notify md5:%s.\r\n",md5_str_buffer,update->md5_str);
            return -1;
        }
        /*int转换成字符串*/
        snprintf(size_str_buffer,SIZE_STR_BUFFER,"%d",size);

        log_info("check update size:%s md5:%s ok.\r\n",size_str_buffer,md5_str_buffer);

        /*设置更新size*/
        log_info("set update size env...\r\n");
        if (device_env_set(ENV_BOOTLOADER_UPDATE_SIZE_NAME,size_str_buffer) != 0) {
            return -1;
        }
        /*设置更新md5*/
        log_info("set update md5 env...\r\n");
        if (device_env_set(ENV_BOOTLOADER_UPDATE_MD5_NAME,md5_str_buffer)!= 0) {
            return -1;
        }
        log_info("set flag new env...\r\n");
        /*设置更新标志*/
        device_env_set(ENV_BOOTLOADER_FLAG_NAME,ENV_BOOTLOADER_NEW);

        log_info("all done. reboot...\r\n");
        /*禁止看门狗*/
        WWDT_Deinit(WWDT);
        /*复位准备升级*/
        extern void hal_delay(void);
        hal_delay();
        __NVIC_SystemReset();
    } else {
        log_error("ymodem recv update file err.size:%d.\r\n",size);
        return -1;
    }

    return 0;
}

/*
* @brief 串口接收主机ADU
* @param handle 串口句柄
* @param adu 数据缓存指针
* @param timeout 等待超时时间
* @return -1 失败
* @return  > 0 成功接收的数据量
* @note
*/
static int receive_adu(xuart_handle_t *handle,uint8_t *adu,uint8_t size,uint32_t timeout)
{
    int rc = -1;
    int read_size,read_size_total = 0;
    char buffer[ADU_SIZE_MAX * 2 + 1];

    while (read_size_total < size) {
        rc = xuart_select(handle,timeout);
        if (rc == -1) {
            log_error("adu select error.read total:%d.\r\n",read_size_total);
            goto exit;
        }
        /*读完了一帧数据*/
        if (rc == 0) {
           goto exit;
        
        }
        /*数据溢出*/
        if (rc > size - read_size_total) {
            log_error("adu size:%d too large than buffer size:%d .error.\r\n",read_size_total + rc,size);
            goto exit;
        }
        read_size = rc;
        rc = xuart_read(handle,adu + read_size_total,read_size);
        if (rc == -1) {
            log_error("adu read error.read total:%d.\r\n",read_size_total);
            goto exit;
        }
   
        /*更新接收数量*/
        read_size_total += rc;
        /*超时时间变为帧超时间*/
        timeout = ADU_FRAME_TIMEOUT;
    }

exit:
    /*打印接收的数据*/
    xstring_hex_to_string(buffer,(char *const)adu,read_size_total);
    log_debug("[recv] %s\r\n",buffer);

    if (rc == 0) {
        log_debug("adu recv over.read size total:%d.\r\n",read_size_total);
        return read_size_total;
    }
    return -1;
}


/*
* @brief 解析adu
* @param adu 数据缓存指针
* @param size 数据大小
* @param rsp 回应的数据缓存指针
* @param update 回应的是否需要升级
* @return -1 失败 
* @return  > 0 回应的adu大小
* @note
*/

static int parse_adu(uint8_t *adu,uint8_t size,uint8_t *rsp,application_update_t *update)
{
    int rc;
    uint8_t *pdu;
    uint8_t pdu_size;
    uint8_t communication_addr;
    uint8_t code;
    uint8_t scale_addr;
    int8_t setting;
    uint16_t manufacturer_id;
    uint32_t software_verion;
    uint16_t calibration_weight;
    uint16_t crc_received,crc_calculated;

    uint8_t status;
    uint8_t scale_cnt;
    uint8_t scale_addr_config[SCALE_CNT_MAX];
    int8_t  temperature;
    int16_t net_weight[SCALE_CNT_MAX];
    uint8_t rsp_size = 0;
    uint8_t rsp_offset = 0;

    if (size < ADU_HEAD_SIZE + ADU_PDU_SIZE_REGION_SIZE + PDU_ADDR_SIZE + PDU_CODE_SIZE + ADU_CRC_SIZE) {
        log_error("adu size:%d < %d err.\r\n",size,ADU_HEAD_SIZE + ADU_PDU_SIZE_REGION_SIZE + PDU_ADDR_SIZE + PDU_CODE_SIZE + ADU_CRC_SIZE);
        return -1;
    }
    /*校验CRC*/
    crc_received = (uint16_t)adu[size - 1] << 8 | adu[size - 2];
    crc_calculated = calculate_crc16(adu,size - ADU_CRC_SIZE);
    if (crc_received != crc_calculated) {
        log_error("crc err.claculate:%d receive:%d.\r\n",crc_calculated,crc_received);
        return -1;
    }
    /*校验数据头*/
    if (adu[ADU_HEAD_OFFSET] != ADU_HEAD0_VALUE || \
        adu[ADU_HEAD_OFFSET + 1] != ADU_HEAD1_VALUE) {
        log_error("communication head0:%d != %d or head1 != %d err.\r\n",adu[ADU_HEAD_OFFSET],adu[ADU_HEAD_OFFSET + 1]);
        return -1;
    }
    /*校验数据长度*/
    pdu_size = adu[ADU_PDU_SIZE_REGION_OFFSET];
    if (pdu_size != size - ADU_HEAD_SIZE - ADU_PDU_SIZE_REGION_SIZE - ADU_CRC_SIZE ) {
        log_error("数据域长度%d != %d err.\r\n",pdu_size,size - ADU_HEAD_SIZE - ADU_PDU_SIZE_REGION_SIZE - ADU_CRC_SIZE);
        return -1;
    }
    /*解析pdu*/
    pdu = &adu[ADU_PDU_OFFSET];
    /*校验通信地址*/
    communication_addr = pdu[PDU_ADDR_OFFSET];
    if (communication_addr != CONTROLLER_ADDR) {
        log_error("communication addr:%d != %d err.\r\n",communication_addr,CONTROLLER_ADDR);
        return -1;
    }

    /*校验命令码*/
    code = pdu[PDU_CODE_OFFSET];
    /*回应adu构建*/
    rsp[rsp_offset ++] = ADU_HEAD0_VALUE;
    rsp[rsp_offset ++] = ADU_HEAD1_VALUE;
    rsp[rsp_offset ++] = 0;//数据域长度预留
    rsp[rsp_offset ++] = CONTROLLER_ADDR;
    rsp[rsp_offset ++] = code;
    switch (code) {
        case CODE_REMOVE_TARE_WEIGHT:/*去皮*/
            if (pdu_size != DATA_REMOVE_TARE_SIZE) {
                log_error("remove tare data size:%d != %d err.\r\n",pdu_size,DATA_REMOVE_TARE_SIZE);
                return -1;
            }
            scale_addr = pdu[PDU_DATA_OFFSET + DATA_SCALE_ADDR_OFFSET];
            log_debug("scale addr:%d remove tare weight...\r\n",scale_addr);
            rc = remove_tare_weight(&communication_task_contex,scale_addr);
            if (rc == 0) {
                rsp[rsp_offset ++] = DATA_RESULT_REMOVE_TARE_SUCCESS;
            } else {
                rsp[rsp_offset ++] = DATA_RESULT_REMOVE_TARE_FAIL;
            }
            break;
        case CODE_CALIBRATION_ZERO:/*0点校准*/
            if (pdu_size != DATA_CALIBRATION_SIZE) {
                log_error("calibration data size:%d != %d err.\r\n",pdu_size,DATA_CALIBRATION_SIZE);
                return -1;
            }
            scale_addr = pdu[PDU_DATA_OFFSET + DATA_SCALE_ADDR_OFFSET];
            calibration_weight = (uint16_t)pdu[PDU_DATA_OFFSET + DATA_CALIBRATION_WEIGHT_OFFSET] | pdu[PDU_DATA_OFFSET + DATA_CALIBRATION_WEIGHT_OFFSET + 1] << 8 ;
            log_debug("scale addr:%d calibration weight:%d...\r\n",scale_addr,calibration_weight);
            rc = calibration_zero(&communication_task_contex,scale_addr,calibration_weight);
            if (rc == 0) {
                rsp[rsp_offset ++] = DATA_RESULT_CALIBRATION_SUCCESS;
            } else {
                rsp[rsp_offset ++] = DATA_RESULT_CALIBRATION_FAIL;
            }
            break;
        case CODE_CALIBRATION_FULL:/*增益校准*/
            if (pdu_size != DATA_CALIBRATION_SIZE) {
                log_error("calibration data size:%d != %d err.\r\n",pdu_size,DATA_CALIBRATION_SIZE);
                return -1;
            }
            scale_addr = pdu[PDU_DATA_OFFSET + DATA_SCALE_ADDR_OFFSET];
            calibration_weight = (uint16_t)pdu[PDU_DATA_OFFSET + DATA_CALIBRATION_WEIGHT_OFFSET] | pdu[PDU_DATA_OFFSET + DATA_CALIBRATION_WEIGHT_OFFSET + 1] << 8 ;
            log_debug("scale addr:%d calibration weight:%d...\r\n",scale_addr,calibration_weight);
            rc = calibration_full(&communication_task_contex,scale_addr,calibration_weight);
            if (rc == 0) {
                rsp[rsp_offset ++] = DATA_RESULT_CALIBRATION_SUCCESS;
            } else {
                rsp[rsp_offset ++] = DATA_RESULT_CALIBRATION_FAIL;
            }
            break;
        case CODE_QUERY_NET_WEIGHT:/*净重*/
            if (pdu_size != DATA_QUERY_NET_WEIGHT_SIZE) {
                log_error("query net weight data size:%d != %d err.\r\n",pdu_size,DATA_QUERY_NET_WEIGHT_SIZE);
                return -1;
            }
            scale_addr = pdu[PDU_DATA_OFFSET + DATA_SCALE_ADDR_OFFSET];
            log_debug("scale addr:%d query net weight...\r\n",scale_addr);
            
            rc = query_net_weight(&communication_task_contex,scale_addr,net_weight);
            if (rc <= 0) {
                log_error("query net weight internal err.\r\n");
                return -1;
            } 
            uint8_t index_end;
            if (scale_addr == 0) {
                index_end = SCALE_CNT_MAX;
            } else {
                index_end = 1;
            }

            for (uint8_t i = 0; i < index_end; i++) {
                /*转换为协议传感器故障值*/
                if (net_weight[i] == SCALE_TASK_NET_WEIGHT_ERR_VALUE) {
                    net_weight[i] = DATA_NET_WEIGHT_ERR_VALUE;
                }
                rsp[rsp_offset ++] = net_weight[i] & 0xFF;
                rsp[rsp_offset ++] = (net_weight[i] >> 8) & 0xFF;
            } 
            break;
        case CODE_SCALE_ADDR_CONFIGRATION:/*电子秤数量*/
            if (pdu_size != DATA_SCALE_ADDR_CONFIGRATION_SIZE) {
                log_error("query scale data size:%d != %d err.\r\n",pdu_size,DATA_SCALE_ADDR_CONFIGRATION_SIZE);
                return -1;
            }
            log_debug("query scale cnt...\r\n");
            scale_cnt = query_scale_cnt(&communication_task_contex,scale_addr_config);
            for(uint8_t i = 0;i < scale_cnt;i ++) {
                rsp[rsp_offset ++] = scale_addr_config[i];
            }
            break;
        case CODE_QUERY_DOOR_STATUS:/*门状态*/
            if (pdu_size != DATA_QUERY_DOOR_STATUS_SIZE) {
                log_error("query door status data size:%d != %d err.\r\n",pdu_size,DATA_QUERY_DOOR_STATUS_SIZE);
                return -1;
            }
            log_debug("query door status...\r\n");
            rc = query_door_status(&communication_task_contex,&status);
            if (rc != 0) {
                log_error("query door status internal err.\r\n");
                return -1;
            }
            if (status == LOCK_TASK_STATUS_DOOR_OPEN) {
                rsp[rsp_offset ++] = DATA_STATUS_DOOR_OPEN;
            } else {
                rsp[rsp_offset ++] = DATA_STATUS_DOOR_CLOSE; 
            }
            break;
        case CODE_LOCK_LOCK:/*关锁*/
            if (pdu_size != DATA_LOCK_LOCK_SIZE) {
                log_error("lock lock data size:%d != %d err.\r\n",pdu_size,DATA_LOCK_LOCK_SIZE);
                return -1;
            }
            log_debug("lock lock...\r\n");
            rc = lock_lock(&communication_task_contex);
            if (rc == 0) {
                rsp[rsp_offset ++] = DATA_RESULT_LOCK_SUCCESS;
            } else {
                rsp[rsp_offset ++] = DATA_RESULT_LOCK_FAIL;
            }
            break;
        case CODE_UNLOCK_LOCK:/*开锁*/
            if (pdu_size != DATA_UNLOCK_LOCK_SIZE) {
                log_error("unlock lock data size:%d != %d err.\r\n",pdu_size,DATA_UNLOCK_LOCK_SIZE);
                return -1;
            }
            log_debug("unlock lock...\r\n");
            rc = unlock_lock(&communication_task_contex);
            if (rc == 0) {
                rsp[rsp_offset ++] = DATA_RESULT_UNLOCK_SUCCESS;
            } else {
                rsp[rsp_offset ++] = DATA_RESULT_UNLOCK_FAIL;
            }
            break;      
        case CODE_QUERY_LOCK_STATUS:/*锁状态*/
            if (pdu_size != DATA_QUERY_LOCK_STATUS_SIZE) {
                log_error("query lock status data size:%d != %d err.\r\n",pdu_size,DATA_QUERY_LOCK_STATUS_SIZE);
                return -1;
            }
            log_debug("query lock status...\r\n");
            rc = query_lock_status(&communication_task_contex,&status);
            if (rc != 0) {
                log_error("query lock status internal err.\r\n");
                return -1;
            }
            if (status == LOCK_TASK_STATUS_LOCK_LOCKED) {
                rsp[rsp_offset ++] = DATA_STATUS_LOCK_LOCKED;
            } else {
                rsp[rsp_offset ++] = DATA_STATUS_LOCK_UNLOCKED; 
            }
            break;
        case CODE_QUERY_TEMPERATURE:/*查询温度设置和温度值*/
            if (pdu_size != DATA_QUERY_TEMPERATURE_SIZE) {
                log_error("query temperature data size:%d != %d err.\r\n",pdu_size,DATA_QUERY_TEMPERATURE_SIZE);
                return -1;
            }
            log_debug("query temperature...\r\n");
            rc = query_temperature(&communication_task_contex,&temperature);
            if (rc != 0) {
                log_error("query temperature internal err.\r\n");
                return -1;
            }
            rsp[rsp_offset ++] = temperature;

            rc = query_temperature_setting(&communication_task_contex,&setting);
            if (rc != 0) {
                log_error("query temperature setting internal err.\r\n");
                return -1;
            }
            rsp[rsp_offset ++] = setting;

            break; 
        case CODE_SET_TEMPERATURE:/*设置温度区间*/
            if (pdu_size != DATA_SET_TEMPERATURE_SIZE) {
                log_error("set temperature data size:%d != %d err.\r\n",pdu_size,DATA_SET_TEMPERATURE_SIZE);
                return -1;
            }
            setting = pdu[PDU_DATA_OFFSET + DATA_TEMPERATURE_OFFSET];
            log_debug("set temperature :%d...\r\n",setting);
            rc = temperature_setting(&communication_task_contex,setting);
            if (rc == 0) {
                rsp[rsp_offset ++] = DATA_RESULT_SET_TEMPERATURE_SUCCESS;     
            } else {
                rsp[rsp_offset ++] = DATA_RESULT_SET_TEMPERATURE_FAIL; 
            }                
            break;
        case CODE_QUERY_MANUFACTURER_HARDWARE_VER:/*查询厂商ID*/
            if (pdu_size != DATA_QUERY_HARDWARE_VER_SIZE) {
                log_error("query manafacture data size:%d != %d err.\r\n",pdu_size,DATA_QUERY_HARDWARE_VER_SIZE);
                return -1;
            }
            log_debug("query manufacture...\r\n");
            query_manufacturer_and_hardware_version(&communication_task_contex,&manufacturer_id);
            rsp[rsp_offset ++] = (manufacturer_id >> 8) & 0xFF;      
            rsp[rsp_offset ++] = manufacturer_id & 0xFF;      
            break;  
        case CODE_FIRMWARE_VERSION:/*查询软件版本*/
            if (pdu_size != DATA_QUERY_SOFTWARE_VER_SIZE) {
                log_error("query software data size:%d != %d err.\r\n",pdu_size,DATA_QUERY_SOFTWARE_VER_SIZE);
                return -1;
            }
            log_debug("query software ver...\r\n");
            query_software_version(&communication_task_contex,&software_verion);
            rsp[rsp_offset ++] = software_verion & 0xFF;
            rsp[rsp_offset ++] = (software_verion >> 8) & 0xFF;      
            rsp[rsp_offset ++] = (software_verion >> 16) & 0xFF;     
            break;

        case CODE_NOTIFY_UPDATE:/*通知升级信息*/
            if (pdu_size != DATA_NOTIFY_UPDATE_SIZE) {
                log_error("notify update data size:%d != %d err.\r\n",pdu_size,DATA_NOTIFY_UPDATE_SIZE);
                return -1;
            }
            log_debug("notify update...\r\n");
            
            /*复制文件长度*/
            update->size = 0;
            update->size |= (uint32_t)pdu[PDU_DATA_OFFSET + DATA_FILE_SIZE_OFFSET + 0] << 24;
            update->size |= (uint32_t)pdu[PDU_DATA_OFFSET + DATA_FILE_SIZE_OFFSET + 1] << 16;
            update->size |= (uint32_t)pdu[PDU_DATA_OFFSET + DATA_FILE_SIZE_OFFSET + 2] << 8;
            update->size |= (uint32_t)pdu[PDU_DATA_OFFSET + DATA_FILE_SIZE_OFFSET + 3] << 0;

            if (update->size > APPLICATION_SIZE_LIMIT) {
                log_error("notify update file size:%d > limit size %d err.\r\n",update->size,APPLICATION_SIZE_LIMIT);
                return -1; 
            }
            /*复制MD5*/
            for (uint8_t i = 0;i < 16 ;i ++) {
                update->md5[i] = pdu[PDU_DATA_OFFSET + DATA_FILE_MD5_OFFSET + i];
            }
            xstring_hex_to_string(update->md5_str,update->md5,16);
            update->update = COMMUNICATION_TASK_APPLICATION_UPDATE;
            log_debug("update file size:%d md5:%s\r\n",update->size,update->md5_str);

            break;
        default:
            log_error("unknow code:%d err.\r\n",code);
    }
   
    /*添加CRC16*/
    rsp[ADU_PDU_SIZE_REGION_OFFSET] = rsp_offset - ADU_HEAD_SIZE - ADU_PDU_SIZE_REGION_SIZE;
    rsp_size = adu_add_crc16(rsp,rsp_offset);
    return rsp_size;
}


/*
* @brief 通过串口回应处理结果
* @param handle 串口句柄
* @param adu 结果缓存指针
* @param handle 串口句柄
* @param adu 结果缓存指针
* @param size 结果大小
* @param timeout 发送超时时间
* @return -1 失败 
* @return  0 成功 
* @note
*/
static int send_adu(xuart_handle_t *handle,uint8_t *adu,uint8_t size,uint32_t timeout)
{
    uint8_t write_size;
    char buffer[ADU_SIZE_MAX * 2 + 1];

    write_size = xuart_write(handle,(const uint8_t *)adu,size);

    /*打印输出的数据*/
    xstring_hex_to_string(buffer,(char *const)adu,write_size);
    log_debug("[send] %s\r\n",buffer);

    if (size != write_size){
        log_error("communication err in  serial write. expect:%d write:%d.\r\n",size,write_size); 
        return -1;      
     }
  
    return xuart_complete(handle,timeout);
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int get_serial_port_by_addr(uint8_t addr)
{
    int port = -1;
    switch (addr) {
    case 11:
        port = 1;
        break;
    case 12:
        port = 2;
        break;
    case 21:
        port = 3;
        break;
    case 22:
        port = 4;
        break;
    case 31:
        port = 5;
        break;
    case 32:
        port = 6;
        break;
    case 41:
        port = 7;
        break;
    case 51:
        port = 8;
        break;
    default :
        log_error("addr:%d is invalid.\r\n",addr);
        break;
    }

    return port;
}
/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static xuart_handle_t *get_serial_handle_by_port(communication_task_contex_t *contex,uint8_t port)
{
    if (!contex->initialized) {
        return NULL;
    }
    for (uint8_t i = 0;i <contex->cnt;i ++) {
        if (contex->scale_task_contex[i].port == port) {
            return &contex->scale_task_contex[i].handle;
        }
    }
    return NULL;
}

/*控制器任务通信中断处理*/
void FLEXCOMM0_IRQHandler()
{
    if (communication_uart_handle.is_port_open) {
        nxp_uart_hal_isr(&communication_uart_handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM1_IRQHandler()
{
    xuart_handle_t *handle;

    handle = get_serial_handle_by_port(&communication_task_contex,1);
    if (handle->is_port_open) {
        nxp_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM2_IRQHandler()
{
    xuart_handle_t *handle;

    handle = get_serial_handle_by_port(&communication_task_contex,2);
    if (handle->is_port_open) {
        nxp_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM3_IRQHandler()
{
    xuart_handle_t *handle;

    handle = get_serial_handle_by_port(&communication_task_contex,3);
    if (handle->is_port_open) {
        nxp_uart_hal_isr(handle);
    }

}
/*电子秤任务通信中断处理*/
void FLEXCOMM4_IRQHandler()
{
    xuart_handle_t *handle;

    handle = get_serial_handle_by_port(&communication_task_contex,4);
    if (handle->is_port_open) {
        nxp_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM5_IRQHandler()
{
    xuart_handle_t *handle;

    handle = get_serial_handle_by_port(&communication_task_contex,5);
    if (handle->is_port_open) {
        nxp_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM6_IRQHandler()
{
    xuart_handle_t *handle;

    handle = get_serial_handle_by_port(&communication_task_contex,6);
    if (handle->is_port_open) {
        nxp_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM7_IRQHandler()
{
    xuart_handle_t *handle;

    handle = get_serial_handle_by_port(&communication_task_contex,7);
    if (handle->is_port_open) {
        nxp_uart_hal_isr(handle);
    }

}
/*电子秤任务通信中断处理*/
void FLEXCOMM8_IRQHandler()
{
    xuart_handle_t *handle;

    handle = get_serial_handle_by_port(&communication_task_contex,8);
    if (handle->is_port_open) {
        nxp_uart_hal_isr(handle);
    }

}


/*
* @brief 通信任务上下文配置初始化
* @param contex 任务参数指针
* @param host_msg_id 主任务消息队列句柄
* @return 无
* @note
*/
static void communication_task_contex_init(communication_task_contex_t *contex)
{
    int rc;
    /*电子秤地址配置信息*/
    scale_addr_configration_t scale_addr;
    rc = communication_read_scale_addr_configration(&scale_addr);
    log_assert_bool_false(rc == 0);

    contex->cnt = scale_addr.cnt;
    for (uint8_t i = 0;i < contex->cnt;i ++) {
        contex->scale_task_contex[i].internal_addr = scale_addr.value[i];
        contex->scale_task_contex[i].phy_addr = COMMUNICATION_TASK_SCALE_DEFAULT_ADDR;
        contex->scale_task_contex[i].port = get_serial_port_by_addr(contex->scale_task_contex[i].internal_addr);
        contex->scale_task_contex[i].baud_rates = SCALE_TASK_SERIAL_BAUDRATES;
        contex->scale_task_contex[i].data_bits = SCALE_TASK_SERIAL_DATABITS;
        contex->scale_task_contex[i].stop_bits = SCALE_TASK_SERIAL_STOPBITS;
        contex->scale_task_contex[i].flag = 1 << i;

        xuart_register_hal_driver(&xuart_hal_driver);

        rc = xuart_open(&contex->scale_task_contex[i].handle,contex->scale_task_contex[i].port, contex->scale_task_contex[i].baud_rates,contex->scale_task_contex[i].data_bits,contex->scale_task_contex[i].stop_bits,
                        contex->scale_task_contex[i].recv, SCALE_TASK_RX_BUFFER_SIZE,contex->scale_task_contex[i].send, SCALE_TASK_TX_BUFFER_SIZE);
        log_assert_bool_false(rc == 0);
        /*清空接收缓存*/
        xuart_clear(&contex->scale_task_contex[i].handle);

        /*创建电子秤消息队列*/
        osMessageQDef(scale_task_msg_queue,1,uint32_t);
        contex->scale_task_contex[i].msg_q_id = osMessageCreate(osMessageQ(scale_task_msg_queue),0);
        log_assert_bool_false(contex->scale_task_contex[i].msg_q_id);
        /*创建电子秤任务*/
        osThreadDef(scale_task, scale_task, osPriorityNormal, 0, 256);
        contex->scale_task_contex[i].task_hdl = osThreadCreate(osThread(scale_task),&contex->scale_task_contex[i]);
        log_assert_bool_false(contex->scale_task_contex[i].task_hdl);
    }  
    /*开锁回应消息队列*/
    osMessageQDef(unlock_lock_rsp_msg_q,1,uint32_t);
    contex->unlock_lock_rsp_msg_q_id = osMessageCreate(osMessageQ(unlock_lock_rsp_msg_q),0);
    log_assert_bool_false(contex->unlock_lock_rsp_msg_q_id);
    /*关锁回应消息队列*/
    osMessageQDef(lock_lock_rsp_msg_q,1,uint32_t);
    contex->lock_lock_rsp_msg_q_id = osMessageCreate(osMessageQ(lock_lock_rsp_msg_q),0);
    log_assert_bool_false(contex->lock_lock_rsp_msg_q_id);
    /*锁状态回应消息队列*/
    osMessageQDef(query_lock_status_rsp_msg_q,1,uint32_t);
    contex->query_lock_status_rsp_msg_q_id = osMessageCreate(osMessageQ(query_lock_status_rsp_msg_q),0);
    log_assert_bool_false(contex->query_lock_status_rsp_msg_q_id);
    /*门状态回应消息队列*/
    osMessageQDef(query_door_status_rsp_msg_q,1,uint32_t);
    contex->query_door_status_rsp_msg_q_id = osMessageCreate(osMessageQ(query_door_status_rsp_msg_q),0);
    log_assert_bool_false(contex->query_door_status_rsp_msg_q_id);
    /*温度回应消息队列*/
    osMessageQDef(query_temperature_rsp_msg_q,1,uint32_t);
    contex->query_temperature_rsp_msg_q_id = osMessageCreate(osMessageQ(query_temperature_rsp_msg_q),0);
    log_assert_bool_false(contex->query_temperature_rsp_msg_q_id);
    /*查询温度设置消息队列*/
    osMessageQDef(query_temperature_setting_rsp_msg_q,1,uint32_t);
    contex->query_temperature_setting_rsp_msg_q_id = osMessageCreate(osMessageQ(query_temperature_setting_rsp_msg_q),0);
    log_assert_bool_false(contex->query_temperature_setting_rsp_msg_q_id);
    /*设置温度等级消息队列*/
    osMessageQDef(temperature_setting_rsp_msg_q,1,uint32_t);
    contex->temperature_setting_rsp_msg_q_id = osMessageCreate(osMessageQ(temperature_setting_rsp_msg_q),0);
    log_assert_bool_false(contex->temperature_setting_rsp_msg_q_id);

    /*净重回应消息队列*/
    osMessageQDef(net_weight_rsp_msg_q,SCALE_CNT_MAX,uint32_t);
    contex->net_weight_rsp_msg_q_id = osMessageCreate(osMessageQ(net_weight_rsp_msg_q),0);
    log_assert_bool_false(contex->net_weight_rsp_msg_q_id);
    /*去皮回应消息队列*/
    osMessageQDef(remove_tare_rsp_msg_q,SCALE_CNT_MAX,uint32_t);
    contex->remove_tare_rsp_msg_q_id = osMessageCreate(osMessageQ(remove_tare_rsp_msg_q),0);
    log_assert_bool_false(contex->remove_tare_rsp_msg_q_id);
    /*0点校准回应消息队列*/
    osMessageQDef(calibration_zero_rsp_msg_q,SCALE_CNT_MAX,uint32_t);
    contex->calibration_zero_rsp_msg_q_id = osMessageCreate(osMessageQ(calibration_zero_rsp_msg_q),0);
    log_assert_bool_false(contex->calibration_zero_rsp_msg_q_id);
    /*增益校准回应消息队列*/
    osMessageQDef(calibration_full_rsp_msg_q,SCALE_CNT_MAX,uint32_t);
    contex->calibration_full_rsp_msg_q_id = osMessageCreate(osMessageQ(calibration_full_rsp_msg_q),0);
    log_assert_bool_false(contex->calibration_full_rsp_msg_q_id);
    /*厂商ID和硬件版本*/
    contex->manufacturer_id = DATA_MANUFACTURER_CHANGHONG_ID;

    /*软件版本*/
    contex->software_version = FIRMWARE_VERSION_HEX;

    contex->initialized = true;
}
       
/*
* @brief 与主机通信任务
* @param argument 任务参数
* @return 无
* @note
*/       
void communication_task(void const * argument)
{
    int rc; 
    application_update_t update;
    

    uint8_t adu_recv[ADU_SIZE_MAX];
    uint8_t adu_send[ADU_SIZE_MAX];
 
    xuart_register_hal_driver(&xuart_hal_driver);

    rc = xuart_open(&communication_uart_handle,COMMUNICATION_TASK_SERIAL_PORT,COMMUNICATION_TASK_SERIAL_BAUDRATES,COMMUNICATION_TASK_SERIAL_DATABITS,COMMUNICATION_TASK_SERIAL_STOPBITS,
                    comm_recv_buffer, COMMUNICATION_TASK_RX_BUFFER_SIZE,comm_send_buffer, COMMUNICATION_TASK_TX_BUFFER_SIZE);
    log_assert_bool_false(rc == 0);
    communication_task_contex_init(&communication_task_contex);
    log_debug("communication task contex init ok.\r\n");

    /*默认配置不升级*/
    update.update = COMMUNICATION_TASK_APPLICATION_NORMAL;

    /*清空接收缓存*/
    xuart_clear(&communication_uart_handle);
    while (1) {

        /*接收主机发送的adu*/
        rc = receive_adu(&communication_uart_handle,(uint8_t *)adu_recv,ADU_SIZE_MAX,ADU_WAIT_TIMEOUT);
        if (rc < 0) {
            /*清空接收缓存*/
            xuart_clear(&communication_uart_handle);
            continue;
        }
        /*解析处理pdu*/
        rc = parse_adu(adu_recv,rc,adu_send,&update);
        if (rc < 0) {
            update.update = COMMUNICATION_TASK_APPLICATION_NORMAL;
            continue;
        }
        /*回应主机处理结果*/
        rc = send_adu(&communication_uart_handle,adu_send,rc,ADU_SEND_TIMEOUT);
        if (rc < 0) {
            continue;
        }
        if (update.update == COMMUNICATION_TASK_APPLICATION_UPDATE) {
            rc = process_update(&update,COMMUNICATION_TASK_UPDATE_TIMEOUT);
            update.update = COMMUNICATION_TASK_APPLICATION_NORMAL;
            if (rc < 0) {
                log_error("update err.\r\n");
            }
        }

    }
}
