#include "board.h"
#include "cmsis_os.h"
#include "utils.h"
#include "serial.h"
#include "firmware_version.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "controller_task.h"
#include "log.h"

int controller_serial_handle;
extern serial_hal_driver_t nxp_serial_uart_hal_driver;
extern void nxp_serial_uart_hal_isr(int handle);
osThreadId   controller_task_hdl;
osMessageQId controller_task_msg_q_id;

osMessageQId controller_task_cfg_msg_q_id;
osMessageQId controller_task_net_weight_msg_q_id;
osMessageQId controller_task_remove_tare_weight_msg_q_id;
osMessageQId controller_task_calibration_zero_msg_q_id;
osMessageQId controller_task_calibration_full_msg_q_id;


typedef enum
{
    ADU_HEAD_STEP = 0,
    ADU_PDU_STEP,
    ADU_CRC_STEP
}adu_step_t;



/*通信协议部分*/
/*ADU*/
#define  ADU_SIZE_MAX                  20
#define  ADU_HEAD_OFFSET               0
#define  ADU_HEAD_SIZE                 2
#define  ADU_HEAD0_VALUE               'M'
#define  ADU_HEAD1_VALUE               'L'
#define  ADU_PDU_SIZE_REGION_OFFSET    2
#define  ADU_PDU_SIZE_REGION_SIZE      1
#define  ADU_PDU_OFFSET                3
#define  ADU_CRC_SIZE                  2
/*PDU*/
#define  PDU_SIZE_MIN                  2
#define  PDU_CODE_CONFIGRATION         0
#define  PDU_CODE_NET_WEIGHT           1
#define  PDU_CODE_REMOVE_TARE_WEIGHT   2
#define  PDU_CODE_CALIBRATION_ZERO     3
#define  PDU_CODE_CALIBRATION_FULL     4
#define  PDU_CODE_FIRMWARE_VERSION     5
#define  PDU_CODE_MAX                  PDU_CODE_FIRMWARE_VERSION

#define  PDU_CONTROLLER_ADDR           1
/*协议错误码*/
#define  PDU_NET_WEIGHT_ERR_VALUE      0x7FFF
#define  PDU_SUCCESS_VALUE             0x00
#define  PDU_FAILURE_VALUE             0x01

/*协议时间*/
#define  ADU_WAIT_TIMEOUT              osWaitForever
#define  ADU_FRAME_TIMEOUT             2
#define  ADU_RSP_TIMEOUT               200
#define  ADU_SEND_TIMEOUT              5



/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};
/*
* @brief 计算接收的数据CRC
* @param adu 接收缓存
* @param buffer_length 数据缓存长度
* @return CRC
* @note
*/

static uint16_t calculate_crc16(uint8_t *adu, uint16_t size)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    uint32_t i; /* will index into CRC lookup */

   /* calculate the CRC  */
    while (size --) {
        i = crc_hi ^ *adu++; 
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}
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

    adu[size ++] = crc16 & 0xff;
    adu[size ++] = crc16 >> 8;
    return size;
}

/*电子秤上下文*/
static scale_contex_t scale_contex;

/*
* @brief 读取电子称地址配置
* @param config 硬件配置指针
* @return 0 成功
* @return -1 失败
* @note
*/
static int controller_read_scale_addr_configration(scale_addr_configration_t *addr)
{
    addr->cnt = 4;
    addr->value[0] = 11;
    addr->value[1] = 21;
    addr->value[2] = 31;
    addr->value[3] = 41;
    
    return 0;
}
/*
* @brief 查找地址在配置表中对应的标号
* @param addr 传感器地址
* @return -1 失败
* @return >=0 对应的标号
* @note
*/
static int find_scale_index(const scale_contex_t *contex,uint8_t addr,uint8_t *start,uint8_t *end)
{   
    if (addr == 0) {
        *start = 0;
        *end = contex->cnt;
        return 0;
    } else {
        for (uint8_t i = 0;i < contex->cnt;i ++) {
            if (addr == contex->task[i].addr) {
                *start = i;
                *end = i + 1;
                return 0;
            }
        }
    }

    return -1;
}


/*
* @brief 请求配置
* @param value 配置缓存指针
* @return -1 失败
* @return  > 0 配置数据大小
* @note
*/
static int request_configration(scale_contex_t *contex,uint8_t *value)
{
    for (uint8_t i = 0;i < contex->cnt;i ++) {
        value[i] = contex->task[i].addr;
    }

    return contex->cnt;
}

/*
* @brief 请求净重值
* @param addr 电子秤地址
* @param value 净重量值指针
* @return -1 失败
* @return  > 0 净重量的数据长度
* @note
*/
static int request_net_weight(const scale_contex_t *contex,const uint8_t addr,uint8_t *value)
{
    int rc;
    uint8_t start,end;
    osStatus   status;
    osEvent    os_event;
    task_msg_t req_msg,rsp_msg;
    utils_timer_t timer;
    uint16_t weight;

    req_msg.type = REQ_NET_WEIGHT;
    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);

    /*给对应的电子秤任务发送消息*/
    rc = find_scale_index(contex,addr,&start,&end);
    if (rc != 0) {
        log_error("scale addr:%d is invlaid.\r\n",addr);
        return -1;
    }
    /*发送消息*/
    for (uint8_t i = start;i < end;i ++) {    
        status = osMessagePut(contex->task[i].msg_q_id,*(uint32_t *)&req_msg,utils_timer_value(&timer));
        log_assert(status == osOK);
    }
    /*等待消息*/
    for (uint8_t i = start;i < end;i ++) { 
        weight = PDU_NET_WEIGHT_ERR_VALUE;
        while (utils_timer_value(&timer) > 0) {
            os_event = osMessageGet(contex->task[i].net_weight_msg_q_id,utils_timer_value(&timer));
            if (os_event.status == osEventMessage ){
                rsp_msg = *(task_msg_t *)&os_event.value.v;
                if (rsp_msg.type == RSP_NET_WEIGHT) {
                    weight = rsp_msg.value == SCALE_TASK_NET_WEIGHT_ERR_VALUE ? PDU_NET_WEIGHT_ERR_VALUE : rsp_msg.value;
                    break;
                }
            } 

        } 
        value[i * 2] = weight & 0xFF;
        value[i * 2 + 1] = weight >> 8;    
    }
           
    return (end - start) * 2;
}


/*
* @brief 请求去除皮重
* @param addr 电子秤地址
* @return -1 失败
* @return  0 成功
* @note
*/

static int request_remove_tare_weight(const scale_contex_t *contex,const uint8_t addr,uint8_t *value)
{
    int rc;
    uint8_t start,end;
    osStatus   status;
    osEvent    os_event;
    task_msg_t req_msg,rsp_msg;
    utils_timer_t timer;
    uint8_t result[SCALE_CNT_MAX];

    req_msg.type = REQ_REMOVE_TARE_WEIGHT;
    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);

    /*给对应的电子秤任务发送消息*/
    rc = find_scale_index(contex,addr,&start,&end);
    if (rc != 0) {
        log_error("scale addr:%d is invlaid.\r\n",addr);
        return -1;
    }
    /*发送消息*/
    for (uint8_t i = start;i < end;i ++) {    
        status = osMessagePut(contex->task[i].msg_q_id,*(uint32_t *)&req_msg,utils_timer_value(&timer));
        log_assert(status == osOK);
    }
    /*等待消息*/
    for (uint8_t i = start;i < end;i ++) { 
        result[i] = PDU_FAILURE_VALUE;
        while (utils_timer_value(&timer) > 0) {
            os_event = osMessageGet(contex->task[i].remove_tare_weight_msg_q_id,utils_timer_value(&timer));
            if (os_event.status == osEventMessage ){
                rsp_msg = *(task_msg_t *)&os_event.value.v;
                if (rsp_msg.type == RSP_REMOVE_TARE_WEIGHT) {
                    result[i] = (rsp_msg.value & 0xFF ) == SCALE_TASK_SUCCESS ? PDU_SUCCESS_VALUE : PDU_FAILURE_VALUE;
                    break;
                }
            } 
        } 
    }
    /*只要有一个错误就是失败*/
    value[0] = PDU_SUCCESS_VALUE;
    for (uint8_t i = start;i < end;i ++) {
       if ( result[i] == PDU_FAILURE_VALUE) {
            value[0] = PDU_FAILURE_VALUE;
           break;
        }
     }
    return 1;
}

/*
* @brief 请求0点校准
* @param addr 电子秤地址
* @param weight 校准重力值
* @return -1 失败
* @return  0 成功
* @note
*/
static int request_calibration_zero(const scale_contex_t *contex,const uint8_t addr,const int16_t weight,uint8_t *value)
{
    int rc;
    uint8_t start,end;
    osStatus   status;
    osEvent    os_event;
    task_msg_t req_msg,rsp_msg;
    utils_timer_t timer;
    uint8_t result[SCALE_CNT_MAX];

    req_msg.type = REQ_CALIBRATION_ZERO;
    req_msg.value = weight;
    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);

    /*给对应的电子秤任务发送消息*/
    rc = find_scale_index(contex,addr,&start,&end);
    if (rc != 0) {
        log_error("scale addr:%d is invlaid.\r\n",addr);
        return -1;
    }
    /*发送消息*/
    for (uint8_t i = start;i < end;i ++) {    
        status = osMessagePut(contex->task[i].msg_q_id,*(uint32_t *)&req_msg,utils_timer_value(&timer));
        log_assert(status == osOK);
    }
    /*等待消息*/
    for (uint8_t i = start;i < end;i ++) { 
        result[i] = PDU_FAILURE_VALUE;
        while (utils_timer_value(&timer) > 0) {
            os_event = osMessageGet(contex->task[i].calibration_zero_msg_q_id,utils_timer_value(&timer));
            if (os_event.status == osEventMessage ){
                rsp_msg = *(task_msg_t *)&os_event.value.v;
                if (rsp_msg.type == RSP_CALIBRATION_ZERO) {
                    result[i] = (rsp_msg.value & 0xFF ) == SCALE_TASK_SUCCESS ? PDU_SUCCESS_VALUE : PDU_FAILURE_VALUE;
                    break;
                }
            } 
        } 
    }
    /*只要有一个错误就是失败*/
    value[0] = PDU_SUCCESS_VALUE;
    for (uint8_t i = start;i < end;i ++) {
       if ( result[i] == PDU_FAILURE_VALUE) {
            value[0] = PDU_FAILURE_VALUE;
           break;
        }
     }

    return 1;         
}

/*
* @brief 请求增益校准
* @param addr 电子秤地址
* @param weight 校准重力值
* @return -1 失败
* @return  0 成功
* @note
*/
static int request_calibration_full(const scale_contex_t *contex,const uint8_t addr,const int16_t weight,uint8_t *value)
{
    int rc;
    uint8_t start,end;
    osStatus   status;
    osEvent    os_event;
    task_msg_t req_msg,rsp_msg;
    utils_timer_t timer;
    uint8_t result[SCALE_CNT_MAX];

    req_msg.type = REQ_CALIBRATION_FULL;
    req_msg.value = weight;
    utils_timer_init(&timer,ADU_RSP_TIMEOUT,false);

    /*给对应的电子秤任务发送消息*/
    rc = find_scale_index(contex,addr,&start,&end);
    if (rc != 0) {
        log_error("scale addr:%d is invlaid.\r\n",addr);
        return -1;
    }
    /*发送消息*/
    for (uint8_t i = start;i < end;i ++) {    
        status = osMessagePut(contex->task[i].msg_q_id,*(uint32_t *)&req_msg,utils_timer_value(&timer));
        log_assert(status == osOK);
    }
    /*等待消息*/
    for (uint8_t i = start;i < end;i ++) { 
        result[i] = PDU_FAILURE_VALUE;
        while (utils_timer_value(&timer) > 0) {
            os_event = osMessageGet(contex->task[i].calibration_full_msg_q_id,utils_timer_value(&timer));
            if (os_event.status == osEventMessage ){
                rsp_msg = *(task_msg_t *)&os_event.value.v;
                if (rsp_msg.type == RSP_CALIBRATION_FULL) {
                    result[i] = (rsp_msg.value & 0xFF ) == SCALE_TASK_SUCCESS ? PDU_SUCCESS_VALUE : PDU_FAILURE_VALUE;
                    break;
                }
            } 
        } 
    }
    /*只要有一个错误就是失败*/
    value[0] = PDU_SUCCESS_VALUE;
    for (uint8_t i = start;i < end;i ++) {
       if ( result[i] == PDU_FAILURE_VALUE) {
            value[0] = PDU_FAILURE_VALUE;
           break;
        }
     }

    return 1;         
}

/*
* @brief 请求固件版本
* @param fw_version 固件版本指针
* @return -1 失败
* @return  0 成功
* @note
*/

static int request_firmware_version(uint8_t *fw_version)
{
    fw_version[0] = FIRMWARE_VERSION_HEX & 0xFF;
    fw_version[1] = (FIRMWARE_VERSION_HEX >> 8) & 0xFF;
    fw_version[2] = (FIRMWARE_VERSION_HEX >> 16) & 0xFF;

    return 3;
}

/*
* @brief 串口接收主机ADU
* @param handle 串口句柄
* @param adu 数据缓存指针
* @param wait_timeout 等待超时时间
* @return -1 失败
* @return  0 成功
* @note
*/
static int receive_adu(int handle,uint8_t *adu,uint32_t wait_timeout)
{
    int rc;
    int read_size,read_size_total = 0;
    uint32_t timeout;
    uint16_t crc_calculated;
    uint16_t crc_received;
    adu_step_t step;

    timeout = wait_timeout;
    read_size = ADU_HEAD_SIZE + ADU_PDU_SIZE_REGION_SIZE;
    read_size_total = 0;
    step = ADU_HEAD_STEP;
  
    while(read_size != 0) {
        rc = serial_select(handle,timeout);
        if (rc == -1) {
            log_error("adu select error.read total:%d. read size:%d.\r\n",read_size_total,read_size);
            return -1;
        }
        if (rc == 0) {
            log_error("adu select timeout.read total:%d. read size:%d.timeout:%d.\r\n",read_size_total,read_size,timeout);
            return -1;
        }
  
        rc = serial_read(handle,(char *)adu + read_size_total,read_size);
        if (rc == -1) {
            log_error("adu read error.read total:%d. read size:%d.\r\n",read_size_total,read_size);
            return -1;
        }
   
        /*打印接收的数据*/
        for (int i = 0;i < rc;i++){
            log_debug("<%2X>\r\n", adu[read_size_total + i]);
        }
   
        read_size_total += rc;
        read_size -= rc;
   
        if (read_size == 0) {
            switch(step){
            /*接收到了协议头和数据长度域*/
            case ADU_HEAD_STEP:
                if (adu[ADU_HEAD_OFFSET]     == ADU_HEAD0_VALUE && \
                    adu[ADU_HEAD_OFFSET + 1] == ADU_HEAD1_VALUE) {
                    step = ADU_PDU_STEP;      
                    read_size = adu[ADU_PDU_SIZE_REGION_OFFSET];
                    if (read_size == 0){
                        log_error("adu err in size value:%d.\r\n",adu[ADU_PDU_SIZE_REGION_OFFSET]);
                        return -1;
                    }
                    timeout = ADU_FRAME_TIMEOUT;
                } else {
                    log_error("adu err in head value0:%d value1:%d.\r\n",adu[ADU_HEAD_OFFSET],adu[ADU_HEAD_OFFSET + 1]);
                    return -1;
                } 
                                                                    
                break;
            /*接收完成了PDU的数据*/
            case ADU_PDU_STEP:
                step = ADU_CRC_STEP;      
                read_size = ADU_CRC_SIZE;
                timeout = ADU_FRAME_TIMEOUT;
            break;
            /*接收完成了全部的数据*/
            case ADU_CRC_STEP:
                crc_calculated = calculate_crc16(adu,read_size_total - ADU_CRC_SIZE);
                crc_received = adu[read_size_total - ADU_CRC_SIZE] | adu[read_size_total - ADU_CRC_SIZE + 1] << 8;
                if (crc_calculated != crc_received) {
                    log_error("adu err in crc.recv:%d calculate:%d.\r\n",crc_received,crc_calculated);
                    return -1;
                } else {
                    return read_size_total;
                }
            break;
            default:
                log_error("adu internal err.\r\n");
                return -1;
                }
        }
    }
    log_error("adu internal err.\r\n");
    return -1;
}

/*
* @brief 解析pdu
* @param pdu 数据缓存指针
* @param size 数据大小
* @param adu 回应的数据缓存指针
* @return -1 失败 
* @return  > 0 回应的adu大小
* @note
*/

static int parse_pdu(uint8_t *pdu,uint8_t size,uint8_t *adu)
{
    int rc;
    uint8_t controller_addr;
    uint8_t code;
    uint8_t scale_addr;
    uint8_t pdu_offset = 0;
    uint8_t adu_size = 0;
    uint8_t calibration_weight;

    if (size < PDU_SIZE_MIN) {
        log_error("pdu size:%d < %d err.\r\n",size,PDU_SIZE_MIN);
        return -1;
    }
    
    controller_addr = pdu[pdu_offset ++];
    if (controller_addr != PDU_CONTROLLER_ADDR) {
        log_error("pdu controller addr:%d != %d err.\r\n",controller_addr,PDU_CONTROLLER_ADDR);
        return -1;
    }
    code = pdu[pdu_offset ++];
    if (code > PDU_CODE_MAX) {
        log_error("pdu code:%d > %d err.\r\n",code,PDU_CODE_MAX);
        return -1;
    }
    /*构建回应ADU*/
    adu[adu_size ++] = ADU_HEAD0_VALUE;
    adu[adu_size ++] = ADU_HEAD1_VALUE;
    adu[adu_size ++] = 0;
    adu[adu_size ++] = PDU_CONTROLLER_ADDR;
    adu[adu_size ++] = code;

    switch (code) {
        case PDU_CODE_CONFIGRATION:
            if (size != pdu_offset) {
                log_error("pdu size:%d of configration err.\r\n",size);
                return -1;
            } 
            rc = request_configration(&scale_contex,&adu[adu_size]);
            if (rc < 0) {
                log_error("request configration err.\r\n");
                return -1;
            }
            adu_size += rc;
        break;
    case PDU_CODE_NET_WEIGHT:
        scale_addr = pdu[pdu_offset ++];
        adu[adu_size ++] = scale_addr;

        if (pdu_offset != size ) {
            log_error("pdu size:%d of net weight err.\r\n",size);
            return -1;
        }
        rc = request_net_weight(&scale_contex,scale_addr,&adu[adu_size]);
        if (rc < 0) {
            log_error("req net weight err.\r\n");
            return -1;
        } 
        adu_size += rc;
        break;
     case PDU_CODE_REMOVE_TARE_WEIGHT:
        scale_addr = pdu[pdu_offset ++];
        adu[adu_size ++] = scale_addr;

        if (pdu_offset != size ) {
            log_error("pdu size:%d of remove tare weight err.\r\n",size);
            return -1;
        }
        rc = request_remove_tare_weight(&scale_contex,scale_addr,&adu[adu_size]);
        if (rc < 0) {
            log_error("req remove tare weight fail.\r\n");
            return -1;
        }
        adu_size += rc;
        break;   
     case PDU_CODE_CALIBRATION_ZERO:
        scale_addr = pdu[pdu_offset ++];
        adu[adu_size ++] = scale_addr;
        calibration_weight = pdu[pdu_offset ++];
        calibration_weight|= pdu[pdu_offset ++] << 8;
        if (pdu_offset != size) {
            log_error("pdu size:%d of calibration zero err.\r\n",size);
            return -1;
        }

        rc = request_calibration_zero(&scale_contex,scale_addr,calibration_weight,&adu[adu_size]);
        if (rc < 0) {
            log_error("req calibration zero fail.\r\n");
            return -1;
        }
        adu_size += rc;
        break;  
     case PDU_CODE_CALIBRATION_FULL:
        scale_addr = pdu[pdu_offset ++];
        adu[adu_size ++] = scale_addr;
        calibration_weight = pdu[pdu_offset ++];
        calibration_weight|= pdu[pdu_offset ++] << 8;
        if (pdu_offset != size) {
            log_error("pdu size:%d of calibration full err.\r\n",size);
            return -1;
        }

        rc = request_calibration_full(&scale_contex,scale_addr,calibration_weight,&adu[adu_size]);
        if (rc < 0) {
            log_error("req calibration zero fail.\r\n");
            return -1;
        }
        adu_size += rc;
        break; 
     case PDU_CODE_FIRMWARE_VERSION:
        if (size != pdu_offset) {
            log_error("pdu size:%d of configration err.\r\n",size);
            return -1;
        } 
        rc = request_firmware_version(&adu[adu_size]);
        if (rc < 0) {
            log_error("request configration err.\r\n");
            return -1;
        }
        adu_size += rc;
        break; 
    default:
        log_error("adu internal err.code:%d.\r\n",code);
        return -1;
    }

    /*添加CRC16*/
    adu[ADU_PDU_SIZE_REGION_OFFSET] = adu_size - ADU_HEAD_SIZE - ADU_PDU_SIZE_REGION_SIZE;
    adu_size = adu_add_crc16(adu,adu_size);
    return adu_size;
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
static int send_adu(int handle,uint8_t *adu,uint8_t size,uint32_t timeout)
{
    uint8_t write_size;

    write_size = serial_write(handle,(char *)adu,size);
    for (int i = 0; i < write_size; i++){
        log_debug("[%2X]\r\n",adu[i]);
    }
    if (size != write_size){
        log_error("controller err in  serial write. expect:%d write:%d.\r\n",size,write_size); 
        return -1;      
     }
  
    return serial_complete(handle,timeout);
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
    case 42:
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

static int get_serial_handle_by_port(scale_contex_t *contex,uint8_t port)
{
    for (uint8_t i = 0;i <contex->cnt;i ++) {
        if (contex->task[i].port == port) {
            return contex->task[i].handle;
        }
    }
    return -1;
}

/*控制器任务通信中断处理*/
void FLEXCOMM0_IRQHandler()
{
    nxp_serial_uart_hal_isr(controller_serial_handle);

}

/*电子秤任务通信中断处理*/
void FLEXCOMM1_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&scale_contex,1);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM2_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&scale_contex,2);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM3_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&scale_contex,3);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}
/*电子秤任务通信中断处理*/
void FLEXCOMM4_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&scale_contex,4);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM5_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&scale_contex,5);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM6_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&scale_contex,6);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}

/*电子秤任务通信中断处理*/
void FLEXCOMM7_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&scale_contex,7);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}
/*电子秤任务通信中断处理*/
void FLEXCOMM8_IRQHandler()
{
    int handle;

    handle = get_serial_handle_by_port(&scale_contex,8);
    if (handle > 0) {
        nxp_serial_uart_hal_isr(handle);
    }

}


/*
* @brief 电子称子任务配置初始化
* @param configration 任务参数指针
* @param host_msg_id 主任务消息队列句柄
* @return 无
* @note
*/
static void controller_scale_contex_init(scale_contex_t *contex)
{
    int rc;
    /*电子秤地址配置信息*/
    scale_addr_configration_t scale_addr;
    rc = controller_read_scale_addr_configration(&scale_addr);
    log_assert(rc == 0);

    contex->cnt = scale_addr.cnt;
    for (uint8_t i = 0;i < contex->cnt;i ++) {
        contex->task[i].addr = scale_addr.value[i];
        contex->task[i].default_addr = CONTROLLER_TASK_SCALE_DEFAULT_ADDR;
        contex->task[i].port = get_serial_port_by_addr(contex->task[i].addr);
        contex->task[i].baud_rates = SCALE_TASK_SERIAL_BAUDRATES;
        contex->task[i].data_bits = SCALE_TASK_SERIAL_DATABITS;
        contex->task[i].stop_bits = SCALE_TASK_SERIAL_STOPBITS;

        rc = serial_create(&contex->task[i].handle,SCALE_TASK_RX_BUFFER_SIZE,SCALE_TASK_RX_BUFFER_SIZE);
        log_assert(rc == 0);
        rc = serial_register_hal_driver(contex->task[i].handle,&nxp_serial_uart_hal_driver);
        log_assert(rc == 0);
 
        rc = serial_open(contex->task[i].handle,
                         contex->task[i].port,
                         contex->task[i].baud_rates,
                         contex->task[i].data_bits,
                         contex->task[i].stop_bits);
        log_assert(rc == 0);
        /*清空接收缓存*/
        serial_flush(contex->task[i].handle);

        /*主消息队列*/
        osMessageQDef(msg_q,1,uint32_t);
        contex->task[i].msg_q_id = osMessageCreate(osMessageQ(msg_q),0);
        log_assert(contex->task[i].msg_q_id);

        /*净重消息队列*/
        osMessageQDef(net_weight_msg_q,1,uint32_t);
        contex->task[i].net_weight_msg_q_id = osMessageCreate(osMessageQ(net_weight_msg_q),0);
        log_assert(contex->task[i].net_weight_msg_q_id);
        /*去皮消息队列*/
        osMessageQDef(remove_tare_weight_msg_q,1,uint32_t);
        contex->task[i].remove_tare_weight_msg_q_id = osMessageCreate(osMessageQ(remove_tare_weight_msg_q),0);
        log_assert(contex->task[i].remove_tare_weight_msg_q_id);
        /*0点校准消息队列*/
        osMessageQDef(calibration_zero_msg_q,1,uint32_t);
        contex->task[i].calibration_zero_msg_q_id = osMessageCreate(osMessageQ(calibration_zero_msg_q),0);
        log_assert(contex->task[i].calibration_zero_msg_q_id);
        /*增益校准消息队列*/
        osMessageQDef(calibration_full_msg_q,1,uint32_t);
        contex->task[i].calibration_full_msg_q_id = osMessageCreate(osMessageQ(calibration_full_msg_q),0);
        log_assert(contex->task[i].calibration_full_msg_q_id);

        /*创建电子秤任务*/
        osThreadDef(scale_task, scale_task, osPriorityNormal, 0, 256);
        contex->task[i].task_hdl = osThreadCreate(osThread(scale_task),&contex->task[i]);
        log_assert(contex->task[i].task_hdl);
        }  
}
       
/*
* @brief 与主机通信任务
* @param argument 任务参数
* @return 无
* @note
*/       
void controller_task(void const * argument)
{
    int rc; 
    uint8_t adu_recv[ADU_SIZE_MAX];
    uint8_t adu_send[ADU_SIZE_MAX];
 
    rc = serial_create(&controller_serial_handle,CONTROLLER_TASK_RX_BUFFER_SIZE,CONTROLLER_TASK_TX_BUFFER_SIZE);
    log_assert(rc == 0);
    rc = serial_register_hal_driver(controller_serial_handle,&nxp_serial_uart_hal_driver);
    log_assert(rc == 0);
 
    rc = serial_open(controller_serial_handle,
                    CONTROLLER_TASK_SERIAL_PORT,
                    CONTROLLER_TASK_SERIAL_BAUDRATES,
                    CONTROLLER_TASK_SERIAL_DATABITS,
                    CONTROLLER_TASK_SERIAL_STOPBITS);
    log_assert(rc == 0); 

    controller_scale_contex_init(&scale_contex);

    log_debug("controller task ok.\r\n");

    /*清空接收缓存*/
    serial_flush(controller_serial_handle);
    while (1) {

        /*接收主机发送的adu*/
        rc = receive_adu(controller_serial_handle,(uint8_t *)adu_recv,ADU_WAIT_TIMEOUT);
        if (rc < 0) {
            /*清空接收缓存*/
            serial_flush(controller_serial_handle);
            continue;
        }
        /*解析处理pdu*/
        rc = parse_pdu((uint8_t *)&adu_recv[ADU_PDU_OFFSET],rc - ADU_HEAD_SIZE - ADU_PDU_SIZE_REGION_SIZE - ADU_CRC_SIZE,adu_send);
        if (rc < 0) {
            continue;
        }
        /*回应主机处理结果*/
        rc = send_adu(controller_serial_handle,adu_send,rc,ADU_SEND_TIMEOUT);
        if (rc < 0) {
            continue;
        }

    }
}
