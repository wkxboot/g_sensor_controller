#include "board.h"
#include "cmsis_os.h"
#include "serial.h"
#include "scale_task.h"
#include "firmware_version.h"
#include "tasks_init.h"
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
    CONTROLLER_TASK_ADU_HEAD_STEP = 0,
    CONTROLLER_TASK_PDU_STEP,
    CONTROLLER_TASK_ADU_CRC_STEP
}controller_step_t;

#define  CONTROLLER_TASK_PDU_VALUE_CNT_MAX            6
#define  CONTROLLER_TASK_PDU_SIZE_MIN                 2      
typedef struct
{
    uint8_t controller_addr;
    uint8_t scale_addr;
    uint8_t code;
    uint8_t value[CONTROLLER_TASK_PDU_VALUE_CNT_MAX];
    uint8_t value_cnt;
}pdu_information_t;


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
* @param buffer 接收缓存
* @param buffer_length 数据缓存长度
* @return CRC
* @note
*/

static uint16_t controller_task_crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    uint32_t i; /* will index into CRC lookup */

   /* calculate the CRC  */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; 
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}
/*
* @brief 计算发送缓存CRC并填充到发送缓存
* @param rsp_buffer 回应缓存
* @param size 当前数据缓存长度
* @return 加上crc后的数据长度
* @note
*/

static uint8_t controller_task_rsp_buffer_add_crc16(uint8_t *rsp_buffer,uint8_t size)
{
    uint16_t crc_calculated;
    crc_calculated = controller_task_crc16(rsp_buffer,size);

    rsp_buffer[size ++] = crc_calculated & 0xff;
    rsp_buffer[size ++] = crc_calculated >> 8;
    return size;
}

/*
* @brief 请求净重值
* @param addr 电子秤地址
* @param net_weight 净重量值指针
* @return -1 失败
* @return  > 0 成功净重量的数据长度
* @note
*/
static int controller_task_request_configration(uint8_t *value)
{
    int rc = -1;
    osStatus   status;
    osEvent    os_event;
    task_msg_t req_msg;
    scale_task_configration_msg_t *rsp_msg;
   
    req_msg.type = REQ_CONFIGRATION;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t *)&req_msg,CONTROLLER_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        return rc;
    }
    
    os_event = osMessageGet(controller_task_cfg_msg_q_id,CONTROLLER_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_event.status == osEventMessage){
        rsp_msg = (scale_task_configration_msg_t *)os_event.value.v;
        rc = rsp_msg->cnt;
        for (uint8_t i = 0;i < rsp_msg->cnt;i ++) {
            value[i] = rsp_msg->scale_addr[i];       
        }
    }
 
    return rc;
}

/*
* @brief 请求净重值
* @param addr 电子秤地址
* @param net_weight 净重量值指针
* @return -1 失败
* @return  > 0 成功净重量的数据长度
* @note
*/
static int controller_task_request_net_weight(const uint8_t addr,int16_t *net_weight)
{
    int rc = -1;
    osStatus   status;
    osEvent    os_event;
    task_msg_t req_msg;
    scale_task_net_weight_msg_t *rsp_msg;
   
    req_msg.type = REQ_NET_WEIGHT;
    req_msg.reserved = addr;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t *)&req_msg,CONTROLLER_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        return rc;
    }
    rc = 0;
    os_event = osMessageGet(controller_task_net_weight_msg_q_id,CONTROLLER_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_event.status == osEventMessage){
        rsp_msg = (scale_task_net_weight_msg_t *)os_event.value.v;
        if (rsp_msg->cnt == 1) {
            *net_weight = rsp_msg->net_weight[rsp_msg->index];
        } else {
            for (uint8_t i = 0;i < rsp_msg->cnt;i ++) {
                net_weight[i] = rsp_msg->net_weight[i];
            }
        }
     rc = rsp_msg->cnt * 2;       
    }
 
    return rc;
}


/*
* @brief 请求去除皮重
* @param addr 电子秤地址
* @return -1 失败
* @return  0 成功
* @note
*/

static int controller_task_request_remove_tar_weight(const uint8_t addr)
{
    osStatus   status;
    osEvent    os_event;
    task_msg_t req_msg;
    scale_task_opt_msg_t *rsp_msg;
    int        rc = -1;
 
    req_msg.type = REQ_REMOVE_TARE_WEIGHT;
    req_msg.reserved = addr;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t *)&req_msg,CONTROLLER_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        return rc;
    }
 
    os_event = osMessageGet(controller_task_remove_tare_weight_msg_q_id,CONTROLLER_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_event.status == osEventMessage){
        rsp_msg = (scale_task_opt_msg_t *)os_event.value.v;
        if (rsp_msg->cnt == 1 ) {
            if (rsp_msg->opt_value[rsp_msg->index] == SCALE_TASK_SUCCESS) {
                rc = 0;
            }
        } else {
            rc = 0;
            for (uint8_t i = 0;i < rsp_msg->cnt;i ++) {
                if (rsp_msg->opt_value[i] == SCALE_TASK_FAILURE) {
                    rc = -1;
                    break;
                }
            }           
        }
    }

    return rc;
}

/*
* @brief 请求0点校准
* @param addr 电子秤地址
* @param weight 校准重力值
* @return -1 失败
* @return  0 成功
* @note
*/
static int controller_task_request_calibration_zero(const uint8_t addr,int16_t weight)
{
    osStatus   status;
    osEvent    os_event;
    task_msg_t req_msg;
    scale_task_opt_msg_t *rsp_msg;
    int        rc = -1;
 
    req_msg.type = REQ_CALIBRATION_ZERO;
    req_msg.value = weight;
    req_msg.reserved = addr;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t *)&req_msg,CONTROLLER_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        return rc;
    }
 
    os_event = osMessageGet(controller_task_calibration_zero_msg_q_id,CONTROLLER_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_event.status == osEventMessage){
        rsp_msg = (scale_task_opt_msg_t *)os_event.value.v;
        if (rsp_msg->cnt == 1 ) {
            if (rsp_msg->opt_value[rsp_msg->index] == SCALE_TASK_SUCCESS) {
                rc = 0;
            }
        } else {
            rc = 0;
            for (uint8_t i = 0;i < rsp_msg->cnt;i ++) {
                if (rsp_msg->opt_value[i] == SCALE_TASK_FAILURE) {
                    rc = -1;
                    break;
                }
            }     
        }
    }
    return rc;
}

/*
* @brief 请求增益校准
* @param addr 电子秤地址
* @param weight 校准重力值
* @return -1 失败
* @return  0 成功
* @note
*/
static int controller_task_request_calibration_full(const uint8_t addr,int16_t weight)
{
    osStatus   status;
    osEvent    os_event;
    task_msg_t req_msg;
    scale_task_opt_msg_t *rsp_msg;
    int        rc = -1;
 
    req_msg.type = REQ_CALIBRATION_FULL;
    req_msg.value = weight;
    req_msg.reserved = addr;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t *)&req_msg,CONTROLLER_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        return rc;
    }
 
    os_event = osMessageGet(controller_task_calibration_full_msg_q_id,CONTROLLER_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_event.status == osEventMessage){
        rsp_msg = (scale_task_opt_msg_t *)os_event.value.v;
        if (rsp_msg->cnt == 1 ) {
            if (rsp_msg->opt_value[rsp_msg->index] == SCALE_TASK_SUCCESS) {
                rc = 0;
            }
        } else {
            rc = 0;
            for (uint8_t i = 0;i < rsp_msg->cnt;i ++) {
                if (rsp_msg->opt_value[i] == SCALE_TASK_FAILURE) {
                    rc = -1;
                    break;
                }
            }     
        }
    }
    return rc;
}

/*
* @brief 请求固件版本
* @param fw_version 固件版本指针
* @return -1 失败
* @return  0 成功
* @note
*/

static int controller_task_request_firmware_version(uint32_t *fw_version)
{
    *fw_version = FIRMWARE_VERSION_HEX;

    return 0;
}

/*
* @brief 串口接收主机ADU
* @param handle 串口句柄
* @param adu adu数据缓存指针
* @param wait_timeout 等待超时时间
* @param frame_timeout 帧超时时间
* @return -1 失败
* @return  0 成功
* @note
*/
static int controller_task_receive_adu(int handle,uint8_t *adu,uint32_t wait_timeout,uint32_t frame_timeout)
{
    int rc;
    int length_to_read,read_length = 0;
    uint32_t timeout;
    uint16_t crc_calculated;
    uint16_t crc_received;
    controller_step_t step;

    timeout = wait_timeout;
    length_to_read = 3;
    read_length = 0;
    step = CONTROLLER_TASK_ADU_HEAD_STEP;
  
    while(length_to_read != 0) {
        rc = serial_select(handle,timeout);
        if (rc == -1) {
            log_error("controller select error.read_len:%d. len_to_read:%d.\r\n",read_length,length_to_read);
            goto exit;
        }
        if (rc == 0) {
            log_error("controller select timeout.read_len:%d. len_to_read:%d.timeout:%d.\r\n",read_length,length_to_read,timeout);
            rc = -1;
            goto exit;
        }
  
        rc = serial_read(handle,(char *)adu + read_length,length_to_read);
        if (rc == -1) {
            log_error("controller read error.read_len:%d. len_to_read:%d.\r\n",adu,length_to_read);
            goto exit;
        }
   
   
        for (int i = 0;i < rc;i++){
            log_debug("<%2X>\r\n", adu[read_length + i]);
        }
   
        read_length += rc;
        length_to_read -= rc;
   
        if (length_to_read == 0) {
            switch(step){
            /*接收到了协议头和数据长度域*/
            case CONTROLLER_TASK_ADU_HEAD_STEP:
                if (adu[CONTROLLER_TASK_ADU_HEAD_OFFSET]     == CONTROLLER_TASK_ADU_HEAD0_VALUE && \
                    adu[CONTROLLER_TASK_ADU_HEAD_OFFSET + 1] == CONTROLLER_TASK_ADU_HEAD1_VALUE) {
                    step = CONTROLLER_TASK_PDU_STEP;      
                    length_to_read = adu[CONTROLLER_TASK_ADU_PDU_LEN_OFFSET];
                    if (length_to_read == 0){
                        log_error("controller err in len value:%d.\r\n",adu[CONTROLLER_TASK_ADU_PDU_LEN_OFFSET]);
                        rc = -1;
                        goto exit;
                    }
                    timeout = frame_timeout;
                } else {
                    log_error("controller err in head value0:%d value1:%d.\r\n",adu[CONTROLLER_TASK_ADU_HEAD_OFFSET],adu[CONTROLLER_TASK_ADU_HEAD_OFFSET + 1]);
                    rc = -1;
                    goto exit;
                } 
                                                                    
                break;
            /*接收完成了PDU的数据*/
            case CONTROLLER_TASK_PDU_STEP:
                step = CONTROLLER_TASK_ADU_CRC_STEP;      
                length_to_read = CONTROLLER_TASK_ADU_CRC_LEN;
            break;
            /*接收完成了全部的数据*/
            case CONTROLLER_TASK_ADU_CRC_STEP:
                crc_calculated = controller_task_crc16(adu,read_length - CONTROLLER_TASK_ADU_CRC_LEN);
                crc_received = adu[read_length - CONTROLLER_TASK_ADU_CRC_LEN] | adu[read_length - CONTROLLER_TASK_ADU_CRC_LEN + 1] << 8;
                if (crc_calculated != crc_received) {
                    log_error("controller err in crc.recv:%d calculate:%d.\r\n",crc_received,crc_calculated);
                    rc = -1;
                    goto exit;
                } else {
                    rc = read_length;
                    goto exit;
                }
            break;
            default:
                log_error("controller internal err.\r\n");
                rc = -1;
                goto exit;
            }
        }
    }
exit:
    return rc;
}

/*
* @brief 构建回应ADU
* @param adu adu缓存指针
* @param pdu_info 请求的pdu详细信息指针
* @param value 处理结果数据指针
* @param cnt 处理结果数据大小
* @return adu长度
* @note
*/
static uint8_t controller_task_build_rsp_adu(uint8_t *adu,const uint8_t *value,const uint8_t cnt)
{
    uint8_t adu_length = 0;
    /*ADU head*/
    adu[adu_length ++] = CONTROLLER_TASK_ADU_HEAD0_VALUE;
    adu[adu_length ++] = CONTROLLER_TASK_ADU_HEAD1_VALUE;
    /*PDU len*/
    adu[adu_length ++] = cnt;
    /*PDU*/
    for (uint8_t i = 0;i < cnt; i ++) {
        adu[adu_length ++] = value[i];
    }
    /*CRC16*/
    adu_length = controller_task_rsp_buffer_add_crc16(adu,adu_length);

    return adu_length;
}



/*
* @brief 检测主机请求码
* @param code 请求码
* @param value_cnt 请求码附带参数
* @return -1 失败
* @return  0 成功
* @note
*/
static int controller_task_pdu_code_check(const uint8_t code,const uint8_t value_cnt)
{
    if (code > CONTROLLER_TASK_PDU_CODE_MAX) {
        log_error("pdu code:%d unknow err.\r\n",code);
        return -1;
    }

    if (code == CONTROLLER_TASK_PDU_CODE_CONFIGRATION && value_cnt != 0){
        log_error("pdu configration value cnt:%d != 0 err.\r\n",value_cnt);
        return -1;
    }else if (code == CONTROLLER_TASK_PDU_CODE_NET_WEIGHT && value_cnt != 0) {
        log_error("pdu net weight value cnt:%d != 0 err.\r\n",value_cnt);
        return -1;
    }else if (code == CONTROLLER_TASK_PDU_CODE_REMOVE_TAR_WEIGHT && value_cnt != 0) {
        log_error("pdu remove tar weight value cnt:%d != 0 err.\r\n",value_cnt);
        return -1;
    }else if (code == CONTROLLER_TASK_PDU_CODE_CALIBRATION_ZERO && value_cnt != 2) {
        log_error("pdu calibration zero value cnt:%d != 2 err.\r\n",value_cnt);
        return -1;
    }else if (code == CONTROLLER_TASK_PDU_CODE_CALIBRATION_FULL && value_cnt != 2) {
        log_error("pdu calibration full value cnt:%d != 2 err.\r\n",value_cnt);
        return -1;
    }

    return 0;
}
/*
* @brief 解析接收的PDU
* @param pdu pdu缓存指针
* @param size pdu大小
* @param pdu_info pdu详细信息
* @return 
* @note
*/
static int controller_task_parse_pdu(const uint8_t *pdu,uint8_t size,pdu_information_t *pdu_info)
{
    int rc;

    if (size < CONTROLLER_TASK_PDU_SIZE_MIN) {
        log_error("pdu size:%d < %d err.\r\n",size,CONTROLLER_TASK_PDU_SIZE_MIN);
        return -1;
    }
    pdu_info->controller_addr = pdu[CONTROLLER_TASK_PDU_CONTROLLER_ADDR_OFFSET];
    if (pdu_info->controller_addr != CONTROLLER_TASK_CONTROLLER_ADDR) {
        log_error("pdu controller addr:%d != %d err.\r\n",pdu_info->controller_addr,CONTROLLER_TASK_CONTROLLER_ADDR);
        return -1;
    }
    pdu_info->code = pdu[CONTROLLER_TASK_PDU_CODE_OFFSET];
    if (pdu_info->code > CONTROLLER_TASK_PDU_CODE_MAX) {
        log_error("pdu code:%d > %d err.\r\n",pdu_info->code,CONTROLLER_TASK_PDU_CODE_MAX);
        return -1;
    }
    if (pdu_info->code == CONTROLLER_TASK_PDU_CODE_CONFIGRATION) {
        pdu_info->value_cnt = size - 2;
    } else {
        pdu_info->scale_addr = pdu[CONTROLLER_TASK_PDU_SCALE_ADDR_OFFSET];
        pdu_info->value_cnt = size - 3;
    }
    rc = controller_task_pdu_code_check(pdu_info->code,pdu_info->value_cnt);
    if (rc != 0) {
        log_error("pdu code check err.\r\n");
        return -1; 
    }

    for (uint8_t i = 0;i < pdu_info->value_cnt;i ++) {
        pdu_info->value[i] = pdu[CONTROLLER_TASK_PDU_VALUE_OFFSET + i];
    }
    return 0;
}
/*
* @brief 处理解析后的pdu信息
* @param pdu_info pdu详细信息指针
* @param value 处理后的结果缓存
* @param value_cnt 结果缓存大小
* @return -1 失败 
* @return  0 成功
* @note
*/

static int controller_task_process_pdu_info(pdu_information_t *pdu_info,uint8_t *value,uint8_t *value_cnt)
{
    int rc;

    int16_t calibration_weight;
    value[0] = CONTROLLER_TASK_CONTROLLER_ADDR;
    value[1] = pdu_info->code;
    switch (pdu_info->code) {
        case CONTROLLER_TASK_PDU_CODE_CONFIGRATION:    
        rc = controller_task_request_configration(&value[2]);
        if (rc <= 0) {
            log_error("req configration err.\r\n");
            goto err_exit;
        } 
        *value_cnt = rc + 2;
        break;
    case CONTROLLER_TASK_PDU_CODE_NET_WEIGHT:
        value[2] = pdu_info->scale_addr;
        rc = controller_task_request_net_weight(pdu_info->scale_addr,(int16_t *)&value[3]);
        if (rc <= 0) {
            log_error("req net weight err.\r\n");
            goto err_exit;
        } 
        *value_cnt = rc + 3;
        break;
     case CONTROLLER_TASK_PDU_CODE_REMOVE_TAR_WEIGHT:
        rc = controller_task_request_remove_tar_weight(pdu_info->scale_addr);

        if (rc == 0) {
            value[2] = CONTROLLER_TASK_SUCCESS_VALUE;
        } else {
            log_error("req remove tar weight fail.\r\n");
            value[2] = CONTROLLER_TASK_FAILURE_VALUE;
        }
        *value_cnt = 3;
        break;   
     case CONTROLLER_TASK_PDU_CODE_CALIBRATION_ZERO:
        calibration_weight = pdu_info->value[1] << 8 | pdu_info->value[0];
        rc = controller_task_request_calibration_zero(pdu_info->scale_addr,calibration_weight);
        if (rc == 0) {
            value[2] = CONTROLLER_TASK_SUCCESS_VALUE;
        } else {
            log_error("req calibration zero fail.\r\n");
            value[2] = CONTROLLER_TASK_FAILURE_VALUE;
        }
        *value_cnt = 3;
        break;  
     case CONTROLLER_TASK_PDU_CODE_CALIBRATION_FULL:
        calibration_weight = pdu_info->value[1] << 8 | pdu_info->value[0];
        rc = controller_task_request_calibration_full(pdu_info->scale_addr,calibration_weight);
        if (rc == 0) {
            value[2] = CONTROLLER_TASK_SUCCESS_VALUE;
        } else {
            log_error("req calibration full fail.\r\n");
            value[2] = CONTROLLER_TASK_FAILURE_VALUE;
        }
        *value_cnt = 3;
        break;  
     case CONTROLLER_TASK_PDU_CODE_FIRMWARE_VERSION:
        rc = controller_task_request_firmware_version((uint32_t *)&value[2]);
        if (rc != 0) {
            log_error("req firmware version err.\r\n");
            goto err_exit;
        }
        *value_cnt = 5;
        break; 
    default:
        log_error("controller internal err.code:%d.\r\n",pdu_info->code);
        goto err_exit;
    }
    return 0;

err_exit:
    return -1;
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
static int controller_task_send_rsp_adu(int handle,uint8_t *adu,uint8_t size,uint32_t timeout)
{
    uint8_t write_length,remain_length;

    write_length = serial_write(handle,(char *)adu,size);
    for (int i = 0; i < write_length; i++){
        log_debug("[%2X]\r\n",adu[i]);
    }
    if (size != write_length){
        log_error("controller err in  serial buffer write. expect:%d write:%d.\r\n",size,write_length); 
        return -1;      
     }
    
    remain_length = serial_complete(handle,timeout);
    if (remain_length != 0){
       log_error("controller err in  serial send timeout.\r\n",); 
       return -1;  
    }

    return 0;
}

/*中断处理*/
void FLEXCOMM3_IRQHandler()
{
    nxp_serial_uart_hal_isr(controller_serial_handle);

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
    uint8_t adu_size;
    uint8_t recv_adu[CONTROLLER_TASK_FRAME_SIZE_MAX];
    uint8_t rsp_adu[CONTROLLER_TASK_FRAME_SIZE_MAX];
    pdu_information_t pdu_info;
    uint8_t value[CONTROLLER_TASK_PDU_VALUE_CNT_MAX];
    uint8_t value_cnt;
 
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
 
    log_debug("controller task sync ok.\r\n");
    while (1) {
        /*清空接收缓存*/
        serial_flush(controller_serial_handle);
        /*接收主机发送的adu*/
        rc = controller_task_receive_adu(controller_serial_handle,(uint8_t *)recv_adu,CONTROLLER_TASK_WAIT_TIMEOUT_VALUE,CONTROLLER_TASK_FRAME_TIMEOUT_VALUE);
        if (rc <= 0) {
            continue;
        }
        /*解析pdu*/
        rc = controller_task_parse_pdu((uint8_t *)&recv_adu[CONTROLLER_TASK_ADU_PDU_OFFSET],rc - CONTROLLER_TASK_ADU_HEAD_LEN - CONTROLLER_TASK_ADU_PDU_LEN_LEN - CONTROLLER_TASK_ADU_CRC_LEN,&pdu_info);
        if (rc < 0) {
            continue;
        }
        /*处理pdu信息*/
        rc = controller_task_process_pdu_info(&pdu_info,value,&value_cnt);
        if ( rc < 0) {  
            continue;
        }
        /*构建处理结果的回应*/
        adu_size = controller_task_build_rsp_adu(rsp_adu,value,value_cnt);
        /*回应主机处理结果*/
        rc = controller_task_send_rsp_adu(controller_serial_handle,rsp_adu,adu_size,CONTROLLER_TASK_SEND_TIMEOUT);
        if (rc < 0) {
            continue;
        }

    }
}
