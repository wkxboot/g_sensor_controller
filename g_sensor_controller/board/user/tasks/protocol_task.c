#include "board.h"
#include "cmsis_os.h"
#include "serial.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "protocol_task.h"
#include "log.h"

extern int protocol_serial_handle;
extern serial_hal_driver_t nxp_serial_uart_hal_driver;

osThreadId   protocol_task_hdl;
osMessageQId protocol_task_msg_q_id;

typedef enum
{
    PROTOCOL_TASK_ADU_HEAD_STEP = 0,
    PROTOCOL_TASK_PDU_STEP,
    PROTOCOL_TASK_ADU_CRC_STEP
}protocol_step_t;


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

static uint16_t protocol_task_crc16(uint8_t *buffer, uint16_t buffer_length)
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

static uint8_t protocol_task_rsp_buffer_add_crc16(uint8_t *rsp_buffer,uint8_t size)
{
    uint16_t crc_calculated;
    crc_calculated = protocol_task_crc16(rsp_buffer,size);

    rsp_buffer[size ++] =  crc_calculated >> 8;
    rsp_buffer[size ++] =  crc_calculated & 0xff;
    return size;
}
/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int protocol_task_request_net_weight(const uint8_t addr,int16_t *net_weight)
{
    osStatus   status;
    osEvent    os_msg;
    task_msg_t req_msg,rsp_msg;
    int        rc = -1;
 
    req_msg.type = REQ_NET_WEIGHT;
    req_msg.value = addr;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t *)&req_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        return rc;
    }
 
    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
        rsp_msg = *(task_msg_t *)&os_msg.value.v;
        if (rsp_msg.type == RSP_NET_WEIGHT && rsp_msg.value == SCALE_TASK_SUCCESS) { 
            rc = copy_net_weight(addr,net_weight);        
        }
            
    }
 
    return rc;
}


/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int protocol_task_request_remove_tar_weight(const uint8_t addr)
{
    osStatus   status;
    osEvent    os_msg;
    task_msg_t req_msg,rsp_msg;
    int        rc = -1;
 
    req_msg.type = REQ_REMOVE_TAR_WEIGHT;
    req_msg.value = addr;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t *)&req_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        return rc;
    }
 
    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
        rsp_msg = *(task_msg_t *)&os_msg.value.v;
        if (rsp_msg.type == RSP_REMOVE_TAR_WEIGHT && rsp_msg.value == SCALE_TASK_SUCCESS) {             
            rc = 0;                           
        }
    }
    return rc;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int protocol_task_request_calibration_zero(const uint8_t addr)
{
    osStatus   status;
    osEvent    os_msg;
    task_msg_t req_msg,rsp_msg;
    int        rc = -1;
 
    req_msg.type = REQ_CALIBRATION_ZERO;
    req_msg.value = 0;
    req_msg.reserved = addr;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t *)&req_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        return rc;
    }
 
    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
        rsp_msg = *(task_msg_t *)&os_msg.value.v;
        if (rsp_msg.type == RSP_CALIBRATION_ZERO && rsp_msg.value == SCALE_TASK_SUCCESS) {             
            rc = 0;                           
        }
    }
    return rc;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int protocol_task_request_calibration_full(const uint8_t addr,int16_t weight)
{
    osStatus   status;
    osEvent    os_msg;
    task_msg_t req_msg,rsp_msg;
    int        rc = -1;
 
    req_msg.type = REQ_CALIBRATION_FULL;
    req_msg.value = weight;
    req_msg.reserved = addr;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t *)&req_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        return rc;
    }
 
    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
        rsp_msg = *(task_msg_t *)&os_msg.value.v;
        if (rsp_msg.type == RSP_CALIBRATION_FULL && rsp_msg.value == SCALE_TASK_SUCCESS) {             
            rc = 0;                           
        }
    }
    return rc;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int protocol_task_request_firmware_version(uint32_t *fw_version)
{
    osStatus   status;
    osEvent    os_msg;
    task_msg_t req_msg,rsp_msg;
    int        rc = -1;
 
    req_msg.type = REQ_FIRMWARE_VERSION;
    status = osMessagePut(scale_task_msg_q_id,*(uint32_t *)&req_msg,PROTOCOL_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        return rc;
    }
 
    os_msg = osMessageGet(protocol_task_msg_q_id,PROTOCOL_TASK_MSG_WAIT_TIMEOUT_VALUE);
    if (os_msg.status == osEventMessage){
        rsp_msg = *(task_msg_t *)&os_msg.value.v;
        if (rsp_msg.type == RSP_FIRMWARE_VERSION && rsp_msg.value == SCALE_TASK_SUCCESS) {         
            *fw_version = (uint32_t)rsp_msg.value << 8 | rsp_msg.reserved;
            rc = 0;                           
        }
    }
    return rc;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int protocol_task_receive_adu(uint8_t *adu,uint32_t wait_timeout,uint32_t frame_timeout)
{
    int rc;
    int length_to_read,read_length = 0;
    uint32_t timeout;
    uint16_t crc_calculated;
    uint16_t crc_received;
    protocol_step_t step;

    timeout = wait_timeout;
    length_to_read = 2;
    read_length = 0;
    step = PROTOCOL_TASK_ADU_HEAD_STEP;
  
    while(length_to_read != 0) {
        rc = serial_select(protocol_serial_handle,timeout);
        if (rc == -1) {
            log_error("protocol select error.read_len:%d. len_to_read:%d.\r\n",read_length,length_to_read);
            goto exit;
        }
        if (rc == 0) {
            log_error("protocol select timeout.read_len:%d. len_to_read:%d.timeout:%d.\r\n",read_length,length_to_read,timeout);
            goto exit;
        }
  
        rc = serial_read(protocol_serial_handle,(char *)adu + read_length,length_to_read);
        if (rc == -1) {
            log_error("protocol read error.read_len:%d. len_to_read:%d.\r\n",adu,length_to_read);
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
            case PROTOCOL_TASK_ADU_HEAD_STEP:
                if (adu[PROTOCOL_TASK_ADU_HEAD_OFFSET]     == PROTOCOL_TASK_ADU_HEAD0_VALUE && \
                    adu[PROTOCOL_TASK_ADU_HEAD_OFFSET + 1] == PROTOCOL_TASK_ADU_HEAD1_VALUE) {
                    step = PROTOCOL_TASK_PDU_STEP;      
                    length_to_read = adu[PROTOCOL_TASK_ADU_PDU_LEN_OFFSET];
                    if (length_to_read == 0){
                        log_error("protocol err in len value:%d.\r\n",adu[PROTOCOL_TASK_ADU_PDU_LEN_OFFSET]);
                        rc = -1;
                        goto exit;
                    }
                    timeout = frame_timeout;
                } else {
                    log_error("protocol err in head value0:%d value1:%d.\r\n",adu[PROTOCOL_TASK_ADU_HEAD_OFFSET],adu[PROTOCOL_TASK_ADU_HEAD_OFFSET + 1]);
                    rc = -1;
                    goto exit;
                } 
                                                                    
                break;
            /*接收完成了PDU的数据*/
            case PROTOCOL_TASK_PDU_STEP:
                step = PROTOCOL_TASK_ADU_CRC_STEP;      
                length_to_read = PROTOCOL_TASK_ADU_CRC_LEN;
            break;
            /*接收完成了全部的数据*/
            case PROTOCOL_TASK_ADU_CRC_STEP:
                crc_calculated = protocol_task_crc16(adu,read_length - PROTOCOL_TASK_ADU_CRC_LEN);
                crc_received = adu[read_length - PROTOCOL_TASK_ADU_CRC_LEN]<< 8 | adu[read_length - PROTOCOL_TASK_ADU_CRC_LEN - 1];
                if (crc_calculated != crc_received) {
                    log_error("protocol err in crc.recv:%d calculate:%d.\r\n",crc_received,crc_calculated);
                    rc = -1;
                    goto exit;
                } else {
                    rc = read_length;
                    goto exit;
                }
            break;
            default:
                log_error("protocol internal err.\r\n");
                rc = -1;
                goto exit;
            }
        }
    }
exit:
    return rc;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static uint8_t protocol_task_build_rsp_adu(uint8_t *adu,uint8_t controller_addr,uint8_t code,uint8_t scale_addr,uint8_t *value,uint8_t cnt)
{
    uint8_t adu_length = 0;
    /*ADU head*/
    adu[adu_length ++] = PROTOCOL_TASK_ADU_HEAD0_VALUE;
    adu[adu_length ++] = PROTOCOL_TASK_ADU_HEAD1_VALUE;
    /*ADU addr*/
    adu[adu_length ++] = controller_addr;
    /*PDU len*/
    adu[adu_length ++] = cnt + 1 + 1;/*pdu code + pdu scale_addr + cnt*/
    /*PDU code*/
    adu[adu_length ++] = code;
    /*PDU value scale addr*/
    adu[adu_length ++] = scale_addr;
    /*PDU value result*/
    for (uint8_t i = 0;i < cnt;i ++) {
         adu[adu_length ++] = value[i];
    }
    adu_length = protocol_task_rsp_buffer_add_crc16(adu,adu_length);

    return adu_length;
  
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int protocol_task_parse_and_process_pdu(const uint8_t addr,const uint8_t *pdu,uint8_t pdu_size,uint8_t *adu)
{
    int rc;
    uint8_t adu_size;
    uint8_t code;
    uint8_t controller_addr,scale_addr;
    int16_t calibration_weight;
    uint8_t value[PROTOCOL_TASK_SCALE_CNT_MAX * 2];
    uint8_t value_cnt;

    if (pdu_size < 3) {
        log_error("pdu size:%d < 3 err.\r\n",pdu_size);
        goto err_exit;
    }

    controller_addr = pdu[PROTOCOL_TASK_PDU_CONTROLLER_ADDR_OFFSET];
    if (controller_addr != addr) {
        log_error("pdu controller addr:%d != %d err.\r\n",controller_addr,addr);
        goto err_exit;
    }
    code = pdu[PROTOCOL_TASK_PDU_CODE_OFFSET];
    if (code > PROTOCOL_TASK_PDU_CODE_MAX) {
        log_error("pdu code:%d > %d err.\r\n",code,PROTOCOL_TASK_PDU_CODE_MAX);
        goto err_exit;
    }
    scale_addr = pdu[PROTOCOL_TASK_PDU_SCALE_ADDR_OFFSET];

    pdu_size -= 3;
    
    switch (code) {
    case PROTOCOL_TASK_PDU_CODE_NET_WEIGHT:
        if (pdu_size != 0) {
            log_error("pdu net weight value size:%d != 0 err.\r\n",pdu_size);
            goto err_exit;
        }
        rc = protocol_task_request_net_weight(scale_addr,(int16_t *)value);
        if (rc != 0) {
            log_error("req net weight err.\r\n");
            goto err_exit;
        }    
        value_cnt = rc;
        break;
     case PROTOCOL_TASK_PDU_CODE_REMOVE_TAR_WEIGHT:
        if (pdu_size != 0) {
            log_error("pdu remove tar weight value size:%d != 0 err.\r\n",pdu_size);
            goto err_exit;
        }
        rc = protocol_task_request_remove_tar_weight(scale_addr);
        value_cnt = 1;
        if (rc == 0) {
            value[0] = PROTOCOL_TASK_SUCCESS_VALUE;
        } else {
            log_error("req remove tar weight fail.\r\n");
            value[0] = PROTOCOL_TASK_FAILURE_VALUE;
        }
        break;   
     case PROTOCOL_TASK_PDU_CODE_CALIBRATION_ZERO:
        if (pdu_size != 2) {
            log_error("pdu calibration zero value size:%d != 2 err.\r\n",pdu_size);
            goto err_exit;
        }
        calibration_weight = pdu[PROTOCOL_TASK_PDU_VALUE_OFFSET + 1] << 8 | pdu[PROTOCOL_TASK_PDU_VALUE_OFFSET];
        if (calibration_weight != 0 ) {
            log_error("pdu calibration zero value:%d not 0 err.\r\n",calibration_weight);
            goto err_exit;
        }
        value_cnt = 1;
        rc = protocol_task_request_calibration_zero(scale_addr);
        if (rc == 0) {
            value[0] = PROTOCOL_TASK_SUCCESS_VALUE;
        } else {
            log_error("req calibration zero fail.\r\n");
            value[0] = PROTOCOL_TASK_FAILURE_VALUE;
        }
        break;  
     case PROTOCOL_TASK_PDU_CODE_CALIBRATION_FULL:
        if (pdu_size != 2) {
            log_error("pdu calibration full value size:%d != 2 err.\r\n",pdu_size);
            goto err_exit;
        }
        calibration_weight = pdu[PROTOCOL_TASK_PDU_VALUE_OFFSET + 1] << 8 | pdu[PROTOCOL_TASK_PDU_VALUE_OFFSET];
        if (calibration_weight == 0 ) {
            log_error("pdu calibration full value:%d == 0 err.\r\n",calibration_weight);
            goto err_exit;
        }
        value_cnt = 1;
        rc = protocol_task_request_calibration_full(scale_addr,calibration_weight);
        if (rc == 0) {
            value[0] = PROTOCOL_TASK_SUCCESS_VALUE;
        } else {
            log_error("req calibration zero fail.\r\n");
            value[0] = PROTOCOL_TASK_FAILURE_VALUE;
        }
        break;  
     case PROTOCOL_TASK_PDU_CODE_FIRMWARE_VERSION:
        if (pdu_size != 0) {
            log_error("pdu calibration full value size:%d != 0 err.\r\n",pdu_size);
            goto err_exit;
        }
        value_cnt = 3;
        rc = protocol_task_request_firmware_version((uint32_t *)value);
        if (rc != 0) {
            log_error("req firmware version err.\r\n");
            goto err_exit;
        }
        break; 
    default:
        log_error("protocol internal err.code:%d.\r\n",code);
        goto err_exit;
    }
    adu_size = protocol_task_build_rsp_adu(adu,controller_addr,code,scale_addr,value,value_cnt);
    return adu_size;

err_exit:
    return -1;
}

       
/*
* @brief 
* @param
* @param
* @return 
* @note
*/       
void protocol_task(void const * argument)
{
    int rc; 
    int length_to_write,write_length,remain_length;
    char recv_adu[PROTOCOL_TASK_FRAME_SIZE_MAX];
    char rsp_adu[PROTOCOL_TASK_FRAME_SIZE_MAX];
   
    
    osMessageQDef(protocol_task_msg_q,1,uint32_t);
    protocol_task_msg_q_id = osMessageCreate(osMessageQ(protocol_task_msg_q),protocol_task_hdl);
    log_assert(protocol_task_msg_q_id); 
 
    rc = serial_create(&protocol_serial_handle,PROTOCOL_TASK_RX_BUFFER_SIZE,PROTOCOL_TASK_TX_BUFFER_SIZE);
    log_assert(rc == 0);
    rc = serial_register_hal_driver(protocol_serial_handle,&nxp_serial_uart_hal_driver);
    log_assert(rc == 0);
 
    rc = serial_open(protocol_serial_handle,
                    PROTOCOL_TASK_SERIAL_PORT,
                    PROTOCOL_TASK_SERIAL_BAUDRATES,
                    PROTOCOL_TASK_SERIAL_DATABITS,
                    PROTOCOL_TASK_SERIAL_STOPBITS);
    log_assert(rc == 0); 
 
    /*等待任务同步*/
    xEventGroupSync(tasks_sync_evt_group_hdl,TASKS_SYNC_EVENT_PROTOCOL_TASK_RDY,TASKS_SYNC_EVENT_ALL_TASKS_RDY,osWaitForever);
    log_debug("protocol task sync ok.\r\n");
    while (1) {
        serial_flush(protocol_serial_handle);
        rc = protocol_task_receive_adu((uint8_t *)recv_adu,PROTOCOL_TASK_WAIT_TIMEOUT_VALUE,PROTOCOL_TASK_FRAME_TIMEOUT_VALUE);
        if (rc < 4) {
            continue;
        }
        rc = protocol_task_parse_and_process_pdu(PROTOCOL_TASK_CONTROLLER_ADDR,(uint8_t *)&recv_adu[PROTOCOL_TASK_ADU_PDU_OFFSET],rc - PROTOCOL_TASK_ADU_HEAD_LEN - PROTOCOL_TASK_ADU_CRC_LEN,(uint8_t *)rsp_adu);
        if (rc < 0) {
            continue;
        }
        /*解析完毕 回应操作结果*/  
        write_length = serial_write(protocol_serial_handle,rsp_adu,rc);
        for (int i=0; i < write_length; i++){
            log_debug("[%2X]\r\n", rsp_adu[i]);
        }
        if (write_length != length_to_write){
            log_error("protocol err in  serial buffer write. expect:%d write:%d.\r\n",length_to_write,write_length); 
            continue;      
        }
    
        remain_length = serial_complete(protocol_serial_handle,PROTOCOL_TASK_SEND_TIMEOUT);
        if (remain_length != 0){
            log_error("protocol err in  serial send timeout.\r\n",); 
            continue;  
        }

    }
}
