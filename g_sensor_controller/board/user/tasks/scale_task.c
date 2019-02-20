#include "board.h"
#include "cmsis_os.h"
#include "serial.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "scale_task.h"
#include "log.h"

extern int scale_serial_handle;
extern serial_hal_driver_t scale_serial_driver;

osThreadId   scale_task_hdl;
osMessageQId scale_task_msg_q_id;

typedef enum
{
    SCALE_TASK_ADU_HEAD_STEP = 0,
    SCALE_TASK_ADU_PDU_STEP,
    SCALE_TASK_ADU_CRC_STEP
}scale_step_t;


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

static uint16_t scale_task_crc16(uint8_t *buffer, uint16_t buffer_length)
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

static uint8_t scale_task_rsp_adu_add_crc16(uint8_t *adu,uint8_t size)
{
    uint16_t crc_calculated;
    crc_calculated = scale_task_crc16(adu,size);

    adu[size ++] =  crc_calculated >> 8;
    adu[size ++] =  crc_calculated & 0xff;
    return size;
}
/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int scale_task_build_adu(uint8_t *adu,uint8_t addr,uint8_t code,uint8_t *value,uint8_t cnt)
{
    uint8_t adu_size = 0;
    /*ADU head*/
    adu[adu_size ++] = SCALE_TASK_ADU_HEAD0_VALUE;
    adu[adu_size ++] = SCALE_TASK_ADU_HEAD1_VALUE;
    /*PDU len*/
    adu[adu_size ++] = cnt + 1 + 1;/*pdu code + pdu scale_addr + cnt*/
    /*ADU addr*/
    adu[adu_size ++] = addr;
    /*PDU code*/
    adu[adu_size ++] = code;
    /*PDU value result*/
    for (uint8_t i = 0;i < cnt;i ++) {
         adu[adu_size ++] = value[i];
    }
    adu_size = scale_task_rsp_adu_add_crc16(adu,adu_size);

    return adu_size;
}


/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int scale_task_req(const uint8_t *adu,const uint8_t len,const uint16_t timeout)
{
    uint16_t length_to_write,write_length,remain_length;
 
    serial_flush(scale_serial_handle);
    length_to_write = len;
    write_length = serial_write(scale_serial_handle,(const char*)adu,length_to_write);
    for (int i=0; i < write_length; i++){
        log_debug("[%2X]\r\n", adu[i]);
    }
    if (write_length != length_to_write){
        log_error("scale err in  serial buffer write. expect:%d write:%d.\r\n",length_to_write,write_length); 
        return -1;    
    } 
    remain_length = serial_complete(scale_serial_handle,timeout);
    if (remain_length != 0) {
        log_error("scale err in  serial send timeout.\r\n"); 
        return -1;
    }
   
    return 0;
}

static int scale_task_wait_rsp(uint8_t *adu,uint32_t timeout)
{
    int rc;
    int length_to_read,read_length = 0;
    uint16_t crc_calculated;
    uint16_t crc_received;
    scale_step_t step;

    length_to_read = 2;
    read_length = 0;
    step = SCALE_TASK_ADU_HEAD_STEP;
  
    while(length_to_read != 0) {
        rc = serial_select(scale_serial_handle,timeout);
        if (rc == -1) {
            log_error("scale select error.read_len:%d. len_to_read:%d.\r\n",read_length,length_to_read);
            goto exit;
        }
        if (rc == 0) {
            log_error("scale select timeout.read_len:%d. len_to_read:%d.timeout:%d.\r\n",read_length,length_to_read,timeout);
            goto exit;
        }
  
        rc = serial_read(scale_serial_handle,(char *)adu + read_length,length_to_read);
        if (rc == -1) {
            log_error("scale read error.read_len:%d. len_to_read:%d.\r\n",adu,length_to_read);
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
            case SCALE_TASK_ADU_HEAD_STEP:
                if (adu[SCALE_TASK_ADU_HEAD_OFFSET]     == SCALE_TASK_ADU_HEAD0_VALUE && \
                    adu[SCALE_TASK_ADU_HEAD_OFFSET + 1] == SCALE_TASK_ADU_HEAD1_VALUE) {
                    step = SCALE_TASK_ADU_PDU_STEP;      
                    length_to_read = adu[SCALE_TASK_ADU_PDU_LEN_OFFSET];
                    if (length_to_read == 0){
                        log_error("scale err in len value:%d.\r\n",adu[SCALE_TASK_ADU_PDU_LEN_OFFSET]);
                        rc = -1;
                        goto exit;
                    }
                    timeout = SCALE_TASK_FRAME_TIMEOUT_VALUE;
                } else {
                    log_error("scale err in head value0:%d value1:%d.\r\n",adu[SCALE_TASK_ADU_HEAD_OFFSET],adu[SCALE_TASK_ADU_HEAD_OFFSET + 1]);
                    rc = -1;
                    goto exit;
                } 
                                                                    
                break;
            /*接收完成了PDU的数据*/
            case SCALE_TASK_ADU_PDU_STEP:
                step = SCALE_TASK_ADU_CRC_STEP;      
                length_to_read = SCALE_TASK_ADU_CRC_LEN;
            break;
            /*接收完成了全部的数据*/
            case SCALE_TASK_ADU_CRC_STEP:
                crc_calculated = scale_task_crc16(adu,read_length - SCALE_TASK_ADU_CRC_LEN);
                crc_received = adu[read_length - SCALE_TASK_ADU_CRC_LEN]<< 8 | adu[read_length - SCALE_TASK_ADU_CRC_LEN - 1];
                if (crc_calculated != crc_received) {
                    log_error("scale err in crc.recv:%d calculate:%d.\r\n",crc_received,crc_calculated);
                    rc = -1;
                    goto exit;
                } else {
                    rc = read_length;
                    goto exit;
                }
            break;
            default:
                log_error("scale internal err.\r\n");
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
static int scale_task_parse_pdu(const uint8_t addr,const uint8_t code,const uint8_t *pdu,uint8_t pdu_size,uint8_t *value,uint8_t *value_cnt)
{
    uint8_t rsp_code;
    uint8_t rsp_addr;

    if (pdu_size < 3) {
        log_error("pdu size:%d < 3 err.\r\n",pdu_size);
        goto err_exit;
    }

    rsp_addr = pdu[SCALE_TASK_PDU_SCALE_ADDR_OFFSET];
    if (rsp_addr != addr) {
        log_error("pdu rsp_addr addr:%d != %d err.\r\n",rsp_addr,addr);
        goto err_exit;
    }
    rsp_code = pdu[SCALE_TASK_PDU_CODE_OFFSET];
    if (rsp_code != code) {
        log_error("pdu rsp code:%d != %d err.\r\n",rsp_code,code);
        goto err_exit;
    }

    pdu_size -= 2;
    
    if (pdu_size > 3) {
        log_error("pdu rsp value size:%d > 2 err.\r\n",pdu_size);
        goto err_exit;
    }
    for (uint8_t i = 0;i < pdu_size;i ++) {
        value[i] = pdu[SCALE_TASK_PDU_VALUE_OFFSET + i];
    }
    *value_cnt = pdu_size;

    
    return 0;

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
typedef struct
{
    uint8_t addr;
    uint8_t port;
    uint32_t baud_rates;
    uint8_t data_bits;
    uint8_t stop_bits;
    int handle;
    osThreadId task_id;
    osMessageQId msg_id;
    osMessageQId host_msg_id;
    bool done;
}scale_task_information_t;

    
/*
* @brief 电子称子任务
* @param argument 任务参数
* @return 无
* @note
*/

void scale_task(void const *argument)
{
    osStatus   status;
    osEvent    os_msg;
    task_msg_t req_msg,rsp_msg;
    int rc; 
    uint8_t adu_size;
    uint8_t req_adu[SCALE_TASK_FRAME_SIZE_MAX];
    uint8_t rsp_adu[SCALE_TASK_FRAME_SIZE_MAX];
    uint8_t req_value[2];
    uint8_t rsp_value[2];
    uint8_t rsp_value_cnt;
 
    osMessageQDef(scale_task_msg_q,1,uint32_t);
    ((scale_task_information_t*)argument)->msg_id = osMessageCreate(osMessageQ(scale_task_msg_q),0);
    log_assert(((scale_task_information_t*)argument)->msg_id);

    rc = serial_create(&((scale_task_information_t*)argument)->handle,SCALE_TASK_RX_BUFFER_SIZE,SCALE_TASK_TX_BUFFER_SIZE);
    log_assert(rc == 0);
    rc = serial_register_hal_driver(((scale_task_information_t*)argument)->handle,&scale_serial_driver);
    log_assert(rc == 0);
 
    rc = serial_open(((scale_task_information_t*)argument)->handle,
                    ((scale_task_information_t*)argument)->port,
                    ((scale_task_information_t*)argument)->baud_rates,
                    ((scale_task_information_t*)argument)->data_bits,
                    ((scale_task_information_t*)argument)->stop_bits);
 
    log_assert(rc == 0); 
 
    while (1) {
        os_msg = osMessageGet(((scale_task_information_t*)argument)->msg_id,SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE);
        if (os_msg.status == osEventMessage) {
            req_msg = *(task_msg_t *)&os_msg.value.v;
 
        /*获取净重值*/
        if (req_msg.type == REQ_NET_WEIGHT) { 
            rsp_msg.type = RSP_NET_WEIGHT;

            adu_size = scale_task_build_adu(req_adu,rsp_msg.reserved,SCALE_TASK_PDU_CODE_NET_WEIGHT,req_value,0);
            rc = scale_task_req(req_adu,adu_size,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE);
            if ( rc != 0 ) {
                log_error("scale addr:%d net weight req err.\r\n",((scale_task_information_t*)argument)->addr);
                continue;
            }
            rc = scale_task_wait_rsp(rsp_adu,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE);
            if ( rc < 0 ) {
                log_error("scale addr:%d net weight rsp err.\r\n",((scale_task_information_t*)argument)->addr);
                continue;
            }
            adu_size = rc;
            rc = scale_task_parse_pdu(((scale_task_information_t*)argument)->addr,SCALE_TASK_PDU_CODE_NET_WEIGHT,&rsp_adu[SCALE_TASK_ADU_PDU_OFFSET],adu_size - 5,rsp_value,&rsp_value_cnt);
            if (rc != 0 || rsp_value_cnt != 2) {
                log_error("scale addr:%d net weight parse cnt:%d err.\r\n",((scale_task_information_t*)argument)->addr,rsp_value_cnt);
                continue;
            }
            goto exit;
        }
        /*去除皮重*/
        if (req_msg.type == REQ_REMOVE_TAR_WEIGHT) { 
            rsp_msg.type = RSP_REMOVE_TAR_WEIGHT;
            adu_size = scale_task_build_adu(req_adu,rsp_msg.reserved,SCALE_TASK_PDU_CODE_REMOVE_TAR_WEIGHT,req_value,0);
            rc = scale_task_req(req_adu,adu_size,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE);
            if ( rc != 0 ) {
                log_error("scale addr:%d remove tar weight req err.\r\n",((scale_task_information_t*)argument)->addr);
                rsp_value[0] = SCALE_TASK_FAILURE;
                rsp_value_cnt = 1;
                goto exit;
            }
            rc = scale_task_wait_rsp(rsp_adu,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE);
            if ( rc < 0 ) {
                log_error("scale addr:%d remove tar weight rsp err.\r\n",((scale_task_information_t*)argument)->addr);
                rsp_value[0] = SCALE_TASK_FAILURE;
                rsp_value_cnt = 1;
                goto exit;
            }
            adu_size = rc;
            rc = scale_task_parse_pdu(((scale_task_information_t*)argument)->addr,SCALE_TASK_PDU_CODE_NET_WEIGHT,&rsp_adu[SCALE_TASK_ADU_PDU_OFFSET],adu_size - 5,rsp_value,&rsp_value_cnt);
            if (rc != 0 || rsp_value_cnt != 1) {
                log_error("scale addr:%d remove tar weight parse cnt:%d err.\r\n",((scale_task_information_t*)argument)->addr,rsp_value_cnt);
                rsp_value[0] = SCALE_TASK_FAILURE;
                rsp_value_cnt = 1;
                goto exit;
            }
            if (rsp_value[0] == SCALE_TASK_SUCCESS_VALUE) {
                rsp_value[0] = SCALE_TASK_SUCCESS;
            } else {
                rsp_value[0] = SCALE_TASK_FAILURE;
            }
            rsp_value_cnt = 1;
            goto exit;
        }

        /*0点校准*/
        if (req_msg.type == REQ_NET_WEIGHT) { 
            rsp_msg.type = RSP_NET_WEIGHT;
            req_value[0] = req_msg.value & 0xFF;
            req_value[1] = (req_msg.value >> 8) & 0xFF;
            adu_size = scale_task_build_adu(req_adu,rsp_msg.reserved,SCALE_TASK_PDU_CODE_REMOVE_TAR_WEIGHT,req_value,2);
            rc = scale_task_req(req_adu,adu_size,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE);
            if ( rc != 0 ) {
                log_error("scale addr:%d calibration zero req err.\r\n",((scale_task_information_t*)argument)->addr);
                rsp_value[0] = SCALE_TASK_FAILURE;
                rsp_value_cnt = 1;
                goto exit;
            }
            rc = scale_task_wait_rsp(rsp_adu,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE);
            if ( rc < 0 ) {
                log_error("scale addr:%d calibration zero rsp err.\r\n",((scale_task_information_t*)argument)->addr);
                rsp_value[0] = SCALE_TASK_FAILURE;
                rsp_value_cnt = 1;
                goto exit;
            }
            adu_size = rc;
            rc = scale_task_parse_pdu(((scale_task_information_t*)argument)->addr,SCALE_TASK_PDU_CODE_NET_WEIGHT,&rsp_adu[SCALE_TASK_ADU_PDU_OFFSET],adu_size - 5,rsp_value,&rsp_value_cnt);
            if (rc != 0 || rsp_value_cnt != 1) {
                log_error("scale addr:%d calibration zero parse cnt:%d err.\r\n",((scale_task_information_t*)argument)->addr,rsp_value_cnt);
                rsp_value[0] = SCALE_TASK_FAILURE;
                rsp_value_cnt = 1;
                goto exit;
            }
            if (rsp_value[0] == SCALE_TASK_SUCCESS_VALUE) {
                rsp_value[0] = SCALE_TASK_SUCCESS;
            } else {
                rsp_value[0] = SCALE_TASK_FAILURE;
            }
            rsp_value_cnt = 1;
            goto exit;
        }
        /*增益校准*/
        if (req_msg.type == REQ_NET_WEIGHT) { 
            rsp_msg.type = RSP_NET_WEIGHT;
           req_value[0] = req_msg.value & 0xFF;
            req_value[1] = (req_msg.value >> 8) & 0xFF;
            adu_size = scale_task_build_adu(req_adu,rsp_msg.reserved,SCALE_TASK_PDU_CODE_REMOVE_TAR_WEIGHT,req_value,2);
            rc = scale_task_req(req_adu,adu_size,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE);
            if ( rc != 0 ) {
                log_error("scale addr:%d calibration full req err.\r\n",((scale_task_information_t*)argument)->addr);
                rsp_value[0] = SCALE_TASK_FAILURE;
                rsp_value_cnt = 1;
                goto exit;
            }
            rc = scale_task_wait_rsp(rsp_adu,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE);
            if ( rc < 0 ) {
                log_error("scale addr:%d calibration full rsp err.\r\n",((scale_task_information_t*)argument)->addr);
                rsp_value[0] = SCALE_TASK_FAILURE;
                rsp_value_cnt = 1;
                goto exit;
            }
            adu_size = rc;
            rc = scale_task_parse_pdu(((scale_task_information_t*)argument)->addr,SCALE_TASK_PDU_CODE_NET_WEIGHT,&rsp_adu[SCALE_TASK_ADU_PDU_OFFSET],adu_size - 5,rsp_value,&rsp_value_cnt);
            if (rc != 0 || rsp_value_cnt != 1) {
                log_error("scale addr:%d calibration full parse cnt:%d err.\r\n",((scale_task_information_t*)argument)->addr,rsp_value_cnt);
                rsp_value[0] = SCALE_TASK_FAILURE;
                rsp_value_cnt = 1;
                goto exit;
            }
            if (rsp_value[0] == SCALE_TASK_SUCCESS_VALUE) {
                rsp_value[0] = SCALE_TASK_SUCCESS;
            } else {
                rsp_value[0] = SCALE_TASK_FAILURE;
            }
            rsp_value_cnt = 1;
            goto exit;
        }
        continue;
exit:           
        if (rsp_value_cnt == 2) {
            rsp_msg.value = rsp_value[1] << 8 | rsp_value[0];
        } else {
            rsp_msg.value = rsp_value[0];
        }
        rsp_msg.reserved = ((scale_task_information_t*)argument)->addr;
        status = osMessagePut(((scale_task_information_t*)argument)->host_msg_id,*(uint32_t*)&rsp_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
        if (status != osOK) {
            log_error("scale task put msg err.code:%d.\r\n",status);
        }
    }
    }
}

typedef struct
{
    uint8_t cnt;
    scale_task_information_t info[SCALE_TASK_SCALE_CNT_MAX];
    bool done[SCALE_TASK_SCALE_CNT_MAX];
}scale_configration_t;

/*
* @brief 电子称子任务配置初始化
* @param configration 任务参数指针
* @param host_msg_id 主任务消息队列句柄
* @return 无
* @note
*/
static void scale_task_scale_configration_init(scale_configration_t *configration,osMessageQId host_msg_id)
{
    configration->cnt = bsp_read_switch();
    if (configration->cnt > SCALE_TASK_SCALE_CNT_MAX) {
        configration->cnt = SCALE_TASK_SCALE_CNT_MAX;
    }
    for (uint8_t i = 1;i <= configration->cnt;i ++) {
        configration->info[i].addr = i * 10 + 1;
        configration->info[i].port = i;
        configration->info[i].baud_rates = SCALE_TASK_SERIAL_BAUDRATES;
        configration->info[i].data_bits = SCALE_TASK_SERIAL_DATABITS;
        configration->info[i].stop_bits = SCALE_TASK_SERIAL_STOPBITS;
        configration->info[i].host_msg_id = host_msg_id;
    }
}

/*
* @brief 电子称子任务创建
* @param configration 任务参数指针
* @return 无
* @note
*/
static void scale_host_task_create_sub_task(scale_configration_t *configration)
{
    for (uint8_t i = 0;i < configration->cnt;i ++) {
        osThreadDef(scale_task, scale_task, osPriorityNormal, 0, 256);
        configration->info[i].task_id = osThreadCreate(osThread(scale_task), NULL);
        log_assert((void *)configration->info[i].task_id);
    }
}


/*
* @brief 处理获取配置消息
* @param configration 配置指针
* @param msg_id 目的消息句柄
* @return 无
* @note
*/
static void scale_host_task_process_configration(osMessageQId msg_id,scale_configration_t *configration)
{
    osStatus status;
    static scale_host_task_configration_msg_t cfg_msg;
    
    cfg_msg.cnt = configration.cnt;
    for (uint8_t i = 0;i < cfg_msg.cnt;i ++) {
        cfg_msg.scale_addr[i] = configration->info[i].addr;
    }
    status = osMessagePut(msg_id,(uint32_t)&cfg_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        log_error("scale host task put cfg msg err.code:%d.\r\n",status);
    }   
}


static void scale_host_task_seek_
/*
* @brief 处理获取配置消息
* @param configration 配置指针
* @param msg_id 目的消息句柄
* @return 无
* @note
*/
static void scale_host_task_process_net_weight(osMessageQId msg_id,uint8_t addr,scale_configration_t *configration)
{
    osStatus status;
    osEvent os_event;
    uint8_t cnt;
    scale_host_msg_t req_msg,rsp_msg;
    utils_timer_t timer;
    static scale_host_task_net_weight_msg_t net_weight_msg;

    utils_timer_init(&timer,SCALE_HOST_TASK_MSG_WAIT_TIMEOUT_VALUE,false);
    req_msg.type = REQ_NET_WEIGHT;

    if (addr == 0) {
        net_weight_msg.cnt = configration->cnt;
    } else {
        net_weight_msg.cnt = 1;
    }

    for (uint8_t i = 0;i < configration->cnt;i ++) {
        osMessagePut(configration->info[i].msg_id,*(uint32_t *)&req_msg,SCALE_HOST_TASK_MSG_PUT_TIMEOUT_VALUE);
    }
    for (uint8_t i = 0;i < configration->cnt;i ++) {
         os_event = osMessageGet(scale_host_task_msg_q_id,utils_timer_value(&timer));
            if (os_event.status == osEventMessage) {
                rsp_msg = *(task_msg_t *)&os_msg.value.v;
                
                
        }

    status = osMessagePut(msg_id,(uint32_t)&net_weight_msg,SCALE_HOST_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        log_error("scale host task put cfg msg err.code:%d.\r\n",status);
    }   
}



/*
* @brief 电子秤主任务
* @param argument 任务参数
* @return 无
* @note
*/

void scale_host_task(void const *argument)
{
    int rc;
    osMessageQId scale_host_task_msg_q_id;
    scale_configration_t configration;
    
    

    osMessageQDef(scale_host_task_msg_q,10,uint32_t);
    scale_host_task_msg_q_id = osMessageCreate(osMessageQ(scale_host_task_msg_q),0);
    log_assert(((scale_task_information_t*)argument)->msg_id);
    /*初始化电子秤子任务配置*/
    scale_task_scale_configration_init(&configration,scale_host_task_msg_q_id);
    /*创建电子秤子任务*/
    scale_host_task_create_sub_task(&configration);
    while (1) {
        os_msg = osMessageGet(scale_host_task_msg_q_id,SCALE_HOST_TASK_MSG_WAIT_TIMEOUT_VALUE);
        if (os_msg.status == osEventMessage) {
            req_msg = *(task_msg_t *)&os_msg.value.v;
 
        /*获取称的配置信息*/
        if (req_msg.type == REQ_SCALE_CONFIGRATION) { 
            scale_host_task_process_configration(controller_task_cfg_msg_q_id,&configration);
        }
        /*获取净重值*/
        if (req_msg.type == REQ_NET_WEIGHT) { 
            scale_host_task_process_net_weight(controller_task_weight_msg_q_id,req_msg.reserved,&configration);
        }
        /*去除皮重*/
        if (req_msg.type == REQ_REMOVE_TAR_WEIGHT) { 
            scale_host_task_process_tare_weight(controller_task_weight_msg_q_id,req_msg.reserved);
        }
        /*0点校准*/
        if (req_msg.type == REQ_CALIBRATION_ZERO) { 
            scale_host_task_process_calibration_zero(controller_task_zero_msg_q_id,req_msg.reserved,req_msg.value);
        }
        /*获取净重值*/
        if (req_msg.type == REQ_NET_WEIGHT) { 
            scale_host_task_process_calibration_full(controller_task_full_msg_q_id,req_msg.reserved,req_msg.value);
        }

    }


}


       