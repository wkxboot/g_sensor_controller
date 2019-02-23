#include "board.h"
#include "cmsis_os.h"
#include "serial.h"
#include "tasks_init.h"
#include "utils.h"
#include "serial.h"
#include "scale_task.h"
#include "controller_task.h"
#include "log.h"

extern int scale_serial_handle;
extern serial_hal_driver_t nxp_serial_uart_hal_driver;

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
static int scale_task_req(const int handle,const uint8_t *adu,const uint8_t len,const uint16_t timeout)
{
    uint16_t length_to_write,write_length,remain_length;
 
    serial_flush(handle);
    length_to_write = len;
    write_length = serial_write(handle,(const char*)adu,length_to_write);
    for (int i=0; i < write_length; i++){
        log_debug("[%2X]\r\n", adu[i]);
    }
    if (write_length != length_to_write){
        log_error("scale err in  serial buffer write. expect:%d write:%d.\r\n",length_to_write,write_length); 
        return -1;    
    } 
    remain_length = serial_complete(handle,timeout);
    if (remain_length != 0) {
        log_error("scale err in  serial send timeout.\r\n"); 
        return -1;
    }
   
    return 0;
}

static int scale_task_wait_rsp(const int handle,uint8_t *adu,uint32_t timeout)
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
        rc = serial_select(handle,timeout);
        if (rc == -1) {
            log_error("scale select error.read_len:%d. len_to_read:%d.\r\n",read_length,length_to_read);
            goto exit;
        }
        if (rc == 0) {
            log_error("scale select timeout.read_len:%d. len_to_read:%d.timeout:%d.\r\n",read_length,length_to_read,timeout);
            goto exit;
        }
  
        rc = serial_read(handle,(char *)adu + read_length,length_to_read);
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
static int scale_task_parse_pdu(const uint8_t *pdu,uint8_t pdu_size,const uint8_t addr,const uint8_t code,uint8_t *value,uint8_t value_cnt)
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
    
    if (pdu_size !=  value_cnt) {
        log_error("pdu rsp value size:%d != %d err.\r\n",pdu_size,value_cnt);
        goto err_exit;
    }
    for (uint8_t i = 0;i < pdu_size;i ++) {
        value[i] = pdu[SCALE_TASK_PDU_VALUE_OFFSET + i];
    }
  
    return 0;

err_exit:
    return -1;
}

/*
* @brief 电子称子任务读取硬件电子称配置
* @param config 硬件配置指针
* @return 0 成功
* @return -1 失败
* @note
*/
static int scale_task_read_scale_hard_configration(scale_hard_configration_t *config)
{
    config->cnt = 4;
    config->addr[0] = 11;
    config->addr[1] = 21;
    config->addr[2] = 31;
    config->addr[3] = 41;
    
    return 0;
}
/*
* @brief 电子称子任务配置初始化
* @param configration 任务参数指针
* @param host_msg_id 主任务消息队列句柄
* @return 无
* @note
*/
static void scale_task_information_init(scale_task_information_t *info)
{
    int rc;
    scale_hard_configration_t hard_configratin;

    rc = scale_task_read_scale_hard_configration(&hard_configratin);
    log_assert(rc == 0);

    info->cnt = hard_configratin.cnt;

    for (uint8_t i = 0;i < info->cnt;i ++) {
        info->scale[i].addr = hard_configratin.addr[i];
        info->scale[i].port = i;
        info->scale[i].baud_rates = SCALE_TASK_SERIAL_BAUDRATES;
        info->scale[i].data_bits = SCALE_TASK_SERIAL_DATABITS;
        info->scale[i].stop_bits = SCALE_TASK_SERIAL_STOPBITS;

        rc = serial_create(&info->scale[i].handle,CONTROLLER_TASK_RX_BUFFER_SIZE,CONTROLLER_TASK_TX_BUFFER_SIZE);
        log_assert(rc == 0);
        rc = serial_register_hal_driver(info->scale[i].handle,&nxp_serial_uart_hal_driver);
        log_assert(rc == 0);
 
        rc = serial_open(info->scale[i].handle,
                        info->scale[i].port,
                        info->scale[i].baud_rates,
                        info->scale[i].data_bits,
                        info->scale[i].stop_bits);
        log_assert(rc == 0); 
    }


    
}




/*
* @brief 处理获取配置消息
* @param configration 配置指针
* @param msg_id 目的消息句柄
* @return 无
* @note
*/
static void scale_task_process_configration_msg(scale_task_information_t *info,osMessageQId msg_id)
{
    osStatus status;
    static scale_task_configration_msg_t cfg_msg;
    
    cfg_msg.cnt = info->cnt;
    for (uint8_t i = 0;i < cfg_msg.cnt;i ++) {
        cfg_msg.scale_addr[i] = info->scale[i].addr;
    }
    status = osMessagePut(msg_id,(uint32_t)&cfg_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
    if (status != osOK) {
        log_error("scale host task put cfg msg err.code:%d.\r\n",status);
    }   
}


/*
* @brief 处理获取净重值
* @param info 电子秤任务信息指针
* @param addr 电子秤地址
* @param msg_id 目的消息句柄
* @return 无
* @note
*/
static int scale_task_process_net_weight_msg(scale_task_information_t *info,uint8_t addr,osMessageQId msg_id)
{
    int rc ;
    osStatus status;
    uint8_t adu_size;
    uint8_t req_adu[SCALE_TASK_FRAME_SIZE_MAX];
    uint8_t rsp_adu[SCALE_TASK_FRAME_SIZE_MAX];
    uint8_t req_value[2];
    uint8_t rsp_value[2];
    uint8_t rsp_value_cnt;
    utils_timer_t timer;
    static scale_task_net_weight_msg_t msg;
    utils_timer_init(&timer,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE,false);

    msg.cnt = 0;
    msg.index = 0;
    for (uint8_t i = 0;i < info->cnt;i ++) {
        if (addr != 0 && addr != info->scale[i].addr) {
            continue;
        }
        msg.cnt ++;/*判断有效地址的数量*/
        msg.index = i;
        adu_size = scale_task_build_adu(req_adu,info->scale[i].addr,SCALE_TASK_PDU_CODE_NET_WEIGHT,req_value,0);
        rc = scale_task_req(info->scale[i].handle,req_adu,adu_size,utils_timer_value(&timer));
        if ( rc != 0 ) {
           log_error("scale addr:%d net req err.\r\n",addr);
           continue;
        }
    }

    for (uint8_t i = 0;i < info->cnt;i ++) {
        if (addr != 0 && addr != info->scale[i].addr) {
            continue;
        }
        rc = scale_task_wait_rsp(info->scale[i].handle,rsp_adu,utils_timer_value(&timer));
        if ( rc <= 0 ) {
            log_error("scale addr:%d net weight rsp err.\r\n",addr);
            msg.net_weight[i] = SCALE_TASK_ERR_NET_WEIGHT;
            continue;
        }
        adu_size = rc;
        rc = scale_task_parse_pdu(&rsp_adu[SCALE_TASK_ADU_PDU_OFFSET],adu_size - 5,addr,SCALE_TASK_PDU_CODE_NET_WEIGHT,rsp_value,2);
        
        if (rc != 0 || rsp_value_cnt != 2) {
            log_error("scale addr:%d net weight parse err.\r\n",addr);
            msg.net_weight[i] = SCALE_TASK_ERR_NET_WEIGHT;
            continue;
        } else {
            msg.net_weight[i] = rsp_value[1] << 8 | rsp_value[0];            
        }
    }
    rc = -1;
    if ( msg.cnt > 0 ) {
        status = osMessagePut(msg_id,(uint32_t)&msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
        if (status != osOK) {
            log_error("scale task put net weight msg err.code:%d.\r\n",status);
        }else {
            rc = 0;
        }
    }

    return rc;
}


/*
* @brief 处理去皮置零
* @param info 电子秤任务信息指针
* @param addr 电子秤地址
* @param msg_id 目的消息句柄
* @return 无
* @note
*/
static int scale_task_process_remove_tare_weight_msg(scale_task_information_t *info,uint8_t addr,osMessageQId msg_id)
{
    int rc ;
    osStatus status;
    uint8_t adu_size;
    uint8_t req_adu[SCALE_TASK_FRAME_SIZE_MAX];
    uint8_t rsp_adu[SCALE_TASK_FRAME_SIZE_MAX];
    uint8_t req_value[2];
    uint8_t rsp_value[2];
    uint8_t rsp_value_cnt;
    utils_timer_t timer;
    static scale_task_opt_msg_t msg;
    utils_timer_init(&timer,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE,false);

    msg.cnt = 0;
    msg.index = 0;
    for (uint8_t i = 0;i < info->cnt;i ++) {
        if (addr != 0 && addr != info->scale[i].addr) {
            continue;
        }
        msg.cnt ++;/*判断有效地址的数量*/
        msg.index = i;
        adu_size = scale_task_build_adu(req_adu,info->scale[i].addr,SCALE_TASK_PDU_CODE_REMOVE_TARE_WEIGHT,req_value,0);
        rc = scale_task_req(info->scale[i].handle,req_adu,adu_size,utils_timer_value(&timer));
        if ( rc != 0 ) {
           log_error("scale addr:%d remove tare req err.\r\n",addr);
           continue;
        }
    }

    for (uint8_t i = 0;i < info->cnt;i ++) {
        if (addr != 0 && addr != info->scale[i].addr) {
            continue;
        }
        rc = scale_task_wait_rsp(info->scale[i].handle,rsp_adu,utils_timer_value(&timer));
        if ( rc <= 0 ) {
            log_error("scale addr:%d remove tare weight rsp err.\r\n",addr);
            msg.opt_value[i] = SCALE_TASK_FAILURE;
            continue;
        }
        adu_size = rc;
        rc = scale_task_parse_pdu(&rsp_adu[SCALE_TASK_ADU_PDU_OFFSET],adu_size - 5,addr,SCALE_TASK_PDU_CODE_REMOVE_TARE_WEIGHT,rsp_value,2);
        
        if (rc != 0 || rsp_value_cnt != 1) {
            log_error("scale addr:%d remove tare weight parse err.\r\n",addr);
            msg.opt_value[i] = SCALE_TASK_FAILURE;
            continue;
        } else {
            msg.opt_value[i] = rsp_value[0] == SCALE_TASK_SUCCESS_VALUE ? SCALE_TASK_SUCCESS : SCALE_TASK_FAILURE;            
        }
    }
    rc = -1;
    if ( msg.cnt > 0 ) {
        status = osMessagePut(msg_id,(uint32_t)&msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
        if (status != osOK) {
            log_error("scale task put remove tare msg err.code:%d.\r\n",status);
        }else {
            rc = 0;
        }
    }

    return rc;
}

/*
* @brief 处理0点校准
* @param info 电子秤任务信息指针
* @param addr 电子秤地址
* @param weight 校准重量
* @param msg_id 目的消息句柄
* @return 无
* @note
*/
static int scale_task_process_calibration_zero_msg(scale_task_information_t *info,uint8_t addr,int16_t weight,osMessageQId msg_id)
{
    int rc ;
    osStatus status;
    uint8_t adu_size;
    uint8_t req_adu[SCALE_TASK_FRAME_SIZE_MAX];
    uint8_t rsp_adu[SCALE_TASK_FRAME_SIZE_MAX];
    uint8_t req_value[2];
    uint8_t rsp_value[2];
    uint8_t rsp_value_cnt;
    utils_timer_t timer;
    static scale_task_opt_msg_t msg;
    utils_timer_init(&timer,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE,false);

    msg.cnt = 0;
    msg.index = 0;
    req_value[0] = weight & 0xFF;
    req_value[1] = weight >> 8;

    for (uint8_t i = 0;i < info->cnt;i ++) {
        if (addr != 0 && addr != info->scale[i].addr) {
            continue;
        }
        msg.cnt ++;/*判断有效地址的数量*/
        msg.index = i;
        adu_size = scale_task_build_adu(req_adu,info->scale[i].addr,SCALE_TASK_PDU_CODE_CALIBRATION_ZERO,req_value,2);
        rc = scale_task_req(info->scale[i].handle,req_adu,adu_size,utils_timer_value(&timer));
        if ( rc != 0 ) {
           log_error("scale addr:%d calibration zero req err.\r\n",addr);
           continue;
        }
    }

    for (uint8_t i = 0;i < info->cnt;i ++) {
        if (addr != 0 && addr != info->scale[i].addr) {
            continue;
        }
        rc = scale_task_wait_rsp(info->scale[i].handle,rsp_adu,utils_timer_value(&timer));
        if ( rc <= 0 ) {
            log_error("scale addr:%d calibration zero rsp err.\r\n",addr);
            msg.opt_value[i] = SCALE_TASK_FAILURE;
            continue;
        }
        adu_size = rc;
        rc = scale_task_parse_pdu(&rsp_adu[SCALE_TASK_ADU_PDU_OFFSET],adu_size - 5,addr,SCALE_TASK_PDU_CODE_CALIBRATION_ZERO,rsp_value,2);
        
        if (rc != 0 || rsp_value_cnt != 1) {
            log_error("scale addr:%d calibration zero parse err.\r\n",addr);
            msg.opt_value[i] = SCALE_TASK_FAILURE_VALUE;
            continue;
        } else {
            msg.opt_value[i] = rsp_value[0] == SCALE_TASK_SUCCESS_VALUE ? SCALE_TASK_SUCCESS : SCALE_TASK_FAILURE;         
        }
    }
    rc = -1;
    if ( msg.cnt > 0 ) {
        status = osMessagePut(msg_id,(uint32_t)&msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
        if (status != osOK) {
            log_error("scale task put calibration zero msg err.code:%d.\r\n",status);
        }else {
            rc = 0;
        }
    }

    return rc;
}
/*
* @brief 处理增益校准
* @param info 电子秤任务信息指针
* @param addr 电子秤地址
* @param weight 校准重量
* @param msg_id 目的消息句柄
* @return 无
* @note
*/
static int scale_task_process_calibration_full_msg(scale_task_information_t *info,uint8_t addr,int16_t weight,osMessageQId msg_id)
{
    int rc ;
    osStatus status;
    uint8_t adu_size;
    uint8_t req_adu[SCALE_TASK_FRAME_SIZE_MAX];
    uint8_t rsp_adu[SCALE_TASK_FRAME_SIZE_MAX];
    uint8_t req_value[2];
    uint8_t rsp_value[2];
    uint8_t rsp_value_cnt;
    utils_timer_t timer;
    static scale_task_opt_msg_t msg;
    utils_timer_init(&timer,SCALE_TASK_SCALE_RSP_TIMEOUT_VALUE,false);

    msg.cnt = 0;
    msg.index = 0;
    req_value[0] = weight & 0xFF;
    req_value[1] = weight >> 8;

    for (uint8_t i = 0;i < info->cnt;i ++) {
        if (addr != 0 && addr != info->scale[i].addr) {
            continue;
        }
        msg.cnt ++;/*判断有效地址的数量*/
        msg.index = i;
        adu_size = scale_task_build_adu(req_adu,info->scale[i].addr,SCALE_TASK_PDU_CODE_CALIBRATION_ZERO,req_value,2);
        rc = scale_task_req(info->scale[i].handle,req_adu,adu_size,utils_timer_value(&timer));
        if ( rc != 0 ) {
           log_error("scale addr:%d calibration full req err.\r\n",addr);
           continue;
        }
    }

    for (uint8_t i = 0;i < info->cnt;i ++) {
        if (addr != 0 && addr != info->scale[i].addr) {
            continue;
        }
        rc = scale_task_wait_rsp(info->scale[i].handle,rsp_adu,utils_timer_value(&timer));
        if ( rc <= 0 ) {
            log_error("scale addr:%d calibration full rsp err.\r\n",addr);
            msg.opt_value[i] = SCALE_TASK_FAILURE;
            continue;
        }
        adu_size = rc;
        rc = scale_task_parse_pdu(&rsp_adu[SCALE_TASK_ADU_PDU_OFFSET],adu_size - 5,addr,SCALE_TASK_PDU_CODE_CALIBRATION_ZERO,rsp_value,2);
        
        if (rc != 0 || rsp_value_cnt != 1) {
            log_error("scale addr:%d calibration full parse err.\r\n",addr);
            msg.opt_value[i] = SCALE_TASK_FAILURE_VALUE;
            continue;
        } else {
            msg.opt_value[i] = rsp_value[0] == SCALE_TASK_SUCCESS_VALUE ? SCALE_TASK_SUCCESS : SCALE_TASK_FAILURE;         
        }
    }
    rc = -1;
    if ( msg.cnt > 0 ) {
        status = osMessagePut(msg_id,(uint32_t)&msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
        if (status != osOK) {
            log_error("scale task put calibration full msg err.code:%d.\r\n",status);
        }else {
            rc = 0;
        }
    }

    return rc;
}
/*
* @brief 电子秤主任务
* @param argument 任务参数
* @return 无
* @note
*/

void scale_task(void const *argument)
{
    osEvent os_event;
    task_msg_t req_msg;
    scale_task_information_t information;

    /*初始化电子秤子任务配置*/
    scale_task_information_init(&information);
    while (1) {
        os_event = osMessageGet(scale_task_msg_q_id,SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE);
        if (os_event.status == osEventMessage) {
            req_msg = *(task_msg_t *)&os_event.value.v;
 
            /*获取称的配置信息*/
            if (req_msg.type == REQ_CONFIGRATION) { 
                scale_task_process_configration_msg(&information,controller_task_cfg_msg_q_id);
            }
            /*获取净重值*/
            if (req_msg.type == REQ_NET_WEIGHT) { 
                scale_task_process_net_weight_msg(&information,req_msg.reserved,controller_task_cfg_msg_q_id);
            }
            /*去除皮重*/
            if (req_msg.type == REQ_REMOVE_TAR_WEIGHT) { 
                scale_task_process_remove_tare_weight_msg(&information,req_msg.reserved,controller_task_cfg_msg_q_id);
            }
            /*0点校准*/
            if (req_msg.type == REQ_CALIBRATION_ZERO) { 
                scale_task_process_calibration_zero_msg(&information,req_msg.reserved,req_msg.value,controller_task_calibration_zero_msg_q_id);
            }
            /*获取净重值*/
            if (req_msg.type == REQ_NET_WEIGHT) { 
                scale_task_process_calibration_full_msg(&information,req_msg.reserved,req_msg.value,controller_task_calibration_full_msg_q_id);
            }
        }

    }


}


       