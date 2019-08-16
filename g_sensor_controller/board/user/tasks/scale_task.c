#include "board.h"
#include "cmsis_os.h"
#include "xuart.h"
#include "xstring.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "communication_task.h"
#include "crc16.h"
#include "log.h"

extern int scale_serial_handle;
extern xuart_hal_driver_t xuart_hal_driver;

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
#define  PDU_CODE_NET_WEIGHT           0
#define  PDU_CODE_REMOVE_TARE_WEIGHT   1
#define  PDU_CODE_CALIBRATION_ZERO     2
#define  PDU_CODE_CALIBRATION_FULL     3
#define  PDU_CODE_SENSOR_ID            4
#define  PDU_CODE_FIRMWARE_VERSION     5
#define  PDU_CODE_SET_ADDR             6
#define  PDU_CODE_MAX                  PDU_CODE_SET_ADDR

/*协议错误码*/
#define  PDU_NET_WEIGHT_ERR_VALUE      0x7FFF
#define  PDU_SUCCESS_VALUE             0x00
#define  PDU_FAILURE_VALUE             0x01
/*协议时间*/
#define  ADU_WAIT_TIMEOUT              osWaitForever
#define  ADU_FRAME_TIMEOUT             3
#define  ADU_QUERY_WEIGHT_TIMEOUT      35
#define  ADU_REMOVE_TARE_TIMEOUT       500
#define  ADU_CALIBRATION_ZERO_TIMEOUT  500
#define  ADU_CALIBRATION_FULL_TIMEOUT  500
#define  ADU_SEND_TIMEOUT              5

/*
* @brief 计算缓存CRC并填充到缓存尾端
* @param adu 缓存指针
* @param size 当前数据缓存长度
* @return 加上crc后的数据长度
* @note
*/

static uint8_t adu_add_crc16(uint8_t *adu,uint8_t size)
{
    uint16_t crc_calculated;
    crc_calculated = calculate_crc16(adu,size);

    adu[size ++] =  crc_calculated & 0xff;
    adu[size ++] =  crc_calculated >> 8;
    return size;
}
/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static int build_adu(uint8_t *adu,uint8_t addr,uint8_t code,uint8_t *value,uint8_t cnt)
{
    uint8_t adu_size = 0;
    /*ADU head*/
    adu[adu_size ++] = ADU_HEAD0_VALUE;
    adu[adu_size ++] = ADU_HEAD1_VALUE;
    /*PDU len*/
    adu[adu_size ++] = cnt + 1 + 1;/*pdu code + pdu scale_addr*/
    /*ADU addr*/
    adu[adu_size ++] = addr;
    /*PDU code*/
    adu[adu_size ++] = code;
    /*PDU value*/
    for (uint8_t i = 0;i < cnt;i ++) {
         adu[adu_size ++] = value[i];
    }
    adu_size = adu_add_crc16(adu,adu_size);

    return adu_size;
}


/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int send_adu(xuart_handle_t *handle,const uint8_t *adu,const uint8_t size,const uint16_t timeout)
{
    uint16_t write_size,write_size_total;
    char buffer[ADU_SIZE_MAX * 2 + 1];

    xuart_clear(handle);
    write_size = size;
    write_size_total = xuart_write(handle,adu,write_size);

    /*打印输出的数据*/
    xstring_hex_to_string(buffer,(const char *)adu,write_size_total);
    log_debug("[send] %s\r\n",buffer);

    if (write_size_total != write_size){
        log_error("scale err in  serial buffer write. expect:%d write:%d.\r\n",write_size,write_size_total); 
        return -1;    
    } 
   
    return xuart_complete(handle,timeout);
}
/*
* @brief 串口接收ADU
* @param handle 串口句柄
* @param adu 数据缓存指针
* @param wait_timeout 等待超时时间
* @return -1 失败
* @return  0 成功
* @note
*/
static int receive_adu(xuart_handle_t *handle,uint8_t *adu,uint32_t wait_timeout)
{
    int rc;
    int read_size,read_size_total = 0;
    uint32_t timeout;
    uint16_t crc_calculated;
    uint16_t crc_received;
    adu_step_t step;
    char buffer[ADU_SIZE_MAX * 2 + 1];

    timeout = wait_timeout;
    read_size = ADU_HEAD_SIZE + ADU_PDU_SIZE_REGION_SIZE;
    read_size_total = 0;
    step = ADU_HEAD_STEP;
  
    while(read_size != 0) {
        rc = xuart_select(handle,timeout);
        if (rc == -1) {
            log_error("adu select error.read total:%d. read size:%d.\r\n",read_size_total,read_size);
            return -1;
        }
        if (rc == 0) {
            log_error("adu select timeout.read total:%d. read size:%d.timeout:%d.\r\n",read_size_total,read_size,timeout);
            return -1;
        }
  
        rc = xuart_read(handle,adu + read_size_total,read_size);
        if (rc == -1) {
            log_error("adu read error.read total:%d. read size:%d.\r\n",read_size_total,read_size);
            return -1;
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
                /*打印输出的数据*/
                xstring_hex_to_string(buffer,(const char *)adu,read_size_total);
                log_debug("[recv] %s\r\n",buffer);
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
* @brief 
* @param
* @param
* @return 
* @note
*/
static int parse_pdu(const uint8_t *pdu,int size,const uint8_t addr,const uint8_t code,uint8_t *value)
{
    int rc;
    uint8_t opt_code;
    uint8_t scale_addr;
    uint8_t pdu_offset = 0;

    if (size < PDU_SIZE_MIN) {
        log_error("pdu size:%d < %d err.\r\n",size,PDU_SIZE_MIN);
        return -1;
    }
    scale_addr = pdu[pdu_offset ++];
    if (scale_addr != addr) {
        log_error("pdu addr:%d != %d err.\r\n",scale_addr,addr);
        return -1;
    }
    opt_code = pdu[pdu_offset ++];
    if (opt_code != code) {
        log_error("pdu code:%d > %d err.\r\n",opt_code,code);
        return -1;
    }
    switch (opt_code) {
    case PDU_CODE_NET_WEIGHT:
        value[0] = pdu[pdu_offset ++];
        value[1] = pdu[pdu_offset ++];      
        if (pdu_offset != size ) {
            log_error("pdu size:%d of net weight err.\r\n",size);
            return -1;
        }  
        rc = 2;
        break;
     case PDU_CODE_REMOVE_TARE_WEIGHT:
        value[0] = pdu[pdu_offset ++];
        if (pdu_offset != size ) {
            log_error("pdu size:%d of remove tare weight err.\r\n",size);
            return -1;
        }
        rc = 1;
        break;   
     case PDU_CODE_CALIBRATION_ZERO:
        value[0] = pdu[pdu_offset ++];
        if (pdu_offset != size ) {
            log_error("pdu size:%d of calibration zero err.\r\n",size);
            return -1;
        }
        rc = 1;
        break;  
     case PDU_CODE_CALIBRATION_FULL:
        value[0] = pdu[pdu_offset ++];
        if (pdu_offset != size ) {
            log_error("pdu size:%d of calibration full err.\r\n",size);
            return -1;
        }
        rc = 1;
        break;  
    default:
        log_error("adu internal err.code:%d.\r\n",code);
        return -1;
    }
    return rc;
}

/*
* @brief 轮询电子秤
* @param handle 电子秤通信句柄
* @param addr 电子秤地址
* @param code 操作码
* @param value 操作值指针
* @param size 操作值数量
* @param rsp 回应缓存
* @param timeout 回应超时
* @return > 0 回应的数据量
* @return -1 失败
* @note
*/
static int scale_task_poll(xuart_handle_t *handle,uint8_t addr,uint8_t code,uint8_t *value,uint8_t size,uint8_t *rsp,uint32_t timeout)
{
    int rc ;
    uint8_t adu_send[ADU_SIZE_MAX];
    uint8_t adu_recv[ADU_SIZE_MAX];

    rc = build_adu(adu_send,addr,code,value,size);
    if (rc <= 0) {
        return -1;
    }

    rc = send_adu(handle,adu_send,rc,ADU_SEND_TIMEOUT);
    if (rc != 0) {
        return -1;
    }

    rc = receive_adu(handle,adu_recv,timeout);
    if (rc <= 0) {
        /*清空接收缓存*/
        xuart_clear(handle);
        return -1;
    }
    rc = parse_pdu((uint8_t *)&adu_recv[ADU_PDU_OFFSET],rc - ADU_HEAD_SIZE - ADU_PDU_SIZE_REGION_SIZE - ADU_CRC_SIZE,addr,code,rsp);
    if (rc < 0 ) {
        return -1;
    }
    return rc;
}
 

/*
* @brief 电子秤任务
* @param argument 任务参数
* @return 无
* @note
*/

void scale_task(void const *argument)
{
    int rc;
    osStatus status;
    osEvent os_event;
    uint8_t req_value[2];
    uint8_t rsp_value[2];

    scale_task_message_t req_msg,net_weight_msg,remove_tare_msg,calibration_zero_msg,calibration_full_msg;
    scale_task_contex_t *task_contex;

    task_contex = (scale_task_contex_t *)argument;
    while (1) {
        os_event = osMessageGet(task_contex->msg_q_id,SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE);
        if (os_event.status == osEventMessage) {
            req_msg = *(scale_task_message_t *)os_event.value.v;
 
            /*获取净重值*/
            if (req_msg.request.type == SCALE_TASK_MSG_TYPE_NET_WEIGHT) { 
                rc = scale_task_poll(&task_contex->handle,task_contex->phy_addr,PDU_CODE_NET_WEIGHT,req_value,0,rsp_value,ADU_QUERY_WEIGHT_TIMEOUT);
                if (rc < 0) {
                    net_weight_msg.response.weight = SCALE_TASK_NET_WEIGHT_ERR_VALUE;
                    log_error("scale:%d poll net weight err.\r\n",task_contex->internal_addr);
                } else {
                    net_weight_msg.response.weight = (uint16_t)rsp_value[1] << 8 | rsp_value[0];
                    if (net_weight_msg.response.weight == PDU_NET_WEIGHT_ERR_VALUE) {
                        net_weight_msg.response.weight = SCALE_TASK_NET_WEIGHT_ERR_VALUE;   
                    }
                }
                net_weight_msg.response.type = SCALE_TASK_MSG_TYPE_RSP_NET_WEIGHT;
                net_weight_msg.response.index = req_msg.request.index;
                net_weight_msg.response.flag = task_contex->flag;

                status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t )&net_weight_msg,SCALE_TASK_PUT_MSG_TIMEOUT);
                if (status != osOK) {
                    log_error("put net weight msg err:%d.\r\n",status);
                }  
            }
           

            /*去除皮重*/
            if (req_msg.request.type == SCALE_TASK_MSG_TYPE_REMOVE_TARE_WEIGHT) { 
                rc = scale_task_poll(&task_contex->handle,task_contex->phy_addr,PDU_CODE_REMOVE_TARE_WEIGHT,req_value,0,rsp_value,ADU_REMOVE_TARE_TIMEOUT);
                if (rc < 0) {
                    remove_tare_msg.response.result = SCALE_TASK_FAIL;
                    log_error("scale:%d poll remove tare weight err.\r\n",task_contex->internal_addr);
                } else {
                    remove_tare_msg.response.result = rsp_value[0] ==  PDU_SUCCESS_VALUE ? SCALE_TASK_SUCCESS : SCALE_TASK_FAIL;
                }
                remove_tare_msg.response.type = SCALE_TASK_MSG_TYPE_RSP_REMOVE_TARE_WEIGHT;
                remove_tare_msg.response.index = req_msg.request.index;
                remove_tare_msg.response.flag = task_contex->flag;

                status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t )&remove_tare_msg,SCALE_TASK_PUT_MSG_TIMEOUT);
                if (status != osOK) {
                    log_error("put net weight msg err:%d.\r\n",status);
                }                      
            }

            /*0点校准*/
            if (req_msg.request.type == SCALE_TASK_MSG_TYPE_CALIBRATION_ZERO_WEIGHT) { 
                req_value[0] = req_msg.request.weight & 0xFF;
                req_value[1] = req_msg.request.weight >> 8;
                rc = scale_task_poll(&task_contex->handle,task_contex->phy_addr,PDU_CODE_CALIBRATION_ZERO,req_value,2,rsp_value,ADU_CALIBRATION_ZERO_TIMEOUT);
                if (rc < 0) {
                    calibration_zero_msg.response.result = SCALE_TASK_FAIL;
                    log_error("scale:%d poll calibration zero weight err.\r\n",task_contex->internal_addr);
                } else {
                    calibration_zero_msg.response.result = rsp_value[0] ==  PDU_SUCCESS_VALUE ? SCALE_TASK_SUCCESS : SCALE_TASK_FAIL;
                }
                calibration_zero_msg.response.type = SCALE_TASK_MSG_TYPE_RSP_CALIBRATION_ZERO_WEIGHT;
                calibration_zero_msg.response.index = req_msg.request.index;
                calibration_zero_msg.response.flag = task_contex->flag;

                status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t )&calibration_zero_msg,SCALE_TASK_PUT_MSG_TIMEOUT);
                if (status != osOK) {
                    log_error("put calibration zero weight msg err:%d.\r\n",status);
                }                             
            }

            /*增益校准*/
            if (req_msg.request.type == SCALE_TASK_MSG_TYPE_CALIBRATION_FULL_WEIGHT) { 
                req_value[0] = req_msg.request.weight & 0xFF;
                req_value[1] = req_msg.request.weight >> 8;
                rc = scale_task_poll(&task_contex->handle,task_contex->phy_addr,PDU_CODE_CALIBRATION_FULL,req_value,2,rsp_value,ADU_CALIBRATION_ZERO_TIMEOUT);
                if (rc < 0) {
                    calibration_full_msg.response.result = SCALE_TASK_FAIL;
                    log_error("scale:%d poll calibration full weight err.\r\n",task_contex->internal_addr);
                } else {
                     calibration_full_msg.response.result = rsp_value[0] ==  PDU_SUCCESS_VALUE ? SCALE_TASK_SUCCESS : SCALE_TASK_FAIL;
                }
                calibration_full_msg.response.type = SCALE_TASK_MSG_TYPE_RSP_CALIBRATION_FULL_WEIGHT;              
                calibration_full_msg.response.index = req_msg.request.index;
                calibration_full_msg.response.flag = task_contex->flag;

                status = osMessagePut(req_msg.request.rsp_message_queue_id,(uint32_t )&calibration_full_msg,SCALE_TASK_PUT_MSG_TIMEOUT);
                if (status != osOK) {
                    log_error("put calibration full weight msg err:%d.\r\n",status);
                }                          
            }
        }

    }


}


       