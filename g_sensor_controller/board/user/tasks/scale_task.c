#include "board.h"
#include "cmsis_os.h"
#include "serial.h"
#include "tasks_init.h"
#include "scale_task.h"
#include "protocol_task.h"
#include "log.h"

extern int scale_serial_handle;
extern serial_hal_driver_t scale_serial_driver;

osThreadId   scale_task_hdl;
osMessageQId scale_task_msg_q_id;

static task_msg_t protocol_msg;

typedef enum
{
    SCALE_TASK_ADU_STEP=0,
    SCALE_TASK_CRC_STEP
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
* @brief 
* @param
* @param
* @return 
* @note
*/
static int scale_task_req(uint8_t *const req,const uint8_t len,const uint16_t timeout)
{
   uint16_t length_to_write,write_length,remain_length;
 
   serial_flush(scale_serial_handle);
   length_to_write = len;
   write_length = serial_write(scale_serial_handle,req,length_to_write);
   for (int i=0; i < write_length; i++){
   log_debug("[%2X]\r\n", req[i]);
   }
   if(write_length != length_to_write){
   log_error("scale err in  serial buffer write. expect:%d write:%d.\r\n",length_to_write,write_length); 
   return -1;    
   } 
   remain_length = serial_complete(scale_serial_handle,timeout);
   if(remain_length != 0){
   log_error("scale err in  serial send timeout.\r\n"); 
   return -1;
   }
   
   return 0;
}

static int scale_task_wait_response(const uint16_t expect_cmd, uint8_t *const response,const uint16_t timeout)
{
  int rc;
  uint8_t  length_to_read,read_length;
  uint16_t crc_calculated,crc_received;
  uint8_t  step;
  uint8_t  recv_buffer[SCALE_TASK_RECV_BUFFER_SIZE];
  int16_t  *net_weight;
  uint16_t  wait_time;

  length_to_read = 2;
  read_length =0;
  step = SCALE_TASK_ADU_STEP;
  wait_time = timeout;
  while(length_to_read != 0){
  rc = serial_select(scale_serial_handle,wait_time);
  if(rc == -1){
   log_error("scale select error.\r\n");
   return -1;
  }
  if(rc == 0){
   log_error("scale select timeout.\r\n");
   return -1;
  }
  
  rc = serial_read(scale_serial_handle,recv_buffer + read_length,length_to_read);
  if(rc == -1) {
   log_error("scale read error.\r\n");
   return -1;
  }
     
  for (int i=0; i < rc; i++){
  log_debug("<%2X>\r\n", recv_buffer[read_length + i]);
  }
   
  read_length +=rc;
  length_to_read -=rc;
   
  if(length_to_read == 0){
     switch(step){
     /*接收到了协议头和数据长度域*/
     case SCALE_TASK_ADU_STEP:
       if(recv_buffer[SCALE_TASK_SOF_OFFSET] == SCALE_TASK_SOF_VALUE){
        step = SCALE_TASK_CRC_STEP;             
        length_to_read = recv_buffer[SCALE_TASK_LEN_OFFSET] - SCALE_TASK_LEN_LEN;              
        
        if(length_to_read == 0){
        log_error("scale err in len value:%d.\r\n",recv_buffer[SCALE_TASK_LEN_OFFSET]);
        return -1;
        }
       }else{
         log_error("scale err in addr value:%d.\r\n",recv_buffer[SCALE_TASK_SOF_OFFSET]);
        return -1;
       } 
                                                                    
     break;
     /*接收完成了全部的数据*/
     case SCALE_TASK_CRC_STEP:
       crc_calculated = scale_task_crc16(recv_buffer + 1,read_length - SCALE_TASK_CRC_LEN - SCALE_TASK_SOF_LEN);
       crc_received = recv_buffer[read_length-1]<< 8 | recv_buffer[read_length-2];
       if(crc_calculated != crc_received){
          log_error("scale err in crc.recv:%d calculate:%d.\r\n",crc_received,crc_calculated);
          return -1;
       }
       
       if(recv_buffer[SCALE_TASK_ADDR_OFFSET] != SCALE_TASK_ADDR_VALUE){
       log_error("scale err in addr:%d.\r\n",recv_buffer[SCALE_TASK_ADDR_OFFSET]);
       return -1;
       }
       
      if( *(uint16_t*)&recv_buffer[SCALE_TASK_CMD_OFFSET] != expect_cmd){
        log_error("scale err in response cmd.expect:%d.recv:%d.\r\n",expect_cmd,*(uint16_t*)&recv_buffer[SCALE_TASK_CMD_OFFSET]); 
        return -1;
      }
      
       /*全部解析正确*/      
       /*设置传感器的回应*/
       if( *(uint16_t*)&recv_buffer[SCALE_TASK_CMD_OFFSET] == SCALE_TASK_CMD_SET_SENSOR_VALUE){
        if(recv_buffer[read_length-3] != SCALE_TASK_STATUS_OK_VALUE){
        log_error("scale set sensor err in status:%d.\r\n",recv_buffer[read_length-3]);
        *response = SCALTE_TASK_SET_SENSOR_FAILURE;
        }else{
        *response = SCALTE_TASK_SET_SENSOR_SUCCESS;
        }
       return 0;
       }
       
       /*获取净重的回应*/
       if(*(uint16_t*)&recv_buffer[SCALE_TASK_CMD_OFFSET] == SCALE_TASK_CMD_GET_NET_WEIGHT_VALUE){
        if(recv_buffer[read_length-3] != SCALE_TASK_STATUS_OK_VALUE){
        log_error("scale net weight err in status:%d.\r\n",recv_buffer[read_length-3]);
        return -1;
        }
        net_weight = (int16_t*)response;
        for(uint8_t i=0;i< SCALE_TASK_SCALE_CNT;i++){
        net_weight[i] = recv_buffer[SCALE_TASK_NET_WEIGHT_OFFSET + 2 *i] << 8 | recv_buffer[SCALE_TASK_NET_WEIGHT_OFFSET + 2 * i + 1];
        if(net_weight[i] == -1){
        net_weight[i] = 0;
        }
        }
        return 0;
       }
       
       break;
       default:
       break;
       }
     }
    wait_time = SCALE_TASK_CHARACTER_TIMEOUT;
    }
  
  return -1;
  }




void scale_task(void const * argument)
{
 osStatus   status;
 osEvent    os_msg;
 task_msg_t *msg;
 uint16_t   crc_calculated;
 int rc; 

 int16_t  net_weight[SCALE_TASK_SCALE_CNT];
 uint8_t  sensor_result;

 osMessageQDef(scale_task_msg_q,2,uint32_t);
 scale_task_msg_q_id = osMessageCreate(osMessageQ(scale_task_msg_q),scale_task_hdl);
 log_assert(scale_task_msg_q_id); 
 
 rc = serial_create(&scale_serial_handle,SCALE_TASK_RX_BUFFER_SIZE,SCALE_TASK_TX_BUFFER_SIZE);
 log_assert(rc == 0);
 rc = serial_register_hal_driver(scale_serial_handle,&scale_serial_driver);
 log_assert(rc == 0);
 
 rc = serial_open(scale_serial_handle,
                  SCALE_TASK_SERIAL_PORT,
                  SCALE_TASK_SERIAL_BAUDRATES,
                  SCALE_TASK_SERIAL_DATABITS,
                  SCALE_TASK_SERIAL_STOPBITS);
 
 log_assert(rc == 0); 

 /*等待任务同步*/
 xEventGroupSync(tasks_sync_evt_group_hdl,TASKS_SYNC_EVENT_SCALE_TASK_RDY,TASKS_SYNC_EVENT_ALL_TASKS_RDY,osWaitForever);
 log_debug("scale task sync ok.\r\n");
 
 while(1){
 os_msg = osMessageGet(scale_task_msg_q_id,SCALE_TASK_MSG_WAIT_TIMEOUT_VALUE);
 if(os_msg.status == osEventMessage){
 msg = (task_msg_t *)os_msg.value.v;
 
 /*设置传感器*/
 if(msg->type == REQ_SET_SENSOR){ 
  
  set_sensor[SCALE_TASK_SENSOR_VALUE_OFF] = msg->sensor_value;
  crc_calculated = scale_task_crc16(set_sensor + 1,SCALE_TASK_SET_SENSOR_LEN - SCALE_TASK_CRC_LEN - SCALE_TASK_SOF_LEN);
  set_sensor[SCALE_TASK_SET_SENSOR_LEN - 1] = crc_calculated >> 8;
  set_sensor[SCALE_TASK_SET_SENSOR_LEN - 2] = crc_calculated ;

  rc = scale_task_req(set_sensor,SCALE_TASK_SET_SENSOR_LEN,SCALE_TASK_SEND_TIMEOUT);
  if(rc != 0){
  log_error("set sensor error.\r\n");
  continue;  
  }
  rc = scale_task_wait_response(SCALE_TASK_CMD_SET_SENSOR_VALUE,&sensor_result,SCALE_TASK_SET_SENSOR_TIMEOUT);
  if(rc != 0){
  continue;  
  }
  
  if(sensor_result == SCALTE_TASK_SET_SENSOR_SUCCESS){
  log_debug("set sensor ok.\r\n");
  }
  /*回复操作结果*/
  /*
  protocol_msg.type = RESPONSE_SET_SENSOR;
  protocol_msg.sensor_result = sensor_result;
  status = osMessagePut(protocol_task_msg_q_id,(uint32_t)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
  if(status != osOK){
  log_error("scale put sensor result msg err:%d.\r\n",status);
  }
 */
 }
 
 if(msg->type == REQ_NET_WEIGHT){ 
  
  rc = scale_task_req(get_net_weight,SCALE_TASK_GET_NET_WEIGHT_LEN,SCALE_TASK_SEND_TIMEOUT );
  if(rc != 0){
  log_error("set sensor error.\r\n");
  continue;  
  }
  rc = scale_task_wait_response(SCALE_TASK_CMD_GET_NET_WEIGHT_VALUE,(uint8_t*)net_weight,SCALE_TASK_GET_NET_WEIGHT_TIMEOUT);
  if(rc != 0){
  continue;  
  }

  /*回复净重值*/
  protocol_msg.type = RESPONSE_NET_WEIGHT;
  protocol_msg.net_weight = net_weight;
  status = osMessagePut(protocol_task_msg_q_id,(uint32_t)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
  if(status != osOK){
  log_error("scale put net weight msg err:%d.\r\n",status);
  }
  
 }
 /*获取称的数量*/
 if(msg->type == REQ_SCALE_CNT){ 
 protocol_msg.type = RESPONSE_SCALE_CNT;
 protocol_msg.scale_cnt = SCALE_TASK_SCALE_CNT;
 status = osMessagePut(protocol_task_msg_q_id,(uint32_t)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
 if(status != osOK){
 log_error("scale put scale cnt msg err:%d.\r\n",status);
 }
 }
 
 /*获取厂家id*/
 if(msg->type == REQ_MANUFACTURE_ID){ 
 protocol_msg.type = RESPONSE_MANUFACTURE_ID;
 protocol_msg.manufacture_id = SCALE_TASK_MANUFACTURE_ID;
 status = osMessagePut(protocol_task_msg_q_id,(uint32_t)&protocol_msg,SCALE_TASK_MSG_PUT_TIMEOUT_VALUE);
 if(status != osOK){
 log_error("scale put id msg err:%d.\r\n",status);
 }
 }
 
 }
 
 
 }
}
