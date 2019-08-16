/**
 * Free YModem implementation.
 *
 * Fredrik Hederstierna 2014
 *
 * This file is in the public domain.
 * You can do whatever you want with it.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

/**
 * Some future improvements and ideas
 *
 * Add rx/tx callbacks if eg storing file to external storage,
 * and full buffer cannot be in memory at the same time.
 *
 * Add support for Ymodem-G standard as described in docs for streaming.
 *
 * Add suppport for async operation mode if calling system cannot handle
 * that calls are synchroneus and blocking.
 *
 * Add unittest
 *
 * Add support for ZModem (which may imply total rewrite).
 */

#include <fymodem.h>
#include "flash_if.h"
#include "xtimer.h"
#include "log.h"


/* filesize 999999999999999 should be enough... */
#define YM_FILE_SIZE_LENGTH        (16)

/* packet constants */
#define YM_PACKET_START_INDEX      (1)
#define YM_PACKET_SEQNO_INDEX      (1 + 1) /* start*/
#define YM_PACKET_SEQNO_COMP_INDEX (2 + 1) /* start, block*/
#define YM_PACKET_HEADER           (3 + 1) /* start, block, block-complement */
#define YM_PACKET_TRAILER          (2)     /* CRC bytes */
#define YM_PACKET_OVERHEAD         (YM_PACKET_HEADER + YM_PACKET_TRAILER)
#define YM_PACKET_SIZE             (128)
#define YM_PACKET_1K_SIZE          (1024)
#define YM_PACKET_RX_TIMEOUT_MS    (1000)
#define YM_PACKET_ERROR_MAX_NBR    (5)

/* contants defined by YModem protocol */
#define YM_SOH                     (0x01)  /* start of 128-byte data packet */
#define YM_STX                     (0x02)  /* start of 1024-byte data packet */
#define YM_EOT                     (0x04)  /* End Of Transmission */
#define YM_ACK                     (0x06)  /* ACKnowledge, receive OK */
#define YM_NAK                     (0x15)  /* Negative ACKnowledge, receiver ERROR, retry */
#define YM_CAN                     (0x18)  /* two CAN in succession will abort transfer */
#define YM_CRC                     (0x43)  /* 'C' == 0x43, request 16-bit CRC, use in place of first NAK for CRC mode */
#define YM_ABT1                    (0x41)  /* 'A' == 0x41, assume try abort by user typing */
#define YM_ABT2                    (0x61)  /* 'a' == 0x61, assume try abort by user typing */


/*包数据*/
#pragma pack (4)
static uint8_t rx_packet_data[YM_PACKET_1K_SIZE + YM_PACKET_HEADER + YM_PACKET_TRAILER];
#pragma pack ()

/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int __ym_sleep_ms(uint32_t ms)
{
    return 0;
}

/*
* @brief 
* @param
* @param
* @return 
* @note
*/

static void __ym_flush(xuart_handle_t *handle)
{
   xuart_clear(handle);
}
/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int __ym_getchar(xuart_handle_t *handle,uint32_t timeout)
{
    int rc;
    uint8_t c;
    xtimer_t timer;
    
    xtimer_init(&timer,0,timeout);


    rc = xuart_select(handle,xtimer_value(&timer));
    if (rc == 0) {
        log_error("ymodem select timeout.\r\n");
        return -1;
    }
    if (rc < 0) {
        log_error("ymodem select err.\r\n");
        return -1;
    }

    rc = xuart_read(handle,&c,1);
    if (rc <= 0) {
        log_error("ymodem read err.\r\n");
        return -1;
    }

    return c;
}


/*
* @brief 
* @param
* @param
* @return 
* @note
*/
static int __ym_putchar(xuart_handle_t *handle,char c)
{
    int rc;

    rc = xuart_write(handle,(const uint8_t*)&c,1);
    if (rc != 1) {
        log_error("ymodem write err.\r\n");
        return -1;
    }
    rc = xuart_complete(handle,5);
    if (rc != 0) {
        log_error("ymodem complete err.\r\n");
        return -1;
    }

    return 0;



}
/* ------------------------------------------------ */
/* calculate crc16-ccitt very fast
   Idea from: http://www.ccsinfo.com/forum/viewtopic.php?t=24977
*/
static uint16_t ym_crc16(const uint8_t *buf, uint16_t len) 
{
  uint16_t x;
  uint16_t crc = 0;
  while (len--) {
    x = (crc >> 8) ^ *buf++;
    x ^= x >> 4;
    crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
  }
  return crc;
}

/* ------------------------------------------------- */
/* write 32bit value as asc to buffer, return chars written. */
static uint32_t ym_writeU32(uint32_t val, uint8_t *buf)
{
  uint32_t ci = 0;
  if (val == 0) {
    /* If already zero then just return zero */
    buf[ci++] = '0';
  }
  else {
    /* Maximum number of decimal digits in uint32_t is 10, add one for z-term */
    uint8_t s[11];
    int32_t i = sizeof(s) - 1;
    /* z-terminate string */
    s[i] = 0;
    while ((val > 0) && (i > 0)) {
      /* write decimal char */
      s[--i] = (val % 10) + '0';
      val /= 10;
    }
    uint8_t *sp = &s[i];
    /* copy results to out buffer */
    while (*sp) {
      buf[ci++] = *sp++;
    }
  }
  /* z-term */
  buf[ci] = 0;
  /* return chars written */
  return ci;
}

/* ------------------------------------------------- */
/* read 32bit asc value from buffer */
static void ym_readU32(const uint8_t* buf, uint32_t *val)
{
  const uint8_t *s = buf;
  uint32_t res = 0;
  uint8_t c;  
  /* trim and strip leading spaces if any */
  do {
    c = *s++;
  } while (c == ' ');
  while ((c >= '0') && (c <= '9')) {
    c -= '0';
    res *= 10;
    res += c;
    /* next char */
    c = *s++;
  }
  *val = res;
}

/* -------------------------------------------------- */
/**
  * Receive a packet from sender
  * @param rxlen
  *     0: end of transmission
  *    -1: abort by sender
  *    >0: packet length
  * @return 0: normally return, success
  *        -1: timeout or packet error
  *         1: abort by user / corrupt packet
  */
static int32_t ym_rx_packet(xuart_handle_t *handle,
                            uint8_t *rxdata,
                            int32_t *rxlen,
                            uint32_t packets_rxed,
                            uint32_t timeout_ms)
{
  *rxlen = 0;
  
  int32_t c = __ym_getchar(handle,timeout_ms);
  if (c < 0) {
    /* end of stream */
    return -1;
  }

  uint32_t rx_packet_size;

  switch (c) {
  case YM_SOH:
    rx_packet_size = YM_PACKET_SIZE;
    break;
  case YM_STX:
    rx_packet_size = YM_PACKET_1K_SIZE;
    break;
  case YM_EOT:
    /* ok */
    return 0;
  case YM_CAN:
    c = __ym_getchar(handle,timeout_ms);
    if (c == YM_CAN) {
      *rxlen = -1;
      /* ok */
      return 0;
    }
    /* fall-through */
  case YM_CRC:
    if (packets_rxed == 0) {
      /* could be start condition, first byte */
      return 1;
    }
   /* fall-through */
  case YM_ABT1:
  case YM_ABT2:
    /* User try abort, 'A' or 'a' received */
    return 1;
  default:
    /* This case could be the result of corruption on the first octet
       of the packet, but it's more likely that it's the user banging
       on the terminal trying to abort a transfer. Technically, the
       former case deserves a NAK, but for now we'll just treat this
       as an abort case. */
    *rxlen = -1;
    return 0;
  }
  
  /* store data RXed */
  rxdata[YM_PACKET_START_INDEX] = (uint8_t)c;
  
  uint32_t i;
  for (i = YM_PACKET_SEQNO_INDEX; i < (rx_packet_size + YM_PACKET_OVERHEAD); i++) {
    c = __ym_getchar(handle,timeout_ms);
    if (c < 0) {
      /* end of stream */
      return -1;
    }
    /* store data RXed */
    rxdata[i] = (uint8_t)c;
  }
  
  /* just a sanity check on the sequence number/complement value.
     caller should check for in-order arrival. */
  uint8_t seq_nbr = (rxdata[YM_PACKET_SEQNO_INDEX] & 0xff);
  uint8_t seq_cmp = ((rxdata[YM_PACKET_SEQNO_COMP_INDEX] ^ 0xff) & 0xff);
  if (seq_nbr != seq_cmp) {
    /* seq nbr error */
    return 1;
  }
  
  /* check CRC16 match */
  uint16_t crc16_val = ym_crc16((const unsigned char *)(rxdata + YM_PACKET_HEADER),
                                rx_packet_size + YM_PACKET_TRAILER);
  if (crc16_val) {
    /* CRC error, non zero */
    return 1;
  }
  *rxlen = rx_packet_size;
  /* success */
  return 0;
}

/* ------------------------------------------------- */
/**
 * Receive a file using the ymodem protocol
 * @param rxdata Pointer to the first byte
 * @param rxlen  Max in length
 * @return The length of the file received, or 0 on error
 */
int32_t fymodem_receive(xuart_handle_t *handle,
                        uint32_t flash_addr,
                        uint32_t flash_limit,
                        char *filename,
                        uint32_t wait_timeout)
{
  int32_t rx_packet_len;
  uint32_t timeout = 0;

  uint8_t filesize_asc[YM_FILE_SIZE_LENGTH];
  uint32_t filesize = 0;

  bool first_try = true;
  bool session_done = false;

  uint32_t nbr_errors = 0;
  uint32_t recv_size = 0;
  /* z-term string */
  filename[0] = 0;
  
  /* receive files */
  do {
    if (!first_try) {
      __ym_putchar(handle,YM_CRC);
    }
    first_try = false;

    bool crc_nak = true;
    bool file_done = false;
    uint32_t packets_rxed = 0;

    do {
      /* receive packets */
      int32_t res = ym_rx_packet(handle,
                                 rx_packet_data,
                                 &rx_packet_len,
                                 packets_rxed,
                                 YM_PACKET_RX_TIMEOUT_MS);
      switch (res) {
      case 0: {
        nbr_errors = 0;
        switch (rx_packet_len) {
        case -1: {
          /* aborted by sender */
          __ym_putchar(handle,YM_ACK);
          return 0;
        }
        case 0: {
          /* EOT - End Of Transmission */
          __ym_putchar(handle,YM_ACK);
          /* TODO: Add some sort of sanity check on the number of
             packets received and the advertised file length. */
          file_done = true;
          /* TODO: set first_try = false; here to resend C ? */
          break;
        }
        default: {
          /* normal packet, check seq nbr */
          uint8_t seq_nbr = rx_packet_data[YM_PACKET_SEQNO_INDEX];
          if (seq_nbr != (packets_rxed & 0xff)) {
            /* wrong seq number */
            __ym_putchar(handle,YM_NAK);
          } else {
            if (packets_rxed == 0) {
              /* The spec suggests that the whole data section should
                 be zeroed, but some senders might not do this.
                 If we have a NULL filename and the first few digits of
                 the file length are zero, then call it empty. */
              int32_t i;
              for (i = YM_PACKET_HEADER; i < YM_PACKET_HEADER + 4; i++) {
                if (rx_packet_data[i] != 0) {
                  break;
                }
              }
              /* non-zero bytes found in header, filename packet has data */
              if (i < YM_PACKET_HEADER + 4) {
                /* read file name */
                uint8_t *file_ptr = (uint8_t*)(rx_packet_data + YM_PACKET_HEADER);
                i = 0;
                while (*file_ptr && (i < FYMODEM_FILE_NAME_MAX_LENGTH)) {
                  filename[i++] = *file_ptr++;
                }
                filename[i++] = '\0';
                file_ptr++;
                /* read file size */
                i = 0;
                while ((*file_ptr != ' ') && (i < YM_FILE_SIZE_LENGTH)) {
                  filesize_asc[i++] = *file_ptr++;
                }
                filesize_asc[i++] = '\0';
                /* convert file size */
                ym_readU32(filesize_asc, &filesize);
                /* check file size */
                if (filesize > flash_limit) {
                  log_error("YM: RX buffer too small (0x%08x vs 0x%08x)\n", (unsigned int)flash_limit, (unsigned int)filesize);
                  goto rx_err_handler;
                }
                /* erase user application area */
                flash_if_erase(flash_addr,flash_limit);

                __ym_putchar(handle,YM_ACK);
                __ym_putchar(handle,crc_nak ? YM_CRC : YM_NAK);
                crc_nak = false;
              }
              else {
                /* filename packet is empty, end session */
                __ym_putchar(handle,YM_ACK);
                file_done = true;
                session_done = true;
                break;
              }
            }
            else {
              /* This shouldn't happen, but we check anyway in case the
                 sender sent wrong info in its filename packet */
              if ((recv_size + rx_packet_len) > (int32_t)flash_limit) {
                log_error("YM: RX buffer overflow (exceeded 0x%08x)\n", (unsigned int)flash_limit);
                goto rx_err_handler;
              }
              /*flash program */
              if (flash_if_write(flash_addr + recv_size,&rx_packet_data[YM_PACKET_HEADER],rx_packet_len) == 0) {
                    recv_size += rx_packet_len;
                    __ym_putchar(handle,YM_ACK);
              } else {
                log_error("flash write err in addr:0x%08x size:%d.\r\n",flash_addr + recv_size,rx_packet_len);
                goto rx_err_handler;
             }
            }
            packets_rxed++;
          }  /* sequence number check ok */
        } /* default */
        } /* inner switch */
        break;
      } /* case 0 */
      default: {
        if (packets_rxed > 0) {
          nbr_errors++;
          if (nbr_errors >= YM_PACKET_ERROR_MAX_NBR) {
            log_error("YM: RX errors too many: %d - ABORT.\n", (unsigned int)nbr_errors);
            goto rx_err_handler;
          }
        } else {/*未开始接收*/
            timeout += YM_PACKET_RX_TIMEOUT_MS;
            if (timeout >= wait_timeout) {
                log_error("YM: wait timeout: %d ms- ABORT.\n",timeout);
                goto rx_err_handler;
            }
        }
        __ym_putchar(handle,YM_CRC);
        break;
      } /* default */
      } /* switch */
      
      /* check end of receive packets */
    } while (! file_done);

    /* check end of receive files */
  } while (! session_done);

  /* return bytes received */
  return filesize;

 rx_err_handler:
  __ym_putchar(handle,YM_CAN);
  __ym_putchar(handle,YM_CAN);
  __ym_sleep_ms(1000);
  return -1;
}

/* ------------------------------------ */
static void ym_send_packet(xuart_handle_t *handle,
                           uint8_t *txdata,
                           int32_t block_nbr)
{
  int32_t tx_packet_size;

  /* We use a short packet for block 0, all others are 1K */
  if (block_nbr == 0) {
    tx_packet_size = YM_PACKET_SIZE;
  }
  else {
    tx_packet_size = YM_PACKET_1K_SIZE;
  }

  uint16_t crc16_val = ym_crc16(txdata, tx_packet_size);
  
  /* For 128 byte packets use SOH, for 1K use STX */
  __ym_putchar(handle, (block_nbr == 0) ? YM_SOH : YM_STX );
  /* write seq numbers */
  __ym_putchar(handle,block_nbr & 0xFF);
  __ym_putchar(handle,~block_nbr & 0xFF);
  
  /* write txdata */
  int32_t i;
  for (i = 0; i < tx_packet_size; i++) {
    __ym_putchar(handle,txdata[i]);
  }

  /* write crc16 */
  __ym_putchar(handle,(crc16_val >> 8) & 0xFF);
  __ym_putchar(handle,crc16_val & 0xFF);
}

/* ----------------------------------------------- */
/* Send block 0 (the filename block), filename might be truncated to fit. */
static void ym_send_packet0(xuart_handle_t *handle,
                            const char* filename,
                            int32_t filesize)
{
  int32_t pos = 0;
  /* put 256byte on stack, ok? reuse other stack mem? */
  uint8_t block[YM_PACKET_SIZE];
  if (filename) {
    /* write filename */
    while (*filename && (pos < YM_PACKET_SIZE - YM_FILE_SIZE_LENGTH - 2)) {
      block[pos++] = *filename++;
    }
    /* z-term filename */
    block[pos++] = 0;
    
    /* write size, TODO: check if buffer can overwritten here. */
    pos += ym_writeU32(filesize, &block[pos]);
  }

  /* z-terminate string, pad with zeros */
  while (pos < YM_PACKET_SIZE) {
    block[pos++] = 0;
  }
  
  /* send header block */
  ym_send_packet(handle,block, 0);
}

/* ------------------------------------------------- */
static void ym_send_data_packets(xuart_handle_t *handle,
                                 uint8_t* txdata,
                                 uint32_t txlen,
                                 uint32_t timeout_ms)
{
  int32_t block_nbr = 1;
  
  while (txlen > 0) {
    /* check if send full 1k packet */
    uint32_t send_size;
    if (txlen > YM_PACKET_1K_SIZE) {
      send_size = YM_PACKET_1K_SIZE;
    } else {
      send_size = txlen;
    }
    /* send packet */
    ym_send_packet(handle,txdata, block_nbr);
    int32_t c = __ym_getchar(handle,timeout_ms);
    switch (c) {
    case YM_ACK: {
      txdata += send_size;
      txlen  -= send_size;
      block_nbr++;
      break;
    }
    case -1:
    case YM_CAN: {
      return;
    }
    default:
      break;
    }
  }
  
  int32_t ch;
  do {
    __ym_putchar(handle,YM_EOT);
    ch = __ym_getchar(handle,timeout_ms);
  } while ((ch != YM_ACK) && (ch != -1));
  
  /* send last data packet */
  if (ch == YM_ACK) {
    ch = __ym_getchar(handle,timeout_ms);
    if (ch == YM_CRC) {
      do {
        ym_send_packet0(handle,0, 0);
        ch = __ym_getchar(handle,timeout_ms);
      } while ((ch != YM_ACK) && (ch != -1));
    }
  }
}

/* ------------------------------------------------------- */
int32_t fymodem_send(xuart_handle_t *handle,uint8_t* txdata, size_t txsize, const char* filename)
{
  bool crc_nak = true;
  bool file_done = false;

  /* flush the RX FIFO, after a cool off delay */
  __ym_sleep_ms(1000);
  __ym_flush(handle);

  /* not in the specs, send CRC here just for balance */
  int32_t ch;
  do {
    __ym_putchar(handle,YM_CRC);
    ch = __ym_getchar(handle,1000);
  } while (ch < 0);
  
  /* we do require transfer with CRC */
  if (ch != YM_CRC) {
    goto tx_err_handler;
  }

  do {
    ym_send_packet0(handle,filename, txsize);
    /* When the receiving program receives this block and successfully
       opened the output file, it shall acknowledge this block with an ACK
       character and then proceed with a normal XMODEM file transfer
       beginning with a "C" or NAK tranmsitted by the receiver. */
    ch = __ym_getchar(handle,YM_PACKET_RX_TIMEOUT_MS);
    if (ch == YM_ACK) {
      ch = __ym_getchar(handle,YM_PACKET_RX_TIMEOUT_MS);
      if (ch == YM_CRC) {
        ym_send_data_packets(handle,txdata, txsize, YM_PACKET_RX_TIMEOUT_MS);
        /* success */
        file_done = true;
      }
    }
    else if ((ch == YM_CRC) && (crc_nak)) {
      crc_nak = false;
      continue;
    }
    else if ((ch != YM_NAK) || (crc_nak)) {
      goto tx_err_handler;
    }
  } while (! file_done);

  return txsize;

 tx_err_handler:
  __ym_putchar(handle,YM_CAN);
  __ym_putchar(handle,YM_CAN);
  __ym_sleep_ms(1000);
  return 0;
}

