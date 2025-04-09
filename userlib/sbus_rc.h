#ifndef __SBUS_RC_H_
#define __SBUS_RC_H_

#define SBUS_RAW_DATA_SIZE (25u)

struct SbusRc
{
    uint8_t head;
    uint8_t flag;
    uint8_t end;
    uint16_t ch[16];
};

static inline uint8_t sbus_rc_raw_check_data(const uint8_t* raw_buf)
{
    return ((raw_buf[0] != 0x0F) || (raw_buf[24] != 0x00));
}

static inline uint8_t sbus_rc_raw_check_size(const int get_size)
{
    return (SBUS_RAW_DATA_SIZE != get_size);
}

static inline void sbus_rc_parse(const uint8_t* sbus_rx_buf, struct SbusRc* sbus_rc)
{
    sbus_rc->head = sbus_rx_buf[0];             // 首部
    sbus_rc->flag = sbus_rx_buf[23];            // 标志符
    sbus_rc->end  = sbus_rx_buf[24];            // 结尾
    sbus_rc->ch[0] =((sbus_rx_buf[2]<<8)  + (sbus_rx_buf[1])) & 0x07ff;
    sbus_rc->ch[1] =((sbus_rx_buf[3]<<5)  + (sbus_rx_buf[2]>>3)) & 0x07ff;
    sbus_rc->ch[2] =((sbus_rx_buf[5]<<10) + (sbus_rx_buf[4]<<2) + (sbus_rx_buf[3]>>6)) & 0x07ff;
    sbus_rc->ch[3] =((sbus_rx_buf[6]<<7)  + (sbus_rx_buf[5]>>1)) & 0x07ff;
    sbus_rc->ch[4] =((sbus_rx_buf[7]<<4)  + (sbus_rx_buf[6]>>4)) & 0x07ff;
    sbus_rc->ch[5] =((sbus_rx_buf[9]<<9)  + (sbus_rx_buf[8]<<1) + (sbus_rx_buf[7]>>7)) & 0x07ff;
    sbus_rc->ch[6] =((sbus_rx_buf[10]<<6) + (sbus_rx_buf[9]>>2)) & 0x07ff;
    sbus_rc->ch[7] =((sbus_rx_buf[11]<<3) + (sbus_rx_buf[10]>>5)) & 0x07ff;
    sbus_rc->ch[8] =((sbus_rx_buf[13]<<8)  + sbus_rx_buf[12]) & 0x07ff;
    sbus_rc->ch[9] =((sbus_rx_buf[14]<<5)  + (sbus_rx_buf[13]>>3)) & 0x07ff;
    sbus_rc->ch[10]=((sbus_rx_buf[16]<<10) + (sbus_rx_buf[15]<<2) + (sbus_rx_buf[14]>>6)) & 0x07ff;
    sbus_rc->ch[11]=((sbus_rx_buf[17]<<7)  + (sbus_rx_buf[16]>>1)) & 0x07ff;
    sbus_rc->ch[12]=((sbus_rx_buf[18]<<4)  + (sbus_rx_buf[17]>>4)) & 0x07ff;
    sbus_rc->ch[13]=((sbus_rx_buf[20]<<9)  + (sbus_rx_buf[19]<<1) + (sbus_rx_buf[18]>>7)) & 0x07ff;
    sbus_rc->ch[14]=((sbus_rx_buf[21]<<6) + (sbus_rx_buf[20]>>2)) & 0x07ff;
    sbus_rc->ch[15]=((sbus_rx_buf[22]<<3) + (sbus_rx_buf[21]>>5)) & 0x07ff;
}

static inline void sbus_rc_reset(struct SbusRc* sbus_rc)
{
    for(int i=0; i<16; i++) {
        sbus_rc->ch[i] = 1000;
    }
    sbus_rc->head = 0x0F;
    sbus_rc->flag = 0x00;
    sbus_rc->end  = 0x00;
}

static inline int sbus_rc_check_parse(const uint8_t* sbus_rx_buf, const int get_size, struct SbusRc* sbus_rc)
{
    if(sbus_rc_raw_check_size(get_size))
        return -1;
    if(sbus_rc_raw_check_data(sbus_rx_buf))
        return -2;
    sbus_rc_parse(sbus_rx_buf, sbus_rc);
    return 0;
}

#endif  // __SBUS_RC_H_
