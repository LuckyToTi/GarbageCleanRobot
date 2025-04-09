#include "robolink.h"
#include <rthw.h>
#include "thirdparty/crc-lib-c/crcLib.h"  // CRC校验库

#define malloc_(size)  rt_malloc(size)
#define free_(ptr)     rt_free(ptr)

struct robolink_raw_msg* robolink_create_new_msg(unsigned int msg_len)
{
    struct robolink_raw_msg* new_msg = NULL;
    new_msg = (struct robolink_raw_msg*) malloc_(sizeof(struct robolink_raw_msg));
    if(new_msg != NULL)
    {
        new_msg->msg = NULL;
        new_msg->len = msg_len;
        new_msg->msg = (uint8_t*) malloc_(msg_len);
        if(new_msg->msg == NULL)
        {
            free_(new_msg);
            new_msg = NULL;
        }
    }
    return new_msg;
}

void robolink_free_unused_msg(struct robolink_raw_msg* msg)
{
    if(msg == NULL)
        return;
    if(msg->msg != NULL)
        free_(msg->msg);
    free_(msg);
}

int32_t robolink_init_msg(struct robolink_raw_msg* msg, uint8_t data_len,
        uint8_t sys_id, uint8_t dev_id, uint8_t data_id)
{
    if(msg == NULL) return -1;
    if(msg->len == 0) return -2;
    if(data_len != msg->len-8) return -3;
    msg->msg[0] = 0xB7;
    msg->msg[2] = data_len;
    msg->msg[3] = sys_id;
    msg->msg[4] = dev_id;
    msg->msg[5] = data_id;
    return 0;
}

void robolink_generate_cnt_autoincrement(struct robolink_raw_msg* msg)
{
    static uint8_t send_cnt = 0;
    msg->msg[1] = send_cnt;  // 计数位
    send_cnt++;
}

void robolink_generate_crc(struct robolink_raw_msg* msg)
{
    uint16_t crc16_ibm_value = crc16_ibm(msg->msg, msg->len-2);
    msg->msg[msg->len-2] = crc16_ibm_value & 0x00FF;
    msg->msg[msg->len-1] = (crc16_ibm_value & 0xFF00) >> 8;
}

struct robolink_raw_msg* robolink_create_init_new_msg(uint8_t data_len,
        uint8_t sys_id, uint8_t dev_id, uint8_t data_id)
{
    struct robolink_raw_msg* new_msg = NULL;
    new_msg = robolink_create_new_msg((uint32_t)data_len + 8u);
    if(new_msg == NULL) return NULL;
    if( robolink_init_msg(new_msg, data_len, sys_id, dev_id, data_id) != 0)
    {
        robolink_free_unused_msg(new_msg);
        return NULL;
    }
    return new_msg;
}

inline uint8_t* robolink_get_data_segment_ptr(struct robolink_raw_msg* msg)
{
    return (msg->msg+6);
}

int robolink_data_check(uint8_t* buffer, uint16_t len)
{
    if(buffer[0] != 0xB7) return -1;
    if(buffer[2] != (len-8)) return -2;
    uint16_t crc16imb_get = (buffer[len-1] << 8) | buffer[len-2];
    uint16_t crc16imb_calc = crc16_ibm(buffer, len-2);
    if(crc16imb_get != crc16imb_calc) return -3;
    return buffer[2];
}

inline int robolink_is_match_id(uint8_t sys_id, uint8_t dev_id, uint8_t data_id, uint8_t* buffer, uint16_t len)
{
    return ((buffer[3] == sys_id) && (buffer[4] == dev_id) && (buffer[5] == data_id));
}
