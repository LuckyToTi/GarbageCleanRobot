#ifndef __ROBOLINK_H_
#define __ROBOLINK_H_

#include "robolink_def.h"
#include "stm32f4xx.h"

struct robolink_raw_msg
{
    uint8_t* msg;
    unsigned int len;
};

struct robolink_raw_msg* robolink_create_init_new_msg(uint8_t data_len, uint8_t sys_id, uint8_t dev_id, uint8_t data_id);
void robolink_free_unused_msg(struct robolink_raw_msg* msg);
void robolink_generate_cnt_autoincrement(struct robolink_raw_msg* msg);
void robolink_generate_crc(struct robolink_raw_msg* msg);
uint8_t* robolink_get_data_segment_ptr(struct robolink_raw_msg* msg);
int robolink_data_check(uint8_t* buffer, uint16_t len);
int robolink_is_match_id(uint8_t sys_id, uint8_t dev_id, uint8_t data_id, uint8_t* buffer, uint16_t len);

#endif  // __ROBOLINK_H_
