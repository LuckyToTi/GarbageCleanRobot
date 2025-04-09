#ifndef __CONFIG_DEF_H_
#define __CONFIG_DEF_H_

#include "stm32f4xx.h"

#define config_str      1
#define config_uint8_t  2
#define config_uint16_t 3
#define config_uint32_t 4
#define config_float    5
#define config_int      6

struct NetConfig {
    char ip_addr[16];
    char gw_addr[16];
    char msk_addr[16];
};

struct RobolinkIdConfig {
    uint8_t local_id;
    uint8_t remote_id;
};

struct RobolinkUdpConfig {
    uint16_t local_port;
    char remote_ip_addr[16];
    uint16_t remote_port;
};

struct UartConfig {
    uint16_t function;
    uint32_t baundrate;
};

struct TcpConfig {
    char ip_addr[16];
    uint16_t port;
};

struct WaterDepthSensorConfig {
    int air_press;
    uint16_t air_press_custom_switch;
};

struct PidConfig {
    float p;
    float i;
    float d;
    float fdlimit;
    float fdout;
};

struct StrAdd{
    char str[30];
    char mem[30];
    char mem2[30];
    void* member_ptr;
    int  ptr_type; // 1.字符串 2.uint8_t 3.uin16_t 4.uint32_t 5.float
};

#endif  // __CONFIG_DEF_H_
