#ifndef __CONFIG_H_
#define __CONFIG_H_

#include "config_def.h"
#include "userlib/pid2.h"

// 配置项添加直接往下添加 调整结构体会导致之前的存储读取错位
struct Config {
    struct NetConfig net_config;
    struct RobolinkIdConfig robolink_id_config;
    struct RobolinkUdpConfig robolink_udp_config;
    struct TcpConfig robot_msg_tcp_svr_config;
    struct UartConfig u1_config;
    struct UartConfig u2_config;
    struct UartConfig u4_485_config;
    struct UartConfig u5_config;
    struct TcpConfig dvl_json_tcp_config;
    struct PidConfig pid_config[16];
    struct PidFeedforwardPositionParam pid2[16];
    struct WaterDepthSensorConfig water_depth_sensor_config;
    int push_invert[8];
    int push2_invert[8];
    uint32_t verification_value;
};

struct Temporary {
    int tint[3];
    float tfloat[3];
    uint16_t log_level[16];
};

const struct Config* get_config();
struct Config* get_config_future();
struct Temporary* get_temp();

void SetConfigToDefault(struct Config* config_to_set);
int SetConfigToDefult();
int SaveConfigFuture();

void config_set_todefult_func();

#endif  // __CONFIG_H_
