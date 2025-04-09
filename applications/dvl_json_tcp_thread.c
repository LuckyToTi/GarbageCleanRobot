#include "dvl_json_tcp_thread.h"
#include <stm32f4xx_hal.h>
#include <sys/socket.h>
#include "cJSON.h"
#include "robolink_udp_task.h"
#include "config.h"
#include "xsens_ahrs_thread.h"
#include "userlib/math2.h"
#include "movelib/motion_algorithm.h"
#include "userlib/module_offline_detection.h"

#define LOG_TAG     "dvl"
#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

//Yaw(NED) = -Yaw（ENU）+ 90
#define _yaw_z_value_180 (get_mt_data_2()->euler_angle.yaw_z)
#define _yaw_z_value_360 (GetAngle360FromAngle180(_yaw_z_value_180))
#define absolute_yaw_z_value (GetAngle360FromAngle180(_yaw_z_value_180))

#define _azimuth_angle (GetAngle360FromAngle180(-_yaw_z_value_360 + 90.0))  // NED坐标系下

float global_reset_z_value = 0;
inline float get_absolute_yaw_z_value()
{
    return absolute_yaw_z_value;
}

inline float get_global_reset_z_value()
{
    return global_reset_z_value;
}

static int dvl_json_tcp_socket_fd_ = -1;
static const int dvl_json_tcp_max_rx_size = 2048;
static inline int ParseDvlJsonData(char* str_data, int len);
static rt_mutex_t dvl_reset_mutex_;
static int dvl_reset_status = 0;
static rt_sem_t dvl_reset_sem_ = RT_NULL;
static float dvl_reset_azimuth_angle;  // NED(0-360)

static int is_first_calc = 1;
float global_dvl_x = 0;
float global_dvl_y = 0;
float global_dvl_yaw = 0;
void OnDvlReset(int status);
void OnDvlDataReady(const struct DvlPositionLocal* dvl_pos_data);


// ENU坐标系，以reset时的角度为0°
float get_dvl_yaw() {
#if 0
    return 360.0 - dvl_position_local_data.yaw;
#else
    return GetAngle360FromAngle180(dvl_reset_azimuth_angle - _azimuth_angle);
#endif
}

struct DvlPositionLocal dvl_position_local_data = {};
struct DvlPositionLocal dvl_position_local_data_fixed = {};
struct DvlPositionLocal dvl_position_local_data_send = {};
inline struct DvlPositionLocal* get_dvl_position_local_data()
{
    return &dvl_position_local_data;
}
inline struct DvlPositionLocal* get_dvl_position_local_data_fixed()
{
    return &dvl_position_local_data_fixed;
}


// dvl_json_tcp线程控制块
rt_thread_t dvl_json_tcp_tid = RT_NULL;
inline rt_thread_t* get_dvl_json_tcp_tid()
{
    return &dvl_json_tcp_tid;
}

void DvlJsonTcpThreadEntry(void* parameter)
{
    dvl_reset_mutex_ = rt_mutex_create("dvlRmtx", RT_IPC_FLAG_PRIO);
    dvl_reset_sem_ = rt_sem_create("dvlRsem", 0, RT_IPC_FLAG_PRIO);
    rt_thread_mdelay(250);
    char* rx_buffer = rt_malloc(sizeof(char)*dvl_json_tcp_max_rx_size);
    struct sockaddr_in serverAddress;
    int connect_ret = -1;
    dvl_reset_azimuth_angle = _azimuth_angle;
    struct MonitoredModule* DVL_m = RegisterNewMonitoredModule("DVL", 0, 1000, 3, NULL);

SOCKET:
    rt_thread_mdelay(1000);
    if(dvl_json_tcp_socket_fd_ != -1) {
        closesocket(dvl_json_tcp_socket_fd_);
        dvl_json_tcp_socket_fd_ = -1;
    }
    dvl_json_tcp_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if(dvl_json_tcp_socket_fd_ == -1){
        print_rl_udp(3, "[ERROR] Fail %d to socket %d.", errno, dvl_json_tcp_socket_fd_);
        LOG_D("[ERROR] Fail to socket %d.", dvl_json_tcp_socket_fd_);
        goto SOCKET;
    }

    // 设置服务器地址和端口号
    connect_ret = -1;
    serverAddress.sin_family = AF_INET;
    rt_memset(&(serverAddress.sin_zero), 0, sizeof(serverAddress.sin_zero));
//    serverAddress.sin_addr.s_addr = inet_addr("192.168.194.95");
//    serverAddress.sin_port = htons(16171);
    serverAddress.sin_addr.s_addr = inet_addr(get_config()->dvl_json_tcp_config.ip_addr);
    serverAddress.sin_port = htons(get_config()->dvl_json_tcp_config.port);
    connect_ret = connect(dvl_json_tcp_socket_fd_, (struct sockaddr *)&serverAddress, sizeof(struct sockaddr_in));
    if(connect_ret == -1) {
        print_rl_udp(3, "Connect socket %d fail %d ! return:%d", dvl_json_tcp_socket_fd_, errno, connect_ret);
        LOG_D("Connect socket<%d> fail! return:%d", dvl_json_tcp_socket_fd_, connect_ret);
        goto SOCKET;
    }

    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(dvl_json_tcp_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    for( ; ; ) {
        rt_memset(rx_buffer, '\0', dvl_json_tcp_max_rx_size);
        int bytes_received = recv(dvl_json_tcp_socket_fd_, rx_buffer, dvl_json_tcp_max_rx_size, 0);
        LOG_D("rx len %d", bytes_received);

        if (bytes_received <= 0) {
            print_rl_udp(3, "receive error %d at the socket <%d>.", errno, dvl_json_tcp_socket_fd_);
            LOG_D("receive error at the socket <%d>.", dvl_json_tcp_socket_fd_);
            goto SOCKET;
        }

        int result = ParseDvlJsonData(rx_buffer, bytes_received);///xz
        if(result == 0) MonitoredModuleReload(DVL_m);
    }
    if(dvl_json_tcp_socket_fd_ != -1) {
        closesocket(dvl_json_tcp_socket_fd_);
        dvl_json_tcp_socket_fd_ = -1;
    }
    rt_free(rx_buffer);
    UnregisterMonitoredModuleByPtr(DVL_m);
}

static int ret_success = 0;
static char* ret_error_message = NULL;
static void DvlResetDeadReckoningThreadEntry(void* parameter);

int StartDvlReset() {
    rt_err_t err_ret = rt_mutex_take(dvl_reset_mutex_, RT_WAITING_NO);
    if(RT_EOK == err_ret){
        if(!dvl_reset_status){
            rt_thread_t tid = RT_NULL;
            rt_err_t start_ret = -1;
            tid = rt_thread_create("DVLrstT", DvlResetDeadReckoningThreadEntry, (void*)0, 1536 , 21, 10);
            if (tid != RT_NULL) start_ret = rt_thread_startup(tid);
            if (RT_EOK == start_ret) {
                dvl_reset_status = 1;
//                OnDvlReset(is_first_calc);
                print_rl_udp(2, "DVL try to reset.");
            }
        }
        rt_mutex_release(dvl_reset_mutex_);
    } else {
        return -1;
    }
    return 0;
}

static float x_last = 0, y_last = 0;
static float x_count = 0, y_count = 0;
static void DvlResetDeadReckoningThreadEntry(void* parameter)
{
    if(dvl_json_tcp_socket_fd_ < 0) {
        LOG_D("DVL socket error.");
        print_rl_udp(3, "DVL socket error.");
        return;
    }

    // 生成json
//    cJSON* cjson = cJSON_CreateObject();  // 创建一个JSON数据对象(链表头结点)
//    cJSON_AddStringToObject(cjson, "command", "reset_dead_reckoning");  // 添加一条字符串类型的JSON数据(添加一个链表节点)
//    char* str = cJSON_Print(cjson);
//    cJSON_Delete(cjson);
    static const char str[] = "{\"command\":\"reset_dead_reckoning\"}";

    // 发送指令
    // int bytes_send = send(dvl_json_tcp_socket_fd_, str, rt_strlen(str), 0);
    int bytes_send = sal_sendto(dvl_json_tcp_socket_fd_, str, rt_strlen(str), 0, NULL, 0);
//    rt_free(str);

    // 等待回应
    if(bytes_send > 0) {
        if(rt_sem_take(dvl_reset_sem_, 1000) != RT_EOK) {
            print_rl_udp(3, "DVL no back.");
        } else {
            if(cJSON_True == ret_success) {
                OnDvlReset(1);
                print_rl_udp(2, "DVL already Reset.");
            } else {
                print_rl_udp(3, "DVL NOT Reset, ERROR message: %s.", ret_error_message);
            }
        }
    } else {
        print_rl_udp(3, "DVL socket send error.");
    }

    // 清除接收到的数据
    ret_success = 0;
    if(ret_error_message != NULL) {
        rt_free(ret_error_message);
        ret_error_message = NULL;
    }

    // 解锁
    if(RT_EOK == rt_mutex_take(dvl_reset_mutex_, RT_WAITING_FOREVER)){
        dvl_reset_status = 0;
        rt_mutex_release(dvl_reset_mutex_);
    }

}

static inline int ParseDvlJsonData(char* str_data, int len) {
    cJSON* cjson = cJSON_Parse(str_data);  //将JSON字符串转换成JSON结构体
    if(cjson == NULL)                       //判断转换是否成功
    {
        LOG_D("cjson error...\r\n");
        return -1;
    }

    char* type = cJSON_GetObjectItem(cjson,"type")->valuestring;
    if(0 == rt_strcmp(type, "position_local")) {
        dvl_position_local_data.ts = cJSON_GetObjectItem(cjson,"ts")->valuedouble;
        dvl_position_local_data.x = cJSON_GetObjectItem(cjson,"x")->valuedouble;
        dvl_position_local_data.y = cJSON_GetObjectItem(cjson,"y")->valuedouble;
        dvl_position_local_data.z = cJSON_GetObjectItem(cjson,"z")->valuedouble;
        dvl_position_local_data.pos_std = cJSON_GetObjectItem(cjson,"std")->valuedouble;
        dvl_position_local_data.roll = cJSON_GetObjectItem(cjson,"roll")->valuedouble;
        dvl_position_local_data.pitch = cJSON_GetObjectItem(cjson,"pitch")->valuedouble;
        dvl_position_local_data.yaw = cJSON_GetObjectItem(cjson,"yaw")->valuedouble;
        dvl_position_local_data.status = cJSON_GetObjectItem(cjson,"status")->valueint;
        // LOG_I("%f %f", dvl_position_local_data.x, dvl_position_local_data.yaw);
        // LedRgbWrite('r', !LedRgbRead('r'));

        uint8_t euler_angles_raw[12] = {};
        rt_memcpy(euler_angles_raw + 0, &dvl_position_local_data.roll, sizeof(float));
        rt_memcpy(euler_angles_raw + 4, &dvl_position_local_data.pitch, sizeof(float));
        rt_memcpy(euler_angles_raw + 8, &dvl_position_local_data.yaw, sizeof(float));
        RobolinkUdpCreateInitSendNewMsgToMb((uint8_t *) euler_angles_raw, 12, 0x04, 0x02, 0x04);

        float azimuth_angle = _azimuth_angle;
        static const int dvl_position_local_len = 41;
        uint8_t* dvl_position_local = rt_malloc(dvl_position_local_len);
        rt_memcpy(dvl_position_local + 0, &dvl_position_local_data.ts, sizeof(float));
        rt_memcpy(dvl_position_local + 4, &dvl_position_local_data.x, sizeof(float));
        rt_memcpy(dvl_position_local + 8, &dvl_position_local_data.y, sizeof(float));
        rt_memcpy(dvl_position_local + 12, &dvl_position_local_data.z, sizeof(float));
        rt_memcpy(dvl_position_local + 16, &dvl_position_local_data.pos_std, sizeof(float));
        rt_memcpy(dvl_position_local + 20, &dvl_position_local_data.roll, sizeof(float));
        rt_memcpy(dvl_position_local + 24, &dvl_position_local_data.pitch, sizeof(float));
        rt_memcpy(dvl_position_local + 28, &dvl_position_local_data.yaw, sizeof(float));
        dvl_position_local[32] = *((uint8_t*)&dvl_position_local_data.status);
        rt_memcpy(dvl_position_local + 33, &dvl_reset_azimuth_angle, sizeof(float));
        rt_memcpy(dvl_position_local + 37, &azimuth_angle, sizeof(float));
        OnDvlDataReady(&dvl_position_local_data);
        rt_memcpy(&dvl_position_local_data_send, &dvl_position_local_data_fixed, sizeof(dvl_position_local_data));

        float temp = 0;
        temp = dvl_position_local_data_send.x;
        dvl_position_local_data_send.x = dvl_position_local_data_send.y;
        dvl_position_local_data_send.y = temp;

        RobolinkUdpCreateInitSendNewMsgToMb((uint8_t *) &dvl_position_local_data_send, dvl_position_local_len, get_config()->robolink_id_config.local_id, 0x09, 0x01);
        rt_free(dvl_position_local);
    } else if(0 == rt_strcmp(type, "response")) {
        char* response_to = cJSON_GetObjectItem(cjson,"response_to")->valuestring;
        if(0 == rt_strcmp(response_to, "reset_dead_reckoning")) {
            ret_success = cJSON_GetObjectItem(cjson,"success")->type;
            if(ret_error_message != NULL) {
                rt_free(ret_error_message);
                ret_error_message = NULL;
            }
            if(ret_success == cJSON_False){
                char* error_message = cJSON_GetObjectItem(cjson,"error_message")->valuestring;
                int error_message_len = rt_strlen(error_message);
                ret_error_message = rt_malloc(error_message_len + 1);
                rt_memcpy(ret_error_message, error_message, error_message_len);
                ret_error_message[error_message_len] = '\0';
            }

            if(dvl_reset_status) rt_sem_release(dvl_reset_sem_);
        }
    }
    cJSON_Delete(cjson);//清除结构体
    return 0;
}




// TODO: 以下是通过xsens或其他陀螺仪修正后的DVL计算，和全局的DVL的计算
// 开始计算的条件是第一次点击reset，且imu有效
// 坐标系建立的参考yaw以imu的yaw=0为准（yaw_imu=0为头）

// 点击重设参考，及点击reset，重置动力定位的目标值可能不用处理，
//   x因为动力定位之后将使用全局DVL值来计算，点击reset影响不到全局值

// 重设参考点目前想的是在move_thread中完成，用参考点(x,y,yaw)与全局DVL值计算出 相对参考点的x,y,yaw(相当于在参考点，模拟点击了一次DVLreset后的值)
//   P.S. 如果来不及做，有个方案可以考虑，设置相对参考点时，就进行一次重置操作，然后直接显示使用imu修复之后的x,y,yaw
//   P.S. 如果按照上一行那样来做，就要考虑全局DLV值和使用imu修复之后的DVL值，这两之间怎么转换

#define IMU_STATUS (1u)
#define YAW_IMU (0)
#define YAW_DVL (0)

inline float get_gloal_yaw()
{
    return absolute_yaw_z_value;
}

void OnDvlReset(int status)
{
    if(is_first_calc) {
        x_count = 0;
        y_count = 0;
        dvl_position_local_data_fixed.x = 0;
        dvl_position_local_data_fixed.y = 0;
        dvl_position_local_data.x = 0;
        dvl_position_local_data.y = 0;
        x_last = dvl_position_local_data.x;
        y_last = dvl_position_local_data.y;
        dvl_reset_azimuth_angle = _azimuth_angle;
        global_reset_z_value = absolute_yaw_z_value;
        is_first_calc = 0;
    }
    if(status == 1){
        x_last = 0;
        y_last = 0;
        dvl_reset_azimuth_angle = _azimuth_angle;
        global_reset_z_value = absolute_yaw_z_value;
    }
}


// TODO
void OnDvlDataReady(const struct DvlPositionLocal* dvl_pos_data)
{
    if(is_first_calc) return;

    rt_memcpy(&dvl_position_local_data_fixed, &dvl_position_local_data, sizeof(dvl_position_local_data));
    float x_fixed = 0, y_fixed = 0;
//    float x_fixed_2 = 0, y_fixed_2 = 0;
    CalcHeadMoveMotorsSpeed(GetAngleOver360(GetAngle360FromAngle180(dvl_position_local_data.yaw) + get_dvl_yaw()),// 漂移量修正
                    dvl_position_local_data.x - x_last, dvl_position_local_data.y - y_last,
                    &x_fixed, &y_fixed);

    CalcHeadMoveMotorsSpeed(global_reset_z_value, x_fixed, y_fixed, &x_fixed, &y_fixed); // 后续坐标系修正成第一个坐标系方向

//    print_rl_udp(2, "get_absolute_yaw_z_value:%f y:%f",x_fixed_2,y_fixed_2);
    x_last = dvl_position_local_data.x;
    y_last = dvl_position_local_data.y;
    x_count += x_fixed;
    y_count += y_fixed;
    dvl_position_local_data_fixed.x = x_count;
    dvl_position_local_data_fixed.y = y_count;
}

// 获取第二个坐标在第一个坐标系内的值, 非直角坐标系
//void GetX1YRealCoordinate(float x2y_y, float x2y_x, float x1y_y, float x1y_x, float x1y_x2y_include_angle, float *x2y_to_x1y_y, float *x2y_to_x1y_x) {
//    float x2y_y_rel = x2y_y;
//    float x2y_x_rel = x2y_x;
//    float cos_x1y_x2y_include_angle = cos(x1y_x2y_include_angle);
//    float sin_x1y_x2y_include_angle = sin(x1y_x2y_include_angle);
//    float Cx2_rotated = cos_x1y_x2y_include_angle * x2y_y_rel - sin_x1y_x2y_include_angle * x2y_x_rel;
//    float Cy2_rotated = sin_x1y_x2y_include_angle * x2y_y_rel + cos_x1y_x2y_include_angle * x2y_x_rel;
//    *x2y_to_x1y_y = Cx2_rotated + x1y_y;
//    *x2y_to_x1y_x = Cy2_rotated + x1y_x;
//}

// 获取第二个坐标在第一个坐标系内的值, 非直角坐标系
void GetX1YRealCoordinate(float x1y_x, float x1y_y, float x2y_x, float x2y_y, float angle_deg, float *output_x, float *output_y) {
        float angle_rad = angle_deg * (M_PI / 180.0);
        float rotated_x = x2y_x * arm_cos_f32(angle_rad) - x2y_y * arm_sin_f32(angle_rad);
        float rotated_y = x2y_x * arm_sin_f32(angle_rad) + x2y_y * arm_cos_f32(angle_rad);
        *output_x = x1y_x + rotated_x;
        *output_y = x1y_y + rotated_y;
}
