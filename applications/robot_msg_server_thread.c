#include "robot_msg_server_thread.h"
#include <stm32f4xx_hal.h>
#include <sys/socket.h>
#include <string.h>
#include "userlib/robot_msg_json.h"
#include "userlib/module_offline_detection.h"
#include "config.h"

#define LOG_TAG     "r_msg_s_t"
#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

const static char client_tx_thread0_name[] = "rMtxSVR0";
const static char client_tx_thread1_name[] = "rMtxSVR1";
const static char client_rx_thread0_name[] = "rMrxSVR0";
const static char client_rx_thread1_name[] = "rMrxSVR1";
const static char* client_thread_name[4] = {
        client_tx_thread0_name,
        client_tx_thread1_name,
        client_rx_thread0_name,
        client_rx_thread1_name};
#define C_TX_NAME_OFFSET (0U)
#define C_RX_NAME_OFFSET (2U)

static const int recv_max_size = 1500;
struct RobotMsgServerSession {
    rt_int32_t server_fd;
    rt_int16_t port;
    rt_int32_t client_fd[2];
    rt_bool_t stop_client[2];
    int active_client;  // 0 1
    rt_mutex_t client_fd_mutex[2];
};
static struct RobotMsgServerSession* msg_svr = NULL;


void RobotMsgServerThread(void* parameter);
void RobotMsgServerSendThread(void* parameter);
void RobotMsgServerRecvThread(void* parameter);
static void server_close(struct RobotMsgServerSession* s);
static void client_close(struct RobotMsgServerSession* s, int id);
static void ProcessRx(uint8_t *data, rt_size_t length);

// 线程控制块
rt_thread_t robot_msg_server_tid = RT_NULL;
inline rt_thread_t* get_robot_msg_server_tid()
{
    return &robot_msg_server_tid;
}

void RobotMsgServerThreadEntry(void* parameter)
{
    // 初始化msg_svr
    if(msg_svr == NULL)
        msg_svr = rt_malloc(sizeof(struct RobotMsgServerSession));
    msg_svr->server_fd = -1;
    msg_svr->port = get_config()->robot_msg_tcp_svr_config.port;
    msg_svr->client_fd[0] = -1; msg_svr->client_fd[1] = -1;
    msg_svr->stop_client[0] = RT_TRUE; msg_svr->stop_client[1] = RT_TRUE;
    msg_svr->active_client = 0;
    msg_svr->client_fd_mutex[0] = rt_mutex_create("RMS_cm0", RT_IPC_FLAG_PRIO);
    msg_svr->client_fd_mutex[1] = rt_mutex_create("RMS_cm1", RT_IPC_FLAG_PRIO);

    rt_thread_t tid;
    tid = rt_thread_create("rMsvr", RobotMsgServerThread, RT_NULL, 2048, 15, 5);
    if (tid != RT_NULL) {
        rt_thread_startup(tid);
        LOG_D("RobotMsgServer start successfully");
    }
}

void RobotMsgServerThread(void* parameter)
{
    rt_int32_t keepalive = 1;
    struct sockaddr_in addr;
    socklen_t addr_size;

    /* 一个socket在使用前，需要预先创建出来，指定SOCK_STREAM为TCP的socket */
    msg_svr->server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (msg_svr->server_fd == -1) {
        LOG_W("msg server: Create socket failed!");
        return;
    }

    //使能心跳机制，默认没有使能
    if(setsockopt(msg_svr->server_fd, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepalive, sizeof(keepalive)) < 0) {
        LOG_W("msg server:set socket keepalive failed!");
        return;
    }

    /* 初始化服务端地址 */
    addr.sin_family = AF_INET;
    addr.sin_port = htons(msg_svr->port);
    addr.sin_addr.s_addr = INADDR_ANY;
    rt_memset(&(addr.sin_zero), 0, sizeof(addr.sin_zero));
    if (bind(msg_svr->server_fd, (struct sockaddr *) &addr, sizeof(struct sockaddr)) == -1) {
        LOG_W("msg server: Unable to bind\n");
        return;
    }

    /* 在socket上进行监听 */
    if (listen(msg_svr->server_fd, 2) == -1) {
        LOG_W("msg server: listen socket failed\n");
        return;
    }

    while (1)
    {
        /* 接受一个客户端连接socket的请求，这个函数调用是阻塞式的 */
        rt_memset(&addr, 0, sizeof(addr));
        int client_fd = accept(msg_svr->server_fd, (struct sockaddr *)&addr, &addr_size);

        if(client_fd < 0) {
            LOG_W("msg server: accept connection failed!");
            continue;
        }

        /* 接受返回的client_addr指向了客户端的地址信息 */
        LOG_D("msg server: new client(%s:%d) connection.", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));

        // 校验ip
        if(addr.sin_addr.s_addr != inet_addr(get_config()->robolink_udp_config.remote_ip_addr)) {
            static const char c_err_send[] = "{\"type\":\"text\",\"text\":\"client ip check error!\"}";
            sal_sendto(client_fd, c_err_send, rt_strlen(c_err_send), 0, NULL, 0);
            shutdown(client_fd, SHUT_RDWR);
            closesocket(client_fd);
            LOG_D("msg server: client ip check error!");
            continue;
        }

        // 关闭当前活动的连接
        client_close(msg_svr, msg_svr->active_client);

        // 确保有空闲的连接可用
        int free_client_id = (msg_svr->active_client==0 ? 1:0);
        while(msg_svr->stop_client[free_client_id] != RT_TRUE) {
            rt_thread_delay(100);
        }
        while(rt_thread_find((char*)client_thread_name[C_TX_NAME_OFFSET + free_client_id]) != NULL ||
              rt_thread_find((char*)client_thread_name[C_RX_NAME_OFFSET + free_client_id]) != NULL) {
            rt_thread_delay(100);
        }
        msg_svr->active_client = free_client_id;

        // 处理active_client
        int err_ret = rt_mutex_take(msg_svr->client_fd_mutex[msg_svr->active_client], RT_WAITING_FOREVER);
        if(RT_EOK == err_ret){
            msg_svr->client_fd[msg_svr->active_client] = client_fd;
            msg_svr->stop_client[msg_svr->active_client] = RT_FALSE;
            rt_mutex_release(msg_svr->client_fd_mutex[msg_svr->active_client]);
        }

        // 设置TCP发送超时时间
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
//        setsockopt(msg_svr->client_fd[msg_svr->active_client], SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(msg_svr->client_fd[msg_svr->active_client], SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        // 启动对应RX&TX线程
        rt_thread_t ctxrx_tid = NULL;
        ctxrx_tid = rt_thread_create(client_thread_name[C_TX_NAME_OFFSET + msg_svr->active_client],
                                     RobotMsgServerSendThread, (void*)msg_svr->active_client, 1024+512+1024, 14, 5);
        if (ctxrx_tid != RT_NULL) {
            rt_thread_startup(ctxrx_tid);
            LOG_D("RobotMsgServerSendThread: -%s- thread start successfully",
                  client_thread_name[C_TX_NAME_OFFSET + msg_svr->active_client]);

            ctxrx_tid = rt_thread_create(client_thread_name[C_RX_NAME_OFFSET + msg_svr->active_client],
                                         RobotMsgServerRecvThread, (void*)msg_svr->active_client, 1024+512, 14, 5);
            if (ctxrx_tid != RT_NULL) {
                rt_thread_startup(ctxrx_tid);
                LOG_D("RobotMsgServerSendThread: -%s- thread start successfully",
                      client_thread_name[C_RX_NAME_OFFSET + msg_svr->active_client]);
            }else{
                client_close(msg_svr, msg_svr->active_client);
                LOG_D("RobotMsgServerSendThread: -%s- thread start faild",
                      client_thread_name[C_RX_NAME_OFFSET + msg_svr->active_client]);
            }
        }else{
            client_close(msg_svr, msg_svr->active_client);
            LOG_D("RobotMsgServerSendThread: -%s- thread start faild",
                  client_thread_name[C_TX_NAME_OFFSET + msg_svr->active_client]);
        }
    }

    /* 退出服务 */
    client_close(msg_svr, 0);
    client_close(msg_svr, 1);
    server_close(msg_svr);
    return ;
}

void RobotMsgServerRecvThread(void* parameter) {
    int client_id = (int)parameter;
    if(client_id != 0 && client_id != 1) goto C_RECV_END;
    uint8_t* recv_buf = rt_malloc(recv_max_size * sizeof(uint8_t));
    if(recv_buf == NULL) goto C_RECV_END;

    /* 客户端连接的处理 */
    while (msg_svr->stop_client[client_id] == RT_FALSE) {

        /* 从connected socket中接收数据，接收buffer是1024大小，但并不一定能够收到1024大小的数据 */
        int32_t recv_len = recv(msg_svr->client_fd[client_id], recv_buf, recv_max_size, 0);

        if (recv_len > 0) {
            ProcessRx(recv_buf, recv_len);
        } else {
            break;
        }
    }

C_RECV_END:
    client_close(msg_svr, client_id);
    if(recv_buf != NULL) {
        rt_free(recv_buf);
    }

    char t_name[9] = {0};
    rt_memcpy(t_name, rt_thread_self()->name, RT_NAME_MAX);
    LOG_D("RobotMsgServerSendThread: -%s- thread finished.", t_name);
}



void RobotMsgServerSendThread(void* parameter)
{
    int client_id = (int)parameter;
    if(client_id != 0 && client_id != 1) goto C_SEND_END;
    int count = 0;
    typedef char* (*get_send_char_func)(void);
    const static get_send_char_func functions[] =
        {&RobotMsgCreateNewJsonStr,
         &ModuleStateCreateNewJsonStr};
    rt_thread_delay(200);

    while(msg_svr->stop_client[client_id] == RT_FALSE)
    {
        char* j_str = functions[count]();
        (count<(sizeof(functions) / sizeof(get_send_char_func) - 1)) ? (++count) : (count=0);

        if(j_str == NULL) {
            continue;
        }

        // 发送数据到客户端
        int ret = sal_sendto(msg_svr->client_fd[client_id], (const void *)j_str, rt_strlen(j_str), 0, NULL, 0);
        rt_free(j_str);

        if (ret <= 0) {
            if (errno != 0)
                break;  /* 发送失败，关闭这个连接  */
        }

        rt_thread_delay(999);
    }

C_SEND_END:
    client_close(msg_svr, client_id);

    char t_name[9] = {0};
    rt_memcpy(t_name, rt_thread_self()->name, RT_NAME_MAX);
    LOG_D("RobotMsgServerSendThread: -%s- thread finished.", t_name);
}

static void server_close(struct RobotMsgServerSession* s)
{
    /* close connection */
    if(s->server_fd >= 0)
        closesocket(s->server_fd);
    s->server_fd = -1;
}

/* client close */
static void client_close(struct RobotMsgServerSession* s, int id)
{
    int err_ret = rt_mutex_take(msg_svr->client_fd_mutex[id], RT_WAITING_FOREVER);
    if(RT_EOK == err_ret){
        s->stop_client[id] = RT_TRUE;
        if(s->client_fd[id] >= 0) {
            shutdown(s->client_fd[id], SHUT_RDWR);
            closesocket(s->client_fd[id]);
            s->client_fd[id] = -1;
        }
        rt_mutex_release(msg_svr->client_fd_mutex[id]);
    }
}

static void ProcessRx(uint8_t *data, rt_size_t length)
{
    data[length < recv_max_size ? length : recv_max_size-1] = '\0';
    int err = RobotMsgModifyDataByJsonStr((const char *)data);
    char* jstr;
    if(err == 0) {
        jstr = GetRobotMsgModifyDataResponseStr(1, NULL);
    } else {
        jstr = GetRobotMsgModifyDataResponseStr(0, NULL);
    }
    int ret = sal_sendto(msg_svr->client_fd[msg_svr->active_client], (const void *)jstr, rt_strlen(jstr), 0, NULL, 0);
    rt_free(jstr);
    if (!(ret > 0)) if (errno != 0) client_close(msg_svr, msg_svr->active_client);
}
