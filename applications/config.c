#include "config.h"
#include <string.h>
#include <stdlib.h>
#include "finsh.h"
#include <ulog.h>
#include "hardware/eeprom.h"
#include "hardware/gyro_config.h"
#include "movelib/motion_algorithm.h"
#include "thirdparty/crc-lib-c/crcLib.h"

#define PID_NUM (15)

static void pid_reset(uint8_t pid_command_id);
int config_store_crc_check_state = 0;
struct Config config;
struct Config config_future;
struct Temporary temp;
static void* cur_ptr;

inline const struct Config* get_config(){
    return &config;
}

inline struct Config* get_config_future(){
    return &config_future;
}

inline struct Temporary* get_temp(){
    return &temp;
}

// {"配置命令1","配置命令2", "配置命令3",变量地址, 变量类型}
#define CMD_NUM_WITH_TWO_CONFIG (0 \
								+ 3/*net_config*/\
								+ 2/*robolink_id_config*/\
								+ 3/*robolink_udp_config*/\
								+ 4/*uart_baundrate*/\
								+ 4/*uart_function*/\
								+ 2/*dvl_json_tcp_config*/\
								+ 8/*push_invert*/\
                                + 8/*push2_invert*/\
)
struct StrAdd cmd_with_two_config[]={
    {"net_config","ip_addr","", &(config_future.net_config.ip_addr), config_str},	// 0
    {"net_config","gw_addr","",&(config_future.net_config.gw_addr), config_str}, 	// 1
    {"net_config","msk_addr","",&(config_future.net_config.msk_addr),config_str},	// 2

    {"robolink_id_config","local_id","",&(config_future.robolink_id_config.local_id), config_uint8_t}, 	// 3
    {"robolink_id_config","remote_id","",&(config_future.robolink_id_config.remote_id),config_uint8_t}, // 4

    {"robolink_udp_config","local_port",     "",&(config_future.robolink_udp_config.local_port), config_uint16_t}, 	// 5
    {"robolink_udp_config","remote_ip_addr", "",&(config_future.robolink_udp_config.remote_ip_addr), config_str}, 	// 6
    {"robolink_udp_config","remote_port",    "",&(config_future.robolink_udp_config.remote_port),config_uint16_t}, 	// 7

    {"u1_config","baundrate", "",&(config_future.u1_config.baundrate), config_uint32_t},			// 8
    {"u2_config","baundrate", "",&(config_future.u2_config.baundrate), config_uint32_t},			// 9
    {"u4_485_config","baundrate", "",&(config_future.u4_485_config.baundrate), config_uint32_t}, 	// 10
    {"u5_config","baundrate", "",&(config_future.u5_config.baundrate), config_uint32_t}, 			// 11

    {"u1_config","function", "",&(config_future.u1_config.function), config_uint16_t}, 			// 12
    {"u2_config","function", "",&(config_future.u2_config.function), config_uint16_t}, 			// 13
    {"u4_485_config","function", "",&(config_future.u4_485_config.function), config_uint16_t}, 	// 14
    {"u5_config","function", "",&(config_future.u5_config.function), config_uint16_t}, 			// 15

    {"dvl_json_tcp_config","ip_addr", "",&(config_future.dvl_json_tcp_config.ip_addr), config_str}, // 16
    {"dvl_json_tcp_config","port", "",&(config_future.dvl_json_tcp_config.port), config_uint16_t}, 	// 17

	{"push_invert","0", "",&(config_future.push_invert[0]), config_int}, // 18
	{"push_invert","1", "",&(config_future.push_invert[1]), config_int}, // 19
	{"push_invert","2", "",&(config_future.push_invert[2]), config_int}, // 20
	{"push_invert","3", "",&(config_future.push_invert[3]), config_int}, // 21
	{"push_invert","4", "",&(config_future.push_invert[4]), config_int}, // 22
	{"push_invert","5", "",&(config_future.push_invert[5]), config_int}, // 23
	{"push_invert","6", "",&(config_future.push_invert[6]), config_int}, // 24
	{"push_invert","7", "",&(config_future.push_invert[7]), config_int}, // 25

    {"push2_invert","0", "",&(config_future.push2_invert[0]), config_int}, // 26
    {"push2_invert","1", "",&(config_future.push2_invert[1]), config_int}, // 27
    {"push2_invert","2", "",&(config_future.push2_invert[2]), config_int}, // 28
    {"push2_invert","3", "",&(config_future.push2_invert[3]), config_int}, // 29
    {"push2_invert","4", "",&(config_future.push2_invert[4]), config_int}, // 30
    {"push2_invert","5", "",&(config_future.push2_invert[5]), config_int}, // 31
    {"push2_invert","6", "",&(config_future.push2_invert[6]), config_int}, // 32
    {"push2_invert","7", "",&(config_future.push2_invert[7]), config_int}, // 33
};

#define CMD_NUM_WITH_THREE_CONFIG (0 \
								+ 0/*xxxxxxxxxx*/\
)
struct StrAdd cmd_with_three_config[]={
    {"NoCmd","NoCmd", "",0, config_uint32_t}, // 1
};

#define CMD_NUM_WITH_PID1_CONFIG 5
struct StrAdd cmd_with_pid1_config[]={
    {"pid1","p", "",0, config_uint32_t}, 	// 0
    {"pid1","i", "",0, config_uint32_t}, 	// 1
    {"pid1","d", "",0, config_uint32_t}, 	// 2
	{"pid1","fdlimit", "",0,config_float},	// 3
	{"pid1","fdout", "",0,config_float}, 	// 4
};

#define CMD_NUM_WITH_PID2_CONFIG (0 \
								+ 5/*FeedforwardControlRun*/\
								+ 8/*PidPositionParam*/\
)
struct StrAdd cmd_with_pid2_config[]={
	{"pid2","feed", "type",0, config_uint16_t}, 			// 0
	{"pid2","feed", "add_value",0, config_float}, 			// 1
	{"pid2","feed", "start_add_input",0, config_float}, 	// 2
	{"pid2","feed", "k",0,config_float}, 					// 3
	{"pid2","feed", "max_out",0,config_float}, 				// 4

	{"pid2","ppp", "is_integral_spare",0,config_uint16_t}, 	// 5
	{"pid2","ppp", "kp",0, config_float}, 					// 6
	{"pid2","ppp", "ki",0, config_float}, 					// 7
	{"pid2","ppp", "kd",0, config_float}, 					// 8
	{"pid2","ppp", "begin_integral",0,config_float}, 		// 9
	{"pid2","ppp", "stop_grow_integral",0, config_float}, 	// 10
	{"pid2","ppp", "max_err_integral",0, config_float}, 	// 11
	{"pid2","ppp", "max_out",0,config_float}, 				// 12
};


#define TEMP_NUM (0 \
					+ 6/*test_val*/\
					+ 8/*log_level*/\
)
struct StrAdd cmd_with_temp_config[]={
	{"tint","0", "",&(temp.tint[0]), config_int}, 		// 0
	{"tint","1", "",&(temp.tint[1]), config_int}, 		// 1
	{"tint","2", "",&(temp.tint[2]), config_int},		// 2
	{"tfloat","0", "",&(temp.tfloat[0]), config_float}, // 3
	{"tfloat","1", "",&(temp.tfloat[1]), config_float}, // 4
	{"tfloat","2", "",&(temp.tfloat[2]), config_float}, // 5

	{"log_level","0", "",&(temp.log_level[0]), config_uint16_t}, // 6
	{"log_level","1", "",&(temp.log_level[1]), config_uint16_t}, // 7
	{"log_level","2", "",&(temp.log_level[2]), config_uint16_t}, // 8
	{"log_level","3", "",&(temp.log_level[3]), config_uint16_t}, // 9
	{"log_level","4", "",&(temp.log_level[4]), config_uint16_t}, // 10
	{"log_level","5", "",&(temp.log_level[5]), config_uint16_t}, // 11
	{"log_level","6", "",&(temp.log_level[6]), config_uint16_t}, // 12
	{"log_level","7", "",&(temp.log_level[7]), config_uint16_t}, // 13
};

// 默认参数
void SetConfigToDefault(struct Config* config_to_set)
{
    memset(config_to_set, 0, sizeof(struct Config));

    // 本机网络设置
    strcpy(config_to_set->net_config.ip_addr, "172.22.22.10");
    strcpy(config_to_set->net_config.gw_addr, "172.22.0.1");
    strcpy(config_to_set->net_config.msk_addr, "255.255.0.0");

    // robolink id设置
    config_to_set->robolink_id_config.local_id = 0x02;
    config_to_set->robolink_id_config.remote_id = 0x01;

    // Robolink UDP 网络参数设置
    config_to_set->robolink_udp_config.local_port = 6666;
    strcpy(config_to_set->robolink_udp_config.remote_ip_addr, "172.22.22.123");
    config_to_set->robolink_udp_config.remote_port = 6666;

	strcpy(config_to_set->robot_msg_tcp_svr_config.ip_addr, "172.22.22.10");
	config_to_set->robot_msg_tcp_svr_config.port= 5678;

    // 串口 参数设置
    config_to_set->u1_config.baundrate = 115200;
    config_to_set->u2_config.baundrate = 115200;
    config_to_set->u4_485_config.baundrate = 9600;
    config_to_set->u5_config.baundrate = 115200;

    config_to_set->u1_config.function = 0;
    config_to_set->u2_config.function = 0;
    config_to_set->u4_485_config.function = 0;
    config_to_set->u5_config.function = 0;

//    pid_reset(1);
//    pid_reset(2);

    config_to_set->water_depth_sensor_config.air_press = 0;
    config_to_set->water_depth_sensor_config.air_press_custom_switch = 0;

    // DVL
    strcpy(config_to_set->dvl_json_tcp_config.ip_addr, "192.168.194.95");
    config_to_set->dvl_json_tcp_config.port= 16171;

    for(int i_ = 0; i_ < 8; i_ ++){
    	config_to_set->push_invert[i_] = 1;
        config_to_set->push2_invert[i_] = 1;
    }
}

int GetConfigFirst()
{
    int err = 0;
    memset(&config, 0, sizeof(config));
    err = EepromRead_NoRtos(0, &config, sizeof(config));
    config_store_crc_check_state = (config.verification_value == crc32((uint8_t*)&config,sizeof(config) - sizeof(config.verification_value)));
    if(err != 0 || config_store_crc_check_state != 1) {
    	SetConfigToDefault(&config);  // 若是无法读取到配置信息，则将配置设置为默认值
    }
    rt_memcpy(&config_future, &config, sizeof(struct Config));
    return err;
}

int SetConfigToDefult()
{
    memset(&config_future, 0, sizeof(config_future));
    SetConfigToDefault(&config_future);
    return SaveConfigFuture();
}

int SaveConfigFuture()
{
	config_future.verification_value = crc32((uint8_t*)&config_future, sizeof(config_future) - sizeof(config_future.verification_value));
	if(0 == EepromWrite(0, &config_future, sizeof(config_future))){
		rt_kprintf("succeeded.\r\n");
		return 0;
	}else{
		rt_kprintf("SAVE ERROR.\r\n");
		return -1;
	}

}

void ConfigPrint(struct Config* config_to_print)
{
    rt_kprintf("net_config ip_addr: %s\r\n",config_to_print->net_config.ip_addr);
    rt_kprintf("net_config gw_addr: %s\r\n",config_to_print->net_config.gw_addr);
    rt_kprintf("net_config msk_addr: %s\r\n",config_to_print->net_config.msk_addr);

    rt_kprintf("robolink_id_config local_id: %d\r\n",config_to_print->robolink_id_config.local_id);
    rt_kprintf("robolink_id_config remote_id: %d\r\n",config_to_print->robolink_id_config.remote_id);

    rt_kprintf("robolink_udp_config local_port: %d\r\n",config_to_print->robolink_udp_config.local_port);
    rt_kprintf("robolink_udp_config remote_ip_addr: %s\r\n",config_to_print->robolink_udp_config.remote_ip_addr);
    rt_kprintf("robolink_udp_config remote_port: %d\r\n",config_to_print->robolink_udp_config.remote_port);

    rt_kprintf("u1_config baundrate: %d\r\n",config_to_print->u1_config.baundrate);
    rt_kprintf("u1_config function: %d\r\n",config_to_print->u1_config.function);

    rt_kprintf("u2_config baundrate: %d\r\n",config_to_print->u2_config.baundrate);
    rt_kprintf("u2_config function: %d\r\n",config_to_print->u2_config.function);

    rt_kprintf("u4_485_config baundrate: %d\r\n",config_to_print->u4_485_config.baundrate);
    rt_kprintf("u4_config function: %d\r\n",config_to_print->u4_485_config.function);

    rt_kprintf("u5_config baundrate: %d\r\n",config_to_print->u5_config.baundrate);
    rt_kprintf("u5_config function: %d\r\n",config_to_print->u5_config.function);

    rt_kprintf("dvl_json_tcp_config ip_addr: %s\r\n",config_to_print->dvl_json_tcp_config.ip_addr);
    rt_kprintf("dvl_json_tcp_config port: %d\r\n",config_to_print->dvl_json_tcp_config.port);

    rt_kprintf("push[0]: %d push[1]: %d push[2]: %d push[3]: %d\r\n",config_to_print->push_invert[0],config_to_print->push_invert[1],config_to_print->push_invert[2],config_to_print->push_invert[3]);
    rt_kprintf("push[4]: %d push[5]: %d push[6]: %d push[7]: %d\r\n",config_to_print->push_invert[4],config_to_print->push_invert[5],config_to_print->push_invert[6],config_to_print->push_invert[7]);

    rt_kprintf("push2[0]: %d push2[1]: %d push2[2]: %d push2[3]: %d\r\n",config_to_print->push2_invert[0],config_to_print->push2_invert[1],config_to_print->push2_invert[2],config_to_print->push2_invert[3]);
    rt_kprintf("push2[4]: %d push2[5]: %d push2[6]: %d push2[7]: %d\r\n",config_to_print->push2_invert[4],config_to_print->push2_invert[5],config_to_print->push2_invert[6],config_to_print->push2_invert[7]);
}

void PidConfigPrint(struct Config* config_to_print, uint8_t pid_command_id)
{
	if(pid_command_id == 1){
	    for(int i_ = 0; i_ < (sizeof(config_to_print->pid_config) / sizeof(config_to_print->pid_config[0])); i_++){
			switch(i_){
				case 0: rt_kprintf("roll   ");break;
				case 1: rt_kprintf("pitch  ");break;
				case 2: rt_kprintf("yaw    ");break;
				case 3: rt_kprintf("depth  ");break;
				case 4: rt_kprintf("x      ");break;
				case 5: rt_kprintf("y      ");break;
				case 6: rt_kprintf("line   ");break;
				default:rt_kprintf("none   ");break;
			}
			LOG_RAW("%2d pid1 p: %6.2f i: %6.2f d: %6.2f "
					"fdlimit:%6.2f fdout:%6.2f"
					"\r\n",
					i_,
					config_to_print->pid_config[i_].p,config_to_print->pid_config[i_].i,config_to_print->pid_config[i_].d,
					config_to_print->pid_config[i_].fdlimit,config_to_print->pid_config[i_].fdout
					);
	    }
	}

	if(pid_command_id == 2){
		for(int i_ = 0; i_ < 16; i_++){
			switch(i_){
				case 0: rt_kprintf("roll   ");break;
				case 1: rt_kprintf("pitch  ");break;
				case 2: rt_kprintf("yaw    ");break;
				case 3: rt_kprintf("depth  ");break;
				case 4: rt_kprintf("x      ");break;
				case 5: rt_kprintf("y      ");break;
				case 6: rt_kprintf("line   ");break;
				default:rt_kprintf("none   ");break;
			}
			LOG_RAW("%2d feed type: %2d add_value: %6.2f "
					"start_add_input: %6.2f k: %6.2f max_out: %6.2f "
					"\r\n",
					i_,
					config_to_print->pid2[i_].feedforward_control_param.type,
					config_to_print->pid2[i_].feedforward_control_param.add_value,
					config_to_print->pid2[i_].feedforward_control_param.start_add_input,
					config_to_print->pid2[i_].feedforward_control_param.k,
					config_to_print->pid2[i_].feedforward_control_param.max_out);
		}
		for (int i_ = 0; i_ < 16; i_++) {
			switch(i_){
				case 0: rt_kprintf("roll   ");break;
				case 1: rt_kprintf("pitch  ");break;
				case 2: rt_kprintf("yaw    ");break;
				case 3: rt_kprintf("depth  ");break;
				case 4: rt_kprintf("x      ");break;
				case 5: rt_kprintf("y      ");break;
				case 6: rt_kprintf("line   ");break;
				default:rt_kprintf("none   ");break;
			}
		    LOG_RAW("%2d ppp kp: %6.2f ki:%6.2f kd: %6.2f is_integral_spare:%d begin_int: %6.2f\r\n",
		            i_, config_to_print->pid2[i_].pid_position_param.kp,
		            config_to_print->pid2[i_].pid_position_param.ki,
		            config_to_print->pid2[i_].pid_position_param.kd,
		            config_to_print->pid2[i_].pid_position_param.is_integral_spare,
					config_to_print->pid2[i_].pid_position_param.begin_integral);
		    LOG_RAW("   stop_grow_int: %6.2f max_err_int:  %6.2f  max_out: %6.2f\r\n",
		            config_to_print->pid2[i_].pid_position_param.stop_grow_integral,
		            config_to_print->pid2[i_].pid_position_param.max_err_integral,
		            config_to_print->pid2[i_].pid_position_param.max_out);
		}
	}
}

extern int config_get_first_status;
static void config_get_init_status(int argc, char**argv)
{
    rt_kprintf("config init status: %d\r\n", config_get_first_status);
}

static void config_store_crc_check_state_print()
{
    rt_kprintf("Config store crc check state: %d.\r\n", config_store_crc_check_state);
}

void config_set_todefult()
{
    int err = SetConfigToDefult();
    rt_kprintf("Config set to defult: %d.\r\n", err);
    if(!err) {
        rt_kprintf("Please restart the device for the configuration to take effect.\r\n");
    }
}

static void pid_reset(uint8_t pid_command_id){
	if(pid_command_id == 1){
		for(int _i = 0; _i < 16; _i ++){
			config_future.pid_config[_i].p = 0.0f;
			config_future.pid_config[_i].i = 0.0f;
			config_future.pid_config[_i].d = 0.0f;
		}
	}
	if(pid_command_id == 2){
		for(int _i = 0; _i < 16; _i ++){
			config_future.pid2[_i].feedforward_control_param.type = 0;
			config_future.pid2[_i].feedforward_control_param.add_value = 0;
			config_future.pid2[_i].feedforward_control_param.start_add_input = 0;
			config_future.pid2[_i].feedforward_control_param.k = 0;
			config_future.pid2[_i].feedforward_control_param.max_out = 0;

			config_future.pid2[_i].pid_position_param.kp = 0.0f;
			config_future.pid2[_i].pid_position_param.ki = 0.0f;
			config_future.pid2[_i].pid_position_param.kd = 0.0f;
			config_future.pid2[_i].pid_position_param.is_integral_spare = 0;
			config_future.pid2[_i].pid_position_param.begin_integral = 0;
			config_future.pid2[_i].pid_position_param.stop_grow_integral = 0;
			config_future.pid2[_i].pid_position_param.max_err_integral = 0;
			config_future.pid2[_i].pid_position_param.max_out = 0;
		}
	}
	SaveConfigFuture();
}

static void pid1_reset(){
	pid_reset(1);
}

static void pid2_reset(){
	pid_reset(2);
}


static void ConfigSetWithTwoConfig(int argc, char**argv){
	int i_ = 0;
	for(i_ = 0; i_ < CMD_NUM_WITH_TWO_CONFIG; i_++){
		if( strcmp(cmd_with_two_config[i_].str, argv[1]) == 0 && strcmp(cmd_with_two_config[i_].mem, argv[2]) == 0){
			switch(cmd_with_two_config[i_].ptr_type){
				case 1: strcpy((char*)cmd_with_two_config[i_].member_ptr,argv[3]);
						SaveConfigFuture();
						return ;
				case 2: {uint8_t value_u8 = atoi(argv[3]);
						rt_memcpy(cmd_with_two_config[i_].member_ptr, &value_u8, sizeof(uint8_t));
						SaveConfigFuture();
						return ;}
				case 3: {uint16_t value_u16 = atoi(argv[3]);
						rt_memcpy(cmd_with_two_config[i_].member_ptr, &value_u16, sizeof(uint16_t));
						SaveConfigFuture();
						return ;}
				case 4: {uint32_t value_u32 = atoi(argv[3]);
						rt_memcpy(cmd_with_two_config[i_].member_ptr, &value_u32, sizeof(uint32_t));
						SaveConfigFuture();
						return ;}
				case 5: {float value_float = atof(argv[3]);
						rt_memcpy(cmd_with_two_config[i_].member_ptr, &value_float, sizeof(float));
						SaveConfigFuture();
						return ;}
				case 6: {int value_int = atoi(argv[3]);
						rt_memcpy(cmd_with_two_config[i_].member_ptr, &value_int, sizeof(int));
						SaveConfigFuture();
						return ;}
				default:break;
			}
		}
	}
	if(i_ == CMD_NUM_WITH_TWO_CONFIG){
		rt_kprintf("Command error -- two config.\r\n");
		return ;
	}
}

static void ConfigSetWithThreeConfig(int argc, char**argv){
	int i_ = 0;
	for(i_ = 0; i_ < CMD_NUM_WITH_THREE_CONFIG; i_++){
		if( strcmp(cmd_with_three_config[i_].str, argv[1]) == 0 && strcmp(cmd_with_three_config[i_].mem, argv[2]) == 0 && strcmp(cmd_with_three_config[i_].mem2, argv[3]) == 0){
			switch(cmd_with_three_config[i_].ptr_type){
				case 1: strcpy((char*)cmd_with_two_config[i_].member_ptr,argv[4]);
						SaveConfigFuture();
						return ;
				case 2: {uint8_t value_u8 = atoi(argv[4]);
						rt_memcpy(cmd_with_two_config[i_].member_ptr, &value_u8, sizeof(uint8_t));
						SaveConfigFuture();
						return ;}
				case 3: {uint16_t value_u16 = atoi(argv[4]);
						rt_memcpy(cmd_with_two_config[i_].member_ptr, &value_u16, sizeof(uint16_t));
						SaveConfigFuture();
						return ;}
				case 4: {uint32_t value_u32 = atoi(argv[4]);
						rt_memcpy(cmd_with_two_config[i_].member_ptr, &value_u32, sizeof(uint32_t));
						SaveConfigFuture();
						return ;}
				case 5: {float value_float = atof(argv[4]);
						rt_memcpy(cmd_with_two_config[i_].member_ptr, &value_float, sizeof(float));
						SaveConfigFuture();
						return ;}
				case 6: {int value_int = atoi(argv[4]);
						rt_memcpy(cmd_with_two_config[i_].member_ptr, &value_int, sizeof(int));
						SaveConfigFuture();
						return ;}
				default:break;
			}
		}
	}
	if(i_ == CMD_NUM_WITH_THREE_CONFIG){
		rt_kprintf("Command error -- three config.\r\n");
		return ;
	}
}

static void ConfigSetWithPidConfig(int argc, char**argv){
	int i_;
	if(strcmp(argv[1], "pid1") == 0){
        for(i_ = 0; i_ < CMD_NUM_WITH_PID1_CONFIG; i_++){
            if( strcmp(cmd_with_pid1_config[i_].mem, argv[3]) == 0){
                float value_f = atof(argv[4]);
                int pid_config_num = atoi(argv[2]);
                if(!((pid_config_num >= 0) && (pid_config_num <= PID_NUM))) {
                	rt_kprintf("Command error -- Pid1 array number does not exist.\r\n");
                	return;
                }
                switch(i_){
                    case 0: cmd_with_pid1_config[i_].member_ptr = &(config_future.pid_config[pid_config_num].p);break;
                    case 1: cmd_with_pid1_config[i_].member_ptr = &(config_future.pid_config[pid_config_num].i);break;
                    case 2: cmd_with_pid1_config[i_].member_ptr = &(config_future.pid_config[pid_config_num].d);break;
                    case 3: cmd_with_pid1_config[i_].member_ptr = &(config_future.pid_config[pid_config_num].fdlimit);break;
                    case 4: cmd_with_pid1_config[i_].member_ptr = &(config_future.pid_config[pid_config_num].fdout);break;
                    default: break;
                }
                rt_memcpy(cmd_with_pid1_config[i_].member_ptr, &value_f, sizeof(float));
                LOG_D("%d %f %f",pid_config_num ,value_f ,config_future.pid_config[pid_config_num].p);
                SaveConfigFuture();
                return ;
            }
        }
		if(i_ == CMD_NUM_WITH_PID1_CONFIG){
			rt_kprintf("Command error -- pid1 config.\r\n");
			return ;
		}
	}else if(strcmp(argv[1], "pid2") == 0){
		for(i_ = 0; i_ < CMD_NUM_WITH_PID2_CONFIG; i_++){
			if( strcmp(cmd_with_pid2_config[i_].mem, argv[3]) == 0 && strcmp(cmd_with_pid2_config[i_].mem2, argv[4]) == 0){
				int pid2_config_num = atoi(argv[2]); // 组号
				if(!((pid2_config_num >= 0) && (pid2_config_num <= PID_NUM))) {
					rt_kprintf("Command error -- Pid2 array number does not exist.\r\n");
					return;
				}
				if (i_ == 0){ 		// == strcmp(config_str_add[i_].mem2, "type") == 0
					uint16_t value_u16 = atoi(argv[5]);
					cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].feedforward_control_param.type);
					rt_memcpy(cmd_with_pid2_config[i_].member_ptr, &value_u16, sizeof(uint16_t));
					SaveConfigFuture();
					return ;
				}else if(i_ == 5){ 	// == strcmp(config_str_add[i_].mem2, "is_integral_spare") == 0
					uint16_t value_u16 = atoi(argv[5]);
					cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].pid_position_param.is_integral_spare);
					rt_memcpy(cmd_with_pid2_config[i_].member_ptr, &value_u16, sizeof(uint16_t));
					SaveConfigFuture();
					return ;
				}else{
					float value_f = atof(argv[5]);
					switch(i_){
//		                case 0: cmd_with_pid_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].feedforward_control_param.type);break;
						case 1: cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].feedforward_control_param.add_value);break;
						case 2: cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].feedforward_control_param.start_add_input);break;
						case 3: cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].feedforward_control_param.k);break;
						case 4: cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].feedforward_control_param.max_out);break;
//		                case 5: cmd_with_pid_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].pid_position_param.is_integral_spare);break;
						case 6: cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].pid_position_param.kp);break;
						case 7: cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].pid_position_param.ki);break;
						case 8: cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].pid_position_param.kd);break;
						case 9: cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].pid_position_param.begin_integral);break;
						case 10 : cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].pid_position_param.stop_grow_integral);break;
						case 11 : cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].pid_position_param.max_err_integral);break;
						case 12 : cmd_with_pid2_config[i_].member_ptr = &(config_future.pid2[pid2_config_num].pid_position_param.max_out);break;
						default: break;
					}
					cur_ptr = cmd_with_pid2_config[i_].member_ptr;
					rt_memcpy(cmd_with_pid2_config[i_].member_ptr, &value_f, sizeof(float));
					SaveConfigFuture();
					return ;
				}
			}
		}
		if(i_ == CMD_NUM_WITH_PID2_CONFIG){
			rt_kprintf("Command error -- pid2 config.\r\n");
			return ;
		}
	}
}

static void config_set(int argc, char**argv)
{
    if(strcmp(argv[1], "pid2") == 0 || strcmp(argv[1], "pid1") == 0){
    	 ConfigSetWithPidConfig(argc, argv);
    }else if(argc == 4){
    	ConfigSetWithTwoConfig(argc, argv);
    }else if(argc == 5){
    	ConfigSetWithThreeConfig(argc, argv);
    }else{
        rt_kprintf("Command error -- config num error\r\n");
        return;
    }
}

static void config_list()
{
    rt_kprintf("--------------------\r\n");
    rt_kprintf("Now config: \r\n");
    ConfigPrint(&config);
    rt_kprintf("--------------------\r\n");
    rt_kprintf("--------------------\r\n");
    rt_kprintf("Future config: \r\n");
    ConfigPrint(&config_future);
    rt_kprintf("--------------------\r\n");
}

static void pid1_list()
{
    rt_kprintf("--------------------\r\n");
    rt_kprintf("pid config: \r\n");
    PidConfigPrint(&config_future,1);
    rt_kprintf("--------------------\r\n");
}
static void pid2_list()
{
    rt_kprintf("--------------------\r\n");
    rt_kprintf("pid config: \r\n");
    PidConfigPrint(&config_future,2);
    rt_kprintf("--------------------\r\n");
}

// 临时pid用
static void edit(int argc, char**argv){
	float value_f = atof(argv[1]);
	if(cur_ptr != 0){
		rt_memcpy(cur_ptr, &value_f, sizeof(float));
	}
}

static void temp_set(int argc, char**argv)
{
    if(argc == 4){
        int i_;
        // pid数组前的匹配遍历
        for(i_ = 0; i_ < TEMP_NUM; i_++){
            if( strcmp(cmd_with_temp_config[i_].str, argv[1]) == 0 && strcmp(cmd_with_temp_config[i_].mem, argv[2]) == 0){
                switch(cmd_with_temp_config[i_].ptr_type){
                    case 1: strcpy((char*)cmd_with_temp_config[i_].member_ptr,argv[3]);
                    		goto temp_return;
                    case 2: {uint8_t value_u8 = atoi(argv[3]);
                            rt_memcpy(cmd_with_temp_config[i_].member_ptr, &value_u8, sizeof(uint8_t));
                            goto temp_return;}
                    case 3: {uint16_t value_u16 = atoi(argv[3]);
                            rt_memcpy(cmd_with_temp_config[i_].member_ptr, &value_u16, sizeof(uint16_t));
                            goto temp_return;}
                    case 4: {uint32_t value_u32 = atoi(argv[3]);
                            rt_memcpy(cmd_with_temp_config[i_].member_ptr, &value_u32, sizeof(uint32_t));
                            goto temp_return;}
                    case 5: {float value_f = atof(argv[3]);
                            rt_memcpy(cmd_with_temp_config[i_].member_ptr, &value_f, sizeof(float));
                            goto temp_return;}
                    case 6: {int value_i = atoi(argv[3]);
                            rt_memcpy(cmd_with_temp_config[i_].member_ptr, &value_i, sizeof(int));
                            goto temp_return;}
                    default:break;
                }
            }
        }
        temp_return:
        if(i_ == TEMP_NUM) {
            rt_kprintf("TEMP ERROR 1. parameter argc 4 match fail \r\n");
            return;
        }else{
        	rt_kprintf("success \r\n");
        }
    }
}

static void temp_list()
{
    rt_kprintf("--------------------\r\n");
    rt_kprintf("temp config: \r\n");
	rt_kprintf("tint[0]: %d tint[1]: %d tint[2]: %d\r\n",temp.tint[0],temp.tint[1],temp.tint[2]);
	LOG_RAW("tfloat[0]: %5.2f tfloat[1]: %5.2f tfloat[2]: %5.2f\r\n",temp.tfloat[0],temp.tfloat[1],temp.tfloat[2]);

	rt_kprintf("log_level 0 roll: 	%d\r\n",temp.log_level[0]);
	rt_kprintf("log_level 1 pitch:	%d\r\n",temp.log_level[1]);
	rt_kprintf("log_level 2 yaw: 	%d\r\n",temp.log_level[2]);
	rt_kprintf("log_level 3 depth:	%d\r\n",temp.log_level[3]);
	rt_kprintf("log_level 4 x: 		%d\r\n",temp.log_level[4]);
	rt_kprintf("log_level 5 y: 		%d\r\n",temp.log_level[5]);
	rt_kprintf("log_level 6 z: 		%d\r\n",temp.log_level[6]);
	rt_kprintf("log_level 7 line: 	%d\r\n",temp.log_level[7]);
    rt_kprintf("--------------------\r\n");
}

static void temp_reset()
{
	memset(&temp.tint, 0, sizeof(temp.tint));
	memset(&temp.tfloat, 0, sizeof(temp.tfloat));
	memset(&temp.log_level, 0, sizeof(temp.log_level));
	rt_kprintf("temp reset success\r\n");
}

static void lgo()
{
	memset(&temp.log_level, 0, sizeof(temp.log_level));
}

void gyro_config_set(int argc, char**argv){
	int gyro_value = atoi(argv[2]);
	int gyro_set_return = 0;

	if(strcmp("axis", argv[1]) == 0){
		rt_kprintf("Please wait four seconds.\r\n");
		gyro_set_return = gyroAxis(gyro_value);
	}else if(strcmp("orient", argv[1]) == 0){
		rt_kprintf("Please wait four seconds.\r\n");
		gyro_set_return = gyroOrient(gyro_value);
	}else if(strcmp("Recalibrate", argv[1]) == 0 || strcmp("rl", argv[1]) == 0){
		rt_kprintf("Please wait eight seconds.\r\n");
		gyroResetCalibration();
	}else{
		rt_kprintf("gyro set parameter ERROR 1.\r\n");
	}
	if(gyro_set_return == -1){
		rt_kprintf("gyro set parameter ERROR 2.\r\n");
	}
}

void gyro_config_list(int argc, char**argv){
	LOG_RAW("gyro_axis: 	%d\r\n",gyroGet(AXIS));
	LOG_RAW("gyro_orient: 	%d\r\n",gyroGet(ORI));
}

void test(){

}


MSH_CMD_EXPORT(config_get_init_status, get config init status);
MSH_CMD_EXPORT(config_store_crc_check_state_print, get cfg store crc check state);
MSH_CMD_EXPORT(config_set_todefult, config set to defult);
MSH_CMD_EXPORT(config_set, config set);
MSH_CMD_EXPORT(config_list, list all config);
MSH_CMD_EXPORT(temp_set, temp set);
MSH_CMD_EXPORT(temp_list, temp list);
MSH_CMD_EXPORT(temp_reset, temp reset);
MSH_CMD_EXPORT(lgo, log off);
MSH_CMD_EXPORT(pid1_list, list all pid1_config);
MSH_CMD_EXPORT(pid2_list, list all pid2_config);
MSH_CMD_EXPORT(pid1_reset, pid1 reset);
MSH_CMD_EXPORT(pid2_reset, pid2 reset);
MSH_CMD_EXPORT(edit, pid2 edit again);
MSH_CMD_EXPORT(gyro_config_set, gyro config set);
MSH_CMD_EXPORT(gyro_config_list, gyro config get);
MSH_CMD_EXPORT(test, test);

