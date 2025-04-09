#include <gnss_thread.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_usart.h>
#include <rthw.h>
#include <rtthread.h>
#include <string.h>
#include "hardware/usart5.h"
#include "thirdparty/minmea/minmea.h"
#include "robolink_udp_task.h"
#include "config.h"
#include "userlib/module_offline_detection.h"

#define LOG_TAG "gnss"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define SATELLITE_NUM (get_config()->push2_invert[1])

const uint16_t gps_rx_max_len = 1024;
uint8_t get_gps_dev_id();
inline uint8_t get_gps_dev_id() {return 0x04;}
static int NMEA0183GetLocatorData(char* line);

struct DvlPositionLocal_C75 dvl_c75_position_local_data = {};
struct DvlPositionLocal_C75* get_dvl_c75_position_local_data(){
    return &dvl_c75_position_local_data;
}

//wtrtk线程控制块
rt_thread_t gnss_tid = RT_NULL;
inline rt_thread_t* get_gnss_tid()
{
    return &gnss_tid;
}

void GnssThreadEntry(void* parameter)
{
    struct MonitoredModule* gnss_NMEA0183_m = RegisterNewMonitoredModule("gnss_NMEA0183", 0, RT_TICK_PER_SECOND * 1.2, 2, NULL);
    char* gps_rx_data = rt_malloc(gps_rx_max_len);
    rt_thread_mdelay(750);
    for(;;)
    {
        rt_memset(gps_rx_data, 0, gps_rx_max_len);
        int get_len = Uart5HalReceiveWaitDataItStopIdle((uint8_t*)gps_rx_data, gps_rx_max_len);
        LOG_D("length:%d", get_len);
        char* line_ptr;
        line_ptr = strtok(gps_rx_data, "\r\n");
        if ((line_ptr != RT_NULL) && ((line_ptr + rt_strlen(line_ptr)) < (gps_rx_data + get_len)))
        {
           // 分割字符串并保存字段
           while ((line_ptr != RT_NULL) && ((line_ptr + rt_strlen(line_ptr) + 2) < (gps_rx_data + get_len))) // \r\n会占用2字符
           {
               LOG_D("l1:%d %s", get_len, line_ptr);
               RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)line_ptr, rt_strlen(line_ptr), get_config()->robolink_id_config.local_id, get_gps_dev_id(), 0x01);
               NMEA0183GetLocatorData(line_ptr);
               line_ptr = strtok(RT_NULL, "\r\n");
           }
           LOG_D("l2:%d %s", get_len, line_ptr);
           RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)line_ptr, rt_strlen(line_ptr), get_config()->robolink_id_config.local_id, get_gps_dev_id(), 0x01);
           NMEA0183GetLocatorData(line_ptr);
        }
        MonitoredModuleReload(gnss_NMEA0183_m);
        rt_thread_mdelay(1);
    }
    rt_free(gps_rx_data);
    UnregisterMonitoredModuleByPtr(gnss_NMEA0183_m);
    return ;
}


inline static int NMEA0183GetLocatorData(char* line)
{
    switch (minmea_sentence_id(line, false)) {
        case MINMEA_SENTENCE_RMC: {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, line)) {
                LOG_D("data valid?: %d %s", frame.valid, line);
                LOG_D("$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d",
                    frame.latitude.value, frame.latitude.scale,
                    frame.longitude.value, frame.longitude.scale,
                    frame.speed.value, frame.speed.scale);
                LOG_D("$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d",
                    minmea_rescale(&frame.latitude, 1000),
                    minmea_rescale(&frame.longitude, 1000),
                    minmea_rescale(&frame.speed, 1000));
                LOG_D("$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                    minmea_tocoord(&frame.latitude),
                    minmea_tocoord(&frame.longitude),
                    minmea_tofloat(&frame.speed));

                char mini_rmc_data[17];
                float latitude = 0;
				float longitude = 0;
				float course = 0;
				float speed = 0;
                if(frame.valid){
                    mini_rmc_data[0] = 'A';
                    latitude = minmea_tocoord(&frame.latitude);
                    longitude = minmea_tocoord(&frame.longitude);
                    course = minmea_tocoord(&frame.course);
                    speed = minmea_tocoord(&frame.speed);
                } else {
                    mini_rmc_data[0] = 'V';
                }
                rt_memcpy(mini_rmc_data+1, &latitude, sizeof(float));
				rt_memcpy(mini_rmc_data+5, &longitude, sizeof(float));
				rt_memcpy(mini_rmc_data+9, &course, sizeof(float));
				rt_memcpy(mini_rmc_data+13, &speed, sizeof(float));
                RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)mini_rmc_data, sizeof(mini_rmc_data), get_config()->robolink_id_config.local_id, get_gps_dev_id(), 0x02);

                return 0;
            } else {
                LOG_D("$xxRMC sentence is not parsed\n");
                return -1;
            }
        } break;

        case MINMEA_SENTENCE_DVEXT: {
                    struct minmea_sentence_dvext frame;
                    if (minmea_parse_dvext(&frame, line)) {
//                        float test_r_value = minmea_tofloat(&frame.r);
//                        float test_p_value = minmea_tofloat(&frame.p);
//                        float test_h_value = minmea_tofloat(&frame.h);
//                        float test_k_value = minmea_tofloat(&frame.k);
//                        float test_u_value = minmea_tofloat(&frame.u);
//                        float test_t_value = minmea_tofloat(&frame.t);
//                        float test_n_value = minmea_tofloat(&frame.n);
//                        float test_e_value = minmea_tofloat(&frame.e);
//                        float test_lat_value = minmea_tofloat(&frame.lat);
//                        float test_long_e_value = minmea_tofloat(&frame.long_e);
//                        float test_eee_value = minmea_tofloat(&frame.eee);
//                        float test_qw = minmea_tofloat(&frame.qw);
//                        float test_qx = minmea_tofloat(&frame.qx);
//                        float test_qy = minmea_tofloat(&frame.qy);
//                        float test_qz = minmea_tofloat(&frame.qz);
//                        float test_va = minmea_tofloat(&frame.va);
//                        float test_vb = minmea_tofloat(&frame.vb);
//                        float test_vc = minmea_tofloat(&frame.vc);
//                        float test_vd = minmea_tofloat(&frame.vd);
//                        float test_ra = minmea_tofloat(&frame.ra);
//                        float test_rb = minmea_tofloat(&frame.rb);
//                        float test_rc = minmea_tofloat(&frame.rc);
//                        float test_rd = minmea_tofloat(&frame.rd);
//
//                        LOG_D("-----------------------------------------------------");
//                        LOG_D("v:%-1c, g:%-1c, adcd:%-5d", frame.v, frame.g, frame.abcd);
//                        LOG_D("r: %-5.3f, h: %-5.3f, k: %-5.3f, u: %-5.3f",test_r_value, test_p_value, test_h_value, test_k_value, test_u_value);
//                        LOG_D("t: %-5.3f, n: %f, e: %-5.3f, lat: %-5.3f, long_e: %-5.3f, eee: %-5.3f",  test_t_value, test_n_value, test_e_value, test_lat_value, test_long_e_value, test_eee_value);
//                        LOG_D("test_qw: %-5.3f, test_qx: %f, test_qy: %-5.3f, test_qz: %-5.3f",  test_qw, test_qx, test_qy, test_qz);
//                        LOG_D("ga: %4d, gb: %4d, gc: %4d, gd: %4d",  frame.ga, frame.gb, frame.gc, frame.gd);
//                        LOG_D("la: %c, lb: %c, lc: %c, ld: %c",  frame.la, frame.lb, frame.lc, frame.ld);
//                        LOG_D("va: %-5.4f, vb: %-5.4f, vc: %-5.4f, vd: %-5.4f",  test_va, test_vb, test_vc, test_vd);
//                        LOG_D("ra: %-5.4f, rb: %-5.4f, rc: %-5.4f, rd: %-5.4f",  test_ra, test_rb, test_rc, test_rd);
                        return 0;
                    } else {
                        LOG_D("$xxDVE sentence is not parsed\n");
                        return -1;
                    }
                } break;

        case MINMEA_SENTENCE_DVPDL: {
                    struct minmea_sentence_dvpdl frame;
                    if (minmea_parse_dvpdl(&frame, line)) {
//                        int8_t hours; int8_t minutes; int8_t seconds; int16_t microseconds;
//                        int8_t hours_; int8_t minutes_; int8_t seconds_; int16_t microseconds_;
//                        hours = frame.dtu.hours;
//                        minutes = frame.dtu.minutes;
//                        seconds = frame.dtu.seconds;
//                        hours_ = frame.tu.hours;
//                        minutes_ = frame.tu.minutes;
//                        seconds_ = frame.tu.seconds;

                        dvl_c75_position_local_data.roll = minmea_tofloat(&frame.adr);
                        dvl_c75_position_local_data.pitch = minmea_tofloat(&frame.adp);
                        dvl_c75_position_local_data.yaw = minmea_tofloat(&frame.ady);
                        dvl_c75_position_local_data.x = minmea_tofloat(&frame.pdx);
                        dvl_c75_position_local_data.y = minmea_tofloat(&frame.pdy);
                        dvl_c75_position_local_data.z = minmea_tofloat(&frame.pdz);
                        dvl_c75_position_local_data.c = frame.c;

                        LOG_D("-----------------------------------------------------");
//                        LOG_D("hours:%-d, hours:%-d, hours:%-d", hours, minutes, seconds);
//                        LOG_D("hours_:%-d, hours_:%-d, hours_:%-d", hours_, minutes_, seconds_);
                        LOG_D("roll: %-5.3f, pitch: %-5.3f, yaw: %-5.3f",dvl_c75_position_local_data.roll, dvl_c75_position_local_data.pitch, dvl_c75_position_local_data.yaw);
                        LOG_D("x: %-5.3f, y: %-5.3f, z: %-5.3f", dvl_c75_position_local_data.x, dvl_c75_position_local_data.y, dvl_c75_position_local_data.z);

                        return 0;
                    } else {
                        LOG_D("$xxDVE sentence is not parsed\n");
                        return -1;
                    }
                } break;
        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, line)) {
                char mini_gga_data[34];
                int8_t hours; int8_t minutes; int8_t seconds; int16_t microseconds;
                float latitude; float longitude;
                int8_t fix_quality; int8_t satellites_tracked;
                float hdop;
                float altitude; char altitude_units;
                float height; char height_units;
                float dgps_age;

                hours = frame.time.hours;
                minutes = frame.time.minutes;
                seconds = frame.time.seconds;
                microseconds = frame.time.microseconds;
                latitude = minmea_tocoord(&frame.latitude);
                longitude = minmea_tocoord(&frame.longitude);
                fix_quality = frame.fix_quality;
                satellites_tracked = frame.satellites_tracked;
                hdop = minmea_tocoord(&frame.hdop);
                altitude = minmea_tocoord(&frame.altitude);
                altitude_units = frame.altitude_units;
                height = minmea_tocoord(&frame.height);
                height_units = frame.height_units;
                dgps_age = minmea_tocoord(&frame.dgps_age);

                rt_memcpy(mini_gga_data+0, &hours, sizeof(int8_t));
                rt_memcpy(mini_gga_data+1, &minutes, sizeof(int8_t));
                rt_memcpy(mini_gga_data+2, &seconds, sizeof(int8_t));
                rt_memcpy(mini_gga_data+3, &microseconds, sizeof(int16_t));
                rt_memcpy(mini_gga_data+5, &latitude, sizeof(double));
                rt_memcpy(mini_gga_data+13, &longitude, sizeof(double));
                rt_memcpy(mini_gga_data+21, &fix_quality, sizeof(int8_t));
                rt_memcpy(mini_gga_data+22, &satellites_tracked, sizeof(int8_t));
                rt_memcpy(mini_gga_data+23, &hdop, sizeof(float));
                rt_memcpy(mini_gga_data+27, &altitude, sizeof(float));
                rt_memcpy(mini_gga_data+31, &altitude_units, sizeof(char));
                rt_memcpy(mini_gga_data+32, &height, sizeof(float));
                rt_memcpy(mini_gga_data+36, &height_units, sizeof(char));
                rt_memcpy(mini_gga_data+37, &dgps_age, sizeof(float));
                RobolinkUdpCreateInitSendNewMsgToMb((uint8_t*)mini_gga_data, sizeof(mini_gga_data), get_config()->robolink_id_config.local_id, get_gps_dev_id(), 0x03);
                return 0;
            } else {
                LOG_D("$xxGGA sentence is not parsed\n");
                return -1;
            }
        } break;

        default:
            break;
    }
    return -1;
}
