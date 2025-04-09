#include "rov_move_thread.h"
#include "controller_switch_thread.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "hardware/motor_control_center.h"
#include "movelib/rov_move.h"
#include <can1_thread.h>

#define LOG_TAG     "e_t"
#define LOG_LVL     LOG_LVL_INFO
#include <ulog.h>

static int LeakageDetection();
uint8_t ComputeCheckSum(const uint8_t *data, size_t len);
uint32_t AngleToPulse(float angle_deg);
void SetPulseBytes(uint8_t* data, int32_t pulse);

uint8_t Can_data[8] = {0xfe,0x02,0x58,0x02,0x00,0x40,0x00,0x9b}; //  0x4000为一圈
uint8_t Reset_Zero[2] = {0x92,0x93};  // 重置消息帧
static uint8_t Reset_Flag = 0;

rt_thread_t equipment_tid = RT_NULL;  // 机器人设备控制模块 摄像头 机械手 水下灯
rt_thread_t* get_equipment_tid(){
    return &equipment_tid;
}

static int gripper_absolute_angle = 0;

void EquipmentThreadEntry(void* parameter){
    rt_thread_delay(1000);
    for( ; ; ){
        int16_t equipment_output[6] = {0, 0, 0, 0, 0, 0};
        #if defined(EQUIPMENT_CLOUD_PLATFORM) && EQUIPMENT_CLOUD_PLATFORM == 1
            uint16_t cloud_platform_pitch = 1500 - get_control_msg()->rov_control.cloud_platform_pitch * 10.0f;    //云台俯仰角度
            uint16_t cloud_platform_revolve = 1620 + get_control_msg()->rov_control.cloud_platform_revolve * 3.2f;    //云台旋转角度
            equipment_output[0] = cloud_platform_pitch;
            equipment_output[1] = cloud_platform_revolve;
            LOG_D("cloud_platform_pitch: %d, cloud_platform_revolve: %d\r\n",cloud_platform_pitch, cloud_platform_revolve);
        #endif
        #if defined(EQUIPMENT_CAMERA) && EQUIPMENT_CAMERA == 1
            uint16_t camera_angle = 1500 + get_control_msg()->rov_control.camera_pitch * 6.0f;              //摄像头角度
            equipment_output[2] = camera_angle;
            LOG_D("camera_angle: %d\r\n",camera_angle);
        #endif
        #if defined(EQUIPMENT_LED) && EQUIPMENT_LED == 1
            uint16_t led_brightness = 1100 + get_control_msg()->rov_control.led_brightness * 6.0f;         //LED亮度控制
            equipment_output[3] = led_brightness;
            LOG_D("led_brightness: %d\r\n",led_brightness);
        #endif
//            get_control_msg()->equip_switch.sonar ? PD15_PWM_OUT_US(20001) : PD15_PWM_OUT_US(0);
            if(get_control_msg()->equip_switch.sonar) {
                PD15_PWM_OUT_US(20001);
            } else {
                PD15_PWM_OUT_US(0);
            }
            LOG_D("sonar : %d\r\n",get_control_msg()->equip_switch.sonar);
            PB1_PWM_OUT_US(equipment_output[2]);

//-----------------------------can机械手舵机部分--------------------------------------------------------------------------------------------------------------------------------------------------
            if(Reset_Flag == 0){
                Can1CreateInitSendNewMsgToMb(0x01,CAN_ID_STD,CAN_RTR_DATA,sizeof(Reset_Zero),(uint8_t*)Reset_Zero);
                Reset_Flag = 1;
            }
            uint32_t conversion_value; //  角度转换成脉冲数
            gripper_absolute_angle = get_control_msg()->rov_control.led_brightness == 100 ? 90 : 0;
            conversion_value = AngleToPulse(gripper_absolute_angle);
            SetPulseBytes(&Can_data[4],conversion_value);
            Can_data[7] = ComputeCheckSum((uint8_t*)Can_data,sizeof(Can_data) -1);
            LOG_D("%x %x %x %x %x %x %x %x",Can_data[0],Can_data[1],Can_data[2],Can_data[3],Can_data[4],Can_data[5],Can_data[6],Can_data[7]);
            Can1CreateInitSendNewMsgToMb(0x01,CAN_ID_STD,CAN_RTR_DATA,sizeof(Can_data),(uint8_t*)Can_data); //  can机械手舵机控制
//-------------------------------------------------------------------------------------------------------------------------------------------
        #if defined(EQUIPMENT_IRON_HAND) && EQUIPMENT_IRON_HAND == 1
            uint16_t iron_hand_angle = 1500 + get_control_msg()->rov_control.gripper_angle * 10.0f;                      //机械手开合
            equipment_output[4] = iron_hand_angle;
            LOG_D("iron_hand_angle: %d\r\n",iron_hand_angle);
            uint16_t iron_hand_speed = 1500 + get_control_msg()->rov_control.gripper_speed * 10.0f;                      //机械手夹放
            equipment_output[5] = iron_hand_speed;
            LOG_D("iron_hand_speed: %d\r\n",iron_hand_speed);
        #endif

        #if defined(PUSHES_OUT_SWITCH) && PUSHES_OUT_SWITCH == 0
            PE5_PWM_OUT_US(equipment_output[0]);
            PE6_PWM_OUT_US(equipment_output[1]);
//          PE5_PWM_OUT_US(equipment_output[2]);
//          PE6_PWM_OUT_US(equipment_output[3]);
            PB0_PWM_OUT_US(equipment_output[3]);
//          PB1_PWM_OUT_US(equipment_output[3]);
        #elif defined(PUSHES_OUT_SWITCH) && PUSHES_OUT_SWITCH == 2
            EquipmentTxDataConvertAndSend(0, equipment_output[0], equipment_output[1],equipment_output[3], 3);
        #endif
        rt_thread_delay(25);
    }
}

inline static int LeakageDetection() {
    static int cnt = 0;
    uint16_t adc = get_adc();
    if(adc < 4000) {
        if((++cnt) > 40) {
            cnt = 0;
            return 1;
        }
    } else {
        if(cnt > 0) cnt--;
    }
    return 0;
}

// 计算校验和
uint8_t ComputeCheckSum(const uint8_t *data, size_t len) {
    uint16_t sum = 1;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF); // 取低8位
}

// 将角度（0~360）映射为脉冲数（16细分时：360° = 3200脉冲）
uint32_t AngleToPulse(float angle_deg) {
    const float PULSE_PER_DEGREE = 16384.0f / 360.0f;  // ≈ 8.888...
    if (angle_deg < 0.0f) angle_deg = 0.0f;
    if (angle_deg > 360.0f) angle_deg = 360.0f;
    return (uint32_t)(angle_deg * PULSE_PER_DEGREE + 0.5f); // 四舍五入
}

// 将 24bit 整数值写入 3 个字节
void SetPulseBytes(uint8_t* data, int32_t pulse) {
    // 限制范围在 int24
    if (pulse > 0x7FFFFF) pulse = 0x7FFFFF;
    if (pulse < -0x800000) pulse = -0x800000;

    data[2] = (uint8_t)(pulse & 0xFF);         // 低位
    data[1] = (uint8_t)((pulse >> 8) & 0xFF);  // 中位
    data[0] = (uint8_t)((pulse >> 16) & 0xFF); // 高位
}


