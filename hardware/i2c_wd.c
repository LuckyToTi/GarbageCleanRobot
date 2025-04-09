#include "i2c_wd.h"

/**Deep GPIO Configuration
 * PB14  -----> DEEP_SDA
 * PB15  -----> DEEP_SCL
 */

static Softi2c_device_t ms5837_i2c_device;

#define WD_I2C_SCL_IO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define WD_I2C_SDA_IO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#define WD_I2C_SCL_PORT    GPIOB
#define WD_I2C_SCL_PIN     LL_GPIO_PIN_15
#define WD_I2C_SCL_PIN_HAL GPIO_PIN_15

#define WD_I2C_SDA_PORT     GPIOB
#define WD_I2C_SDA_PIN      LL_GPIO_PIN_14
#define WD_I2C_SDA_PIN_HAL  GPIO_PIN_14

static void Ms5837_Set_Sda_Out(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {};
    GPIO_InitStructure.Pin = WD_I2C_SDA_PIN_HAL;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(WD_I2C_SDA_PORT, &GPIO_InitStructure);
}

static void Ms5837_Set_Sda_In(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {};
    GPIO_InitStructure.Pin = WD_I2C_SDA_PIN_HAL;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(WD_I2C_SDA_PORT, &GPIO_InitStructure);
}

static void Ms5837_Set_Scl_Level(uint8_t level)
{
    if(level)
    {
        LL_GPIO_SetOutputPin(WD_I2C_SCL_PORT, WD_I2C_SCL_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(WD_I2C_SCL_PORT, WD_I2C_SCL_PIN);
    }
}

static void Ms5837_Set_Sda_Level(uint8_t level)
{
    if(level)
    {
        LL_GPIO_SetOutputPin(WD_I2C_SDA_PORT, WD_I2C_SDA_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(WD_I2C_SDA_PORT, WD_I2C_SDA_PIN);
    }
}

static uint8_t Ms5837_Read_Sda_Level(void)
{
    return LL_GPIO_IsInputPinSet(WD_I2C_SDA_PORT, WD_I2C_SDA_PIN);
}

static void Ms5837_Softi2c_Delay(unsigned int time)
{
    rt_hw_us_delay(time);
}

void Ms5837_Softi2c_Gpio_Init(void)
{
    WD_I2C_SCL_IO_CLK_ENABLE();
    WD_I2C_SDA_IO_CLK_ENABLE();

    LL_GPIO_SetOutputPin(WD_I2C_SCL_PORT, WD_I2C_SCL_PIN);
    LL_GPIO_SetOutputPin(WD_I2C_SDA_PORT, WD_I2C_SDA_PIN);

    GPIO_InitTypeDef GPIO_InitStructure = {};
    GPIO_InitStructure.Pin = WD_I2C_SCL_PIN_HAL;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(WD_I2C_SCL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = WD_I2C_SDA_PIN_HAL;
    HAL_GPIO_Init(WD_I2C_SDA_PORT, &GPIO_InitStructure);
}

void Ms5837_Softi2c_Init(void)
{
    Softi2c_Device_Deinit(&ms5837_i2c_device);
    ms5837_i2c_device.Set_Sda_In_Mode = Ms5837_Set_Sda_In;
    ms5837_i2c_device.Set_Sda_Out_Mode = Ms5837_Set_Sda_Out;
    ms5837_i2c_device.I2c_Scl = Ms5837_Set_Scl_Level;
    ms5837_i2c_device.I2c_Sda = Ms5837_Set_Sda_Level;
    ms5837_i2c_device.Read_Sda = Ms5837_Read_Sda_Level;
    ms5837_i2c_device.Softi2c_Delay_Us = Ms5837_Softi2c_Delay;
    ms5837_i2c_device.delay_time_us = 5;
    Softi2c_Device_Init_Check(&ms5837_i2c_device);
}

const Softi2c_device_t* Get_Ms5837_I2c_Device(void)
{
    return &ms5837_i2c_device;
}
