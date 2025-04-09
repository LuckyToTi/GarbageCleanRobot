#include "iic3.h"
#include <board.h>
#include <stm32f4xx_hal.h>
#include <rthw.h>
#include <rtthread.h>

// 片上i2c总线为
// I2C3_SCL PA8
// I2C3_SDA PC9

I2C_HandleTypeDef hi2c3 = {};
static const int iic3_wait_time_ms_ = 300;  // iic3等待时间
static rt_mutex_t iic3_lock_mutex_ = RT_NULL;  // I2c3BspInit()时，同时初始化该mutex
static rt_sem_t iic3_mem_tx_sem_ = RT_NULL;  // iic3 MEM_RX完成中断信号
static rt_sem_t iic3_mem_rx_sem_ = RT_NULL;  // iic3 MEM_RX完成中断信号
static void __HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
static void __HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);

inline I2C_HandleTypeDef* get_hi2c3_handle()
{
    return &hi2c3;
}

// 专为读取配置时，初始化iic
void I2c3BspInit_()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    __HAL_RCC_I2C3_CLK_ENABLE();

    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
      Error_Handler();
    }
}

void I2c3BspInit()
{
    // 反初始化，重新执行初始化
    HAL_I2C_DeInit(&hi2c3);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    __HAL_RCC_I2C3_CLK_ENABLE();
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);

    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
      Error_Handler();
    }

    HAL_I2C_RegisterCallback(&hi2c3, HAL_I2C_MEM_TX_COMPLETE_CB_ID, __HAL_I2C_MemTxCpltCallback);
    HAL_I2C_RegisterCallback(&hi2c3, HAL_I2C_MEM_RX_COMPLETE_CB_ID, __HAL_I2C_MemRxCpltCallback);

    iic3_mem_tx_sem_ = rt_sem_create("iic3MTS", 0, RT_IPC_FLAG_PRIO);
    iic3_mem_rx_sem_ = rt_sem_create("iic3MRS", 0, RT_IPC_FLAG_PRIO);
    iic3_lock_mutex_ = rt_mutex_create("iic3_l", RT_IPC_FLAG_PRIO);  // init iic3 lock (mutex)
    if (iic3_lock_mutex_ == RT_NULL) {
      Error_Handler();
    }
}

// this function handles I2C3 event interrupt
void I2C3_EV_IRQHandler()
{
    rt_interrupt_enter();
    HAL_I2C_EV_IRQHandler(&hi2c3);
    rt_interrupt_leave();
}

// this function handles I2C3 error interrupt
void I2C3_ER_IRQHandler()
{
    rt_interrupt_enter();
    HAL_I2C_ER_IRQHandler(&hi2c3);
    rt_interrupt_leave();
}

static void __HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C3) {
        rt_sem_release(iic3_mem_tx_sem_);
    }
}

static void __HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C3) {
        rt_sem_release(iic3_mem_rx_sem_);
    }
}

// 成功返回:0
int I2c3MemWrite(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    int err_ret = rt_mutex_take(iic3_lock_mutex_, iic3_wait_time_ms_);
    if(RT_EOK == err_ret){
        err_ret = HAL_I2C_Mem_Write_IT(&hi2c3, DevAddress, MemAddress, MemAddSize, pData, Size);
        if(err_ret == HAL_OK) {
            err_ret = rt_sem_take(iic3_mem_tx_sem_, iic3_wait_time_ms_);
        }
        rt_mutex_release(iic3_lock_mutex_);
    }
    return err_ret;
}

// 成功返回:0
int I2c3MemRead(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    int err_ret = rt_mutex_take(iic3_lock_mutex_, iic3_wait_time_ms_);
    if(RT_EOK == err_ret){
        err_ret = HAL_I2C_Mem_Read_IT(&hi2c3, DevAddress, MemAddress, MemAddSize, pData, Size);
        if(err_ret == HAL_OK) {
            err_ret = rt_sem_take(iic3_mem_rx_sem_, iic3_wait_time_ms_);
        }
        rt_mutex_release(iic3_lock_mutex_);
    }
    return err_ret;
}

// 成功返回:0
int I2c3MemRead_NoRtos(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    return HAL_I2C_Mem_Read(&hi2c3, DevAddress, MemAddress, MemAddSize, pData, Size, 1000);
}
