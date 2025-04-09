#include "usart2.h"
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_uart.h>
#include <rtthread.h>

/**USART2 GPIO Configuration
 * PD5  -----> USART2_TX
 * PD6  -----> USART2_RX
 */

UART_HandleTypeDef huart2;

static rt_sem_t usart2_hal_rx_idle_rxd_sem = RT_NULL;
static rt_sem_t usart2_hal_tx_sem_ = RT_NULL;

static rt_mutex_t usart2_lock_mutex_ = RT_NULL;
static const int usart2_wait_time_ms_ = 300;
void Hal_Usart2_Rx_Complete_IRQHandler();
void Hal_Usart2_Tx_Complete_IRQHandler();

void Usart2Init()
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    HAL_UART_RegisterCallback(&huart2, HAL_UART_RX_COMPLETE_CB_ID, Hal_Usart2_Rx_Complete_IRQHandler);
    HAL_UART_RegisterCallback(&huart2, HAL_UART_TX_COMPLETE_CB_ID, Hal_Usart2_Tx_Complete_IRQHandler);

    usart2_hal_rx_idle_rxd_sem = rt_sem_create("U2Idle", 0, RT_IPC_FLAG_PRIO);
    usart2_hal_tx_sem_ = rt_sem_create("usart2MTS", 0, RT_IPC_FLAG_PRIO);
    usart2_lock_mutex_ = rt_mutex_create("usart2_l", RT_IPC_FLAG_PRIO);

    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

int Usart2HalSendDataIt(uint8_t* tx_buffer_address, uint16_t tx_buffer_len)
{
    int err_ret = rt_mutex_take(usart2_lock_mutex_, usart2_wait_time_ms_);
    if(RT_EOK == err_ret) {
        err_ret = HAL_UART_Transmit_IT(&huart2, tx_buffer_address, tx_buffer_len);
        if(err_ret == HAL_OK) {
            err_ret = rt_sem_take(usart2_hal_tx_sem_, usart2_wait_time_ms_);
        }
        rt_mutex_release(usart2_lock_mutex_);
    }
    return err_ret;
}

void USART2_IRQHandler(void)
{
    rt_interrupt_enter();
    if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET)
    {
       __HAL_UART_CLEAR_IDLEFLAG(&huart2);
       rt_sem_release(usart2_hal_rx_idle_rxd_sem);
    }
    HAL_UART_IRQHandler(&huart2);
    rt_interrupt_leave();
}

void Hal_Usart2_Rx_Complete_IRQHandler()
{
    rt_sem_release(usart2_hal_rx_idle_rxd_sem);
}

void Hal_Usart2_Tx_Complete_IRQHandler()
{
    rt_sem_release(usart2_hal_tx_sem_);
}

int Usart2HalReceiveWaitDataItStopIdle(uint8_t* buffer, uint16_t max_len)
{
    int rx_len = 0;
    int hal_ret = HAL_UART_Receive_IT(&huart2,buffer,max_len);
    if(hal_ret != HAL_OK) return -hal_ret;

    __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
    rt_sem_take(usart2_hal_rx_idle_rxd_sem, RT_WAITING_FOREVER);
    rx_len = huart2.RxXferSize - huart2.RxXferCount;
    if(rx_len != max_len)
        HAL_UART_AbortReceive(&huart2);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
    return rx_len;
}

int Usart2HalReceiveWaitTimeoutDataItStopIdle(uint8_t* buffer, uint16_t max_len, int timeout)
{
    int rx_len = 0;
    int hal_ret = HAL_UART_Receive_IT(&huart2,buffer,max_len);
    if(hal_ret != HAL_OK) return -hal_ret;

    __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
    rt_err_t ret = rt_sem_take(usart2_hal_rx_idle_rxd_sem, timeout);
    if(ret != RT_EOK) {
        HAL_UART_AbortReceive(&huart2);
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
        return -1;
    }
    rx_len = huart2.RxXferSize - huart2.RxXferCount;
    if(rx_len != max_len)
        HAL_UART_AbortReceive(&huart2);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
    return rx_len;
}
