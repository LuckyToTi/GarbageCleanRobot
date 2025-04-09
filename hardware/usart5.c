 #include "usart5.h"
#include "config.h"
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_uart.h>
#include <rthw.h>
#include "drv_common.h"

UART_HandleTypeDef huart5;

static rt_sem_t uart5_hal_rx_idle_rxd_sem = RT_NULL;
void Hal_Uart5_Rx_Complete_IRQHandler();

void Uart5HalInterfaceInit()
{
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    huart5.Instance = UART5;
    huart5.Init.BaudRate = get_config()->u5_config.baundrate;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
      Error_Handler();
    }

    HAL_UART_RegisterCallback(&huart5, HAL_UART_RX_COMPLETE_CB_ID, Hal_Uart5_Rx_Complete_IRQHandler);
    uart5_hal_rx_idle_rxd_sem = rt_sem_create("U5Idle", 0, RT_IPC_FLAG_PRIO);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

void UART5_IRQHandler()
{
    rt_interrupt_enter();
    if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))
    {
        rt_sem_release(uart5_hal_rx_idle_rxd_sem);
        __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    }
    HAL_UART_IRQHandler(&huart5);
    rt_interrupt_leave();
}

void Hal_Uart5_Rx_Complete_IRQHandler()
{
    rt_sem_release(uart5_hal_rx_idle_rxd_sem);
}

int Uart5HalReceiveWaitDataItStopIdle(uint8_t* buffer, uint16_t max_len)
{
    int rx_len = 0;
    int hal_ret = HAL_UART_Receive_IT(&huart5, buffer, max_len);
    if(hal_ret != HAL_OK) return -hal_ret;
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    rt_sem_take(uart5_hal_rx_idle_rxd_sem, RT_WAITING_FOREVER);
    rx_len = huart5.RxXferSize - huart5.RxXferCount;
    if(rx_len != max_len)
        HAL_UART_AbortReceive(&huart5);
    __HAL_UART_DISABLE_IT(&huart5, UART_IT_IDLE);
    return rx_len;
}
