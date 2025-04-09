#include "usart1.h"
#include "config.h"
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "rtthread.h"


/**USART1 GPIO Configuration
 * PA9   -----> USART1_TX
 * PA10  -----> USART1_RX
 */

UART_HandleTypeDef huart1;

static rt_sem_t usart1_hal_rx_idle_rxd_sem = RT_NULL;
void Hal_Usart1_Rx_Complete_IRQHandler();


void Usart1Init()
{
    usart1_hal_rx_idle_rxd_sem = rt_sem_create("U1Idle", 0, RT_IPC_FLAG_PRIO);
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    huart1.Instance = USART1;
//    huart1.Init.BaudRate = 9600;
    huart1.Init.BaudRate = get_config()->u1_config.baundrate;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID, Hal_Usart1_Rx_Complete_IRQHandler);

    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}


void USART1_IRQHandler(void)
{
    rt_interrupt_enter();
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        rt_sem_release(usart1_hal_rx_idle_rxd_sem);
    }
    HAL_UART_IRQHandler(&huart1);
    rt_interrupt_leave();
}

void Hal_Usart1_Rx_Complete_IRQHandler()
{
    rt_sem_release(usart1_hal_rx_idle_rxd_sem);
}

int Uart1HalReceiveWaitDataItStopIdle(uint8_t* buffer, uint16_t max_len)
{
    int rx_len = 0;
    int hal_ret = HAL_UART_Receive_IT(&huart1, buffer, max_len);
    if(hal_ret != HAL_OK) return -hal_ret;
    __HAL_USART_ENABLE_IT(&huart1, USART_IT_IDLE);
    rt_sem_take(usart1_hal_rx_idle_rxd_sem, RT_WAITING_FOREVER);
    rx_len = huart1.RxXferSize - huart1.RxXferCount;
    if(rx_len != max_len)
        HAL_UART_AbortReceive(&huart1);
    __HAL_USART_DISABLE_IT(&huart1, USART_IT_IDLE);
    return rx_len;
}
