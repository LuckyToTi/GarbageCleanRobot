#include "eth.h"
#include <rthw.h>
#include <rtthread.h>

/**
 * @brief       ETH底层驱动，时钟使能，引脚配置
 *    @note     此函数会被HAL_ETH_Init()调用
 * @param       heth:以太网句柄
 * @retval      无
 */
void hal_eth_init()
{
    GPIO_InitTypeDef gpio_init_struct;

    ETH_CLK_GPIO_CLK_ENABLE();          /* 开启ETH_CLK时钟 */
    ETH_MDIO_GPIO_CLK_ENABLE();         /* 开启ETH_MDIO时钟 */
    ETH_CRS_GPIO_CLK_ENABLE();          /* 开启ETH_CRS时钟 */
    ETH_MDC_GPIO_CLK_ENABLE();          /* 开启ETH_MDC时钟 */
    ETH_RXD0_GPIO_CLK_ENABLE();         /* 开启ETH_RXD0时钟 */
    ETH_RXD1_GPIO_CLK_ENABLE();         /* 开启ETH_RXD1时钟 */
    ETH_TX_EN_GPIO_CLK_ENABLE();        /* 开启ETH_TX_EN时钟 */
    ETH_TXD0_GPIO_CLK_ENABLE();         /* 开启ETH_TXD0时钟 */
    ETH_TXD1_GPIO_CLK_ENABLE();         /* 开启ETH_TXD1时钟 */
    ETH_RESET_GPIO_CLK_ENABLE();        /* 开启ETH_RESET时钟 */
    __HAL_RCC_ETH_CLK_ENABLE();         /* 开启ETH时钟 */


    /* 网络引脚设置 RMII接口
     * ETH_MDIO -------------------------> PA2
     * ETH_MDC --------------------------> PC1
     * ETH_RMII_REF_CLK------------------> PA1
     * ETH_RMII_CRS_DV ------------------> PA7
     * ETH_RMII_RXD0 --------------------> PC4
     * ETH_RMII_RXD1 --------------------> PC5
     * ETH_RMII_TX_EN -------------------> PG11
     * ETH_RMII_TXD0 --------------------> PG13
     * ETH_RMII_TXD1 --------------------> PG14
     * ETH_RESET-------------------------> PD3
     */

    /* PA1,2,7 */
    gpio_init_struct.Pin = ETH_CLK_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* 推挽复用 */
    gpio_init_struct.Pull = GPIO_NOPULL;                    /* 不带上下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_HIGH;               /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF11_ETH;             /* 复用为ETH功能 */
    HAL_GPIO_Init(ETH_CLK_GPIO_PORT, &gpio_init_struct);    /* ETH_CLK引脚模式设置 */

    gpio_init_struct.Pin = ETH_MDIO_GPIO_PIN;
    HAL_GPIO_Init(ETH_MDIO_GPIO_PORT, &gpio_init_struct);   /* ETH_MDIO引脚模式设置 */

    gpio_init_struct.Pin = ETH_CRS_GPIO_PIN;
    HAL_GPIO_Init(ETH_CRS_GPIO_PORT, &gpio_init_struct);    /* ETH_CRS引脚模式设置 */

    /* PC1 */
    gpio_init_struct.Pin = ETH_MDC_GPIO_PIN;
    HAL_GPIO_Init(ETH_MDC_GPIO_PORT, &gpio_init_struct);    /* ETH_MDC初始化 */

    /* PC4 */
    gpio_init_struct.Pin = ETH_RXD0_GPIO_PIN;
    HAL_GPIO_Init(ETH_RXD0_GPIO_PORT, &gpio_init_struct);   /* ETH_RXD0初始化 */

    /* PC5 */
    gpio_init_struct.Pin = ETH_RXD1_GPIO_PIN;
    HAL_GPIO_Init(ETH_RXD1_GPIO_PORT, &gpio_init_struct);   /* ETH_RXD1初始化 */


    /* PG11,13,14 */
    gpio_init_struct.Pin = ETH_TX_EN_GPIO_PIN;
    HAL_GPIO_Init(ETH_TX_EN_GPIO_PORT, &gpio_init_struct);  /* ETH_TX_EN初始化 */

    gpio_init_struct.Pin = ETH_TXD0_GPIO_PIN;
    HAL_GPIO_Init(ETH_TXD0_GPIO_PORT, &gpio_init_struct);   /* ETH_TXD0初始化 */

    gpio_init_struct.Pin = ETH_TXD1_GPIO_PIN;
    HAL_GPIO_Init(ETH_TXD1_GPIO_PORT, &gpio_init_struct);   /* ETH_TXD1初始化 */

    /* 复位引脚 */
    gpio_init_struct.Pin = ETH_RESET_GPIO_PIN;      /* ETH_RESET初始化 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;    /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;            /* 无上下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_HIGH;       /* 高速 */
    HAL_GPIO_Init(ETH_RESET_GPIO_PORT, &gpio_init_struct);

    ETHERNET_RST(0);     /* 硬件复位 */
    HAL_Delay(50);
    ETHERNET_RST(1);     /* 复位结束 */
    HAL_Delay(10);

    HAL_NVIC_SetPriority(ETH_IRQn, 1, 0);           /* 网络中断优先级应该高一点 */
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

void phy_reset(void) {
    ETHERNET_RST(0);     /* 硬件复位 */
    rt_thread_mdelay(50);
    ETHERNET_RST(1);     /* 复位结束 */
    rt_thread_mdelay(100);
}
