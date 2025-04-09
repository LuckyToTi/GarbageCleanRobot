#ifndef __ETH_H_
#define __ETH_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"

/* 引脚 定义 */
#define ETH_CLK_GPIO_PORT               GPIOA
#define ETH_CLK_GPIO_PIN                GPIO_PIN_1
#define ETH_CLK_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOA_CLK_ENABLE();}while(0)                  /* 所在IO口时钟使能 */

#define ETH_MDIO_GPIO_PORT              GPIOA
#define ETH_MDIO_GPIO_PIN               GPIO_PIN_2
#define ETH_MDIO_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOA_CLK_ENABLE();}while(0)                 /* 所在IO口时钟使能 */

#define ETH_CRS_GPIO_PORT               GPIOA
#define ETH_CRS_GPIO_PIN                GPIO_PIN_7
#define ETH_CRS_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOA_CLK_ENABLE();}while(0)                  /* 所在IO口时钟使能 */

#define ETH_MDC_GPIO_PORT               GPIOC
#define ETH_MDC_GPIO_PIN                GPIO_PIN_1
#define ETH_MDC_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOC_CLK_ENABLE();}while(0)                  /* 所在IO口时钟使能 */

#define ETH_RXD0_GPIO_PORT              GPIOC
#define ETH_RXD0_GPIO_PIN               GPIO_PIN_4
#define ETH_RXD0_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE();}while(0)                  /* 所在IO口时钟使能 */

#define ETH_RXD1_GPIO_PORT              GPIOC
#define ETH_RXD1_GPIO_PIN               GPIO_PIN_5
#define ETH_RXD1_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE();}while(0)                  /* 所在IO口时钟使能 */

#define ETH_TX_EN_GPIO_PORT             GPIOB
#define ETH_TX_EN_GPIO_PIN              GPIO_PIN_11
#define ETH_TX_EN_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE();}while(0)                  /* 所在IO口时钟使能 */

#define ETH_TXD0_GPIO_PORT              GPIOB
#define ETH_TXD0_GPIO_PIN               GPIO_PIN_12
#define ETH_TXD0_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE();}while(0)                  /* 所在IO口时钟使能 */

#define ETH_TXD1_GPIO_PORT              GPIOB
#define ETH_TXD1_GPIO_PIN               GPIO_PIN_13
#define ETH_TXD1_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE();}while(0)                  /* 所在IO口时钟使能 */

#define ETH_RESET_GPIO_PORT             GPIOC
#define ETH_RESET_GPIO_PIN              GPIO_PIN_0
#define ETH_RESET_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOC_CLK_ENABLE();}while(0)                  /* 所在IO口时钟使能 */


/* ETH端口定义 */
#define ETHERNET_RST(x)  do{ x ? \
                            HAL_GPIO_WritePin(ETH_RESET_GPIO_PORT, ETH_RESET_GPIO_PIN, GPIO_PIN_SET) : \
                            HAL_GPIO_WritePin(ETH_RESET_GPIO_PORT, ETH_RESET_GPIO_PIN, GPIO_PIN_RESET); \
                        }while(0)

void hal_eth_init();

#endif  // __ETH_H_
