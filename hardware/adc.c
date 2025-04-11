#include "adc.h"

#define ADC_BUFFER_SIZE     0x04
#define ADC_FILTER_SIMPLE   0x08

__ALIGN_BEGIN static volatile uint16_t adc1_data[ADC_BUFFER_SIZE] __ALIGN_END;
static uint16_t adc1_data_filter[ADC_BUFFER_SIZE];
static rt_sem_t adc1_filter_sem = RT_NULL;

static void ADC_ProcessData(void);

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = { 0 };

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 4;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;

    HAL_ADC_MspInit(&hadc1);
    HAL_ADC_Init(&hadc1);

    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = 3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = 4;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1_data, ADC_BUFFER_SIZE);
    adc1_filter_sem = rt_sem_create("adc_sem", 1, RT_IPC_FLAG_PRIO);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    if (adcHandle->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_ENABLE()
        ;
        __HAL_RCC_GPIOA_CLK_ENABLE()
        ;
        __HAL_RCC_DMA2_CLK_ENABLE()
        ;

        /**ADC1 GPIO Configuration
         PA3     ------> ADC1_IN3
         PA4     ------> ADC1_IN4
         PA5     ------> ADC1_IN5
         PA6     ------> ADC1_IN6
         */
        GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        hdma_adc1.Instance = DMA2_Stream0;
        hdma_adc1.Init.Channel = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
        {
        }

        __HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_TC);
        HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{
    if (adcHandle->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

        if (adc1_filter_sem != RT_NULL)
        {
            rt_sem_delete(adc1_filter_sem);
            adc1_filter_sem = RT_NULL;
        }
    }
}

static void ADC_ProcessData(void)
{
    static uint16_t filter_buf[ADC_BUFFER_SIZE][ADC_FILTER_SIMPLE] = { 0 };
    static uint8_t index = 0;

    if (rt_sem_take(adc1_filter_sem, RT_WAITING_FOREVER) == RT_EOK)
    {
        for (int i = 0; i < ADC_BUFFER_SIZE; i++)
        {
            filter_buf[i][index] = adc1_data[i];
        }
        index = (index + 1) % ADC_FILTER_SIMPLE;

        for (int i = 0; i < ADC_BUFFER_SIZE; i++)
        {
            uint32_t sum = 0;
            for (int j = 0; j < ADC_FILTER_SIMPLE; j++)
            {
                sum += filter_buf[i][j];
            }
            adc1_data_filter[i] = sum / ADC_FILTER_SIMPLE;
        }
    }
}

void DMA2_Stream0_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&hdma_adc1);
    rt_sem_release(adc1_filter_sem);
    rt_interrupt_leave();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        ADC_ProcessData();
    }
}

uint16_t Get_ADC_FilterValue_Safe(uint32_t channel)
{
    uint16_t result = 0;

    switch (channel)
    {
    case ADC_CHANNEL_3:
        result = adc1_data_filter[0];
        break;
    case ADC_CHANNEL_4:
        result = adc1_data_filter[1];
        break;
    case ADC_CHANNEL_5:
        result = adc1_data_filter[2];
        break;
    case ADC_CHANNEL_6:
        result = adc1_data_filter[3];
        break;
    default:
        break;
    }

    return result;
}
