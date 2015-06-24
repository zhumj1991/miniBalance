/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * adc.c - Analog Digital Conversion
 *
 * TODO: Describe functionality.
 *
 * Sample time: According to the formula in the stm32 product manual
 *              page 69, with a Ts of 28.5 samples, 12-bit, and ADC@12
 *              the highest impedance to use is 25.2kOhm.
 */
#include "stm32f4xx_conf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "adc.h"
#include "pm.h"
#include "nvicconf.h"
#include "imu.h"

#ifdef ADC_OUTPUT_RAW_DATA
#include "uart.h"
#include "acc.h"
#endif

// PORT C
#define GPIO_VBAT        			GPIO_Pin_5

// CHANNELS
#define NBR_OF_ADC_CHANNELS   1
#define CH_VBAT               ADC_Channel_15

#define CH_VREF               ADC_Channel_Vbat
#define CH_TEMP               ADC_Channel_TempSensor

static bool isInit;
volatile AdcGroup adcValues[ADC_MEAN_SIZE];

xQueueHandle      adcQueue;

static void adcDmaInit(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  // DMA channel1 configuration
  DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adcValues[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = NBR_OF_ADC_CHANNELS * (ADC_MEAN_SIZE * 2);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  // Enable DMA channel
  DMA_Cmd(DMA2_Stream0, ENABLE);
}

/**
 * Decimates the adc samples after oversampling
 */
static void adcDecimate(AdcGroup * oversampled, AdcGroup * decimated)
{
  uint32_t i, j;
  uint32_t sum;
  uint32_t sumVref;
  AdcGroup *adcIterator;
  AdcPair  *adcOversampledPair;
  AdcPair  *adcDecimatedPair;

  // Compute sums and decimate each channel
  adcDecimatedPair = (AdcPair *)decimated;
  for (i = 0; i < NBR_OF_ADC_CHANNELS; i++)
  {
    adcIterator = oversampled;
    sum = 0;
    sumVref = 0;
    for (j = 0; j < ADC_MEAN_SIZE; j++)
    {
      adcOversampledPair = &((AdcPair *)adcIterator)[i];
      sum += adcOversampledPair->val;
      sumVref += adcOversampledPair->vref;
      adcIterator++;
    }
    // Decimate
    adcDecimatedPair->val = sum / ADC_DECIMATE_DIVEDEND;
    adcDecimatedPair->vref = sumVref / ADC_DECIMATE_DIVEDEND;
    adcDecimatedPair++;
  }
}

void adcInit(void)
{

  if(isInit)
    return;
	
  ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef		ADC_CommonInitStructure;
	
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable TIM2, GPIOA and ADC1 clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = ADC_TRIG_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = ADC_TRIG_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 4;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // TIM2 channel2 configuration in PWM mode
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
  // Halt timer 2 during debug halt.
  DBGMCU_Config(DBGMCU_TIM2_STOP, ENABLE);

  adcDmaInit();

  // ADC1 configuration
  ADC_CommonStructInit(&ADC_CommonInitStructure);
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);	
	
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 2;
  ADC_Init(ADC1, &ADC_InitStructure);

  // ADC1 channel sequence
  ADC_RegularChannelConfig(ADC1, CH_VREF, 1, ADC_SampleTime_28Cycles);
  ADC_RegularChannelConfig(ADC1, CH_VBAT, 2, ADC_SampleTime_28Cycles);

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_VBATCmd(ENABLE);
  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);

  // Enable the DMA1 channel Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_ADC_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  adcQueue = xQueueCreate(1, sizeof(AdcGroup));

  xTaskCreate(adcTask, "ADC", configMINIMAL_STACK_SIZE, NULL,
							/*priority*/3, NULL);

  isInit = true;
}

bool adcTest(void)
{
  return isInit;
}

float adcConvertToVoltageFloat(uint16_t v, uint16_t vref)
{
  return (v / (vref / ADC_INTERNAL_VREF));
}

void adcDmaStart(void)
{
  // Enable the Transfer Complete and Half Transfer Interrupt
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
  // Enable ADC1 DMA
  ADC_DMACmd(ADC1, ENABLE);
  // TIM2 counter enable
  TIM_Cmd(TIM2, ENABLE);
}

void adcDmaStop(void)
{
//  TIM_Cmd(TIM2, DISABLE);
}

void adcInterruptHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken;
  AdcGroup* adcBuffer;

  if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
  {
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
    adcBuffer = (AdcGroup *)&adcValues[0];
    xQueueSendFromISR(adcQueue, &adcBuffer, &xHigherPriorityTaskWoken);
  }
}

void adcTask(void *param)
{
  AdcGroup* adcRawValues;
  AdcGroup adcValues;

  vTaskSetApplicationTaskTag(0, (void*)TASK_ADC_ID_NBR);
  vTaskDelay(1000);

  adcDmaStart();

  while(1)
  {
    xQueueReceive(adcQueue, &adcRawValues, portMAX_DELAY);
    adcDecimate(adcRawValues, &adcValues);  // 10% CPU
    pmBatteryUpdate(&adcValues);

#ifdef ADC_OUTPUT_RAW_DATA
    uartSendDataDma(sizeof(AdcGroup)*ADC_MEAN_SIZE, (uint8_t*)adcRawValues);
#endif
  }
}
