/**
  ******************************************************************************
  * @file    stm32f4xx_adc.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    05-March-2012
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Analog to Digital Convertor (ADC) peripheral:
  *           - Initialization and Configuration (in addition to ADC multi mode 
  *             selection)
  *           - Analog Watchdog configuration
  *           - Temperature Sensor & Vrefint (Voltage Reference internal) & VBAT
  *             management 
  *           - Regular Channels Configuration
  *           - Regular Channels DMA Configuration
  *           - Injected channels Configuration
  *           - Interrupts and flags management
  *         
  *  @verbatim
  *
  *          ===================================================================
  *                                   如何使用这个驱动
  *          ===================================================================

  *          1.  使能ＡＤＣ接口的时钟 
  *                  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADCx, ENABLE); 
  *     
  *          2. ADC引脚配置
  *               允许使用以下功能的ADC个时钟：
  *                 － RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);   
  *               － 在使用gpio_init()函数配置这些ADC引脚；
  *
  *          3. 配置ADC分频器，转换的分辨率和数据
  *              使用  ADC_Init() 函数。
  *          4. 使用 ADC_Cmd() 函数启动ＡＤＣ外设.
  *
  *          正规ADC通道组配置
  *          ====================================    
  *            - 要配置ADC正规通道组功能，使用
  *              ADC_Init()初始化 和 ADC_RegularChannelConfig() 函数.
  *            - 要使用连续模式,请使用 ADC_continuousModeCmd()函数。
  *     
  *            - 要配置和使用非连续模式, 请使用
  *              ADC_DiscModeChannelCountConfig() 和 ADC_DiscModeCmd() 函数.
  *            - 要读取ADC转换后的值,请使用ADC_GetConversionValue()函数。
  *             
  *
  *          多模式ADC的正规渠道配置
  *          ===============================================
  *            - 请参阅 "正规ADC通道组配置" 说明
  *              配置的ADC1，ADC2和ADC3定期的通道。      
  *            - 选择多模式ADC正规通道功能（双重或三重模式）
  *              使用ADC_CommonInit（）函数和配置
  *              DMA 模式使用 ADC_MultiModeDMARequestAfterLastTransferCmd()函数 
  *                  
  *            - ADC的转换值，使用	ADC_GetMultiModeConversionValue() 函数
  *              
  *
  *          正规通道组的DMA功能配置
  *          ====================================================== 
  *           - 要启用DMA模式为正规渠道组, 请使用  ADC_DMACmd() 函数。
  *             
  *           - 为使 DMA 产生连续的DMA请求
  *             请使用 ADC_DMARequestAfterLastTransferCmd() 函数
  *           
  *
  *          注入通道组配置
  *          =====================================    
  *            - 要配置ADC注入通道组功能, 请使用
  *              ADC_InjectedChannelConfig() 和  ADC_InjectedSequencerLengthConfig()
  *              函数。
  *            - 要使用连续模式,请使用 ADC_continuousModeCmd() 函数
  *              
  *            - 要使用注入的非连续模式, 请使用
  *               ADC_InjectedDiscModeCmd() 函数
  *            - 要使用的AutoInjected模式, 请使用 ADC_AutoInjectedConvCmd()  函数
  *              
  *            - 要读取ADC转换后的值, 请使用 ADC_GetInjectedConversionValue()函数 
  *              
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @defgroup ADC 
  * @brief ADC driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 

/* ADC DISCNUM mask */
#define CR1_DISCNUM_RESET         ((uint32_t)0xFFFF1FFF)

/* ADC AWDCH mask */
#define CR1_AWDCH_RESET           ((uint32_t)0xFFFFFFE0)   

/* ADC Analog watchdog enable mode mask */
#define CR1_AWDMode_RESET         ((uint32_t)0xFF3FFDFF)   

/* CR1 register Mask */
#define CR1_CLEAR_MASK            ((uint32_t)0xFCFFFEFF)

/* ADC EXTEN mask */
#define CR2_EXTEN_RESET           ((uint32_t)0xCFFFFFFF)  

/* ADC JEXTEN mask */
#define CR2_JEXTEN_RESET          ((uint32_t)0xFFCFFFFF)  

/* ADC JEXTSEL mask */
#define CR2_JEXTSEL_RESET         ((uint32_t)0xFFF0FFFF)  

/* CR2 register Mask */
#define CR2_CLEAR_MASK            ((uint32_t)0xC0FFF7FD)

/* ADC SQx mask */
#define SQR3_SQ_SET               ((uint32_t)0x0000001F)  
#define SQR2_SQ_SET               ((uint32_t)0x0000001F)  
#define SQR1_SQ_SET               ((uint32_t)0x0000001F)  

/* ADC L Mask */
#define SQR1_L_RESET              ((uint32_t)0xFF0FFFFF) 

/* ADC JSQx mask */
#define JSQR_JSQ_SET              ((uint32_t)0x0000001F) 

/* ADC JL mask */
#define JSQR_JL_SET               ((uint32_t)0x00300000) 
#define JSQR_JL_RESET             ((uint32_t)0xFFCFFFFF) 

/* ADC SMPx mask */
#define SMPR1_SMP_SET             ((uint32_t)0x00000007)  
#define SMPR2_SMP_SET             ((uint32_t)0x00000007) 

/* ADC JDRx registers offset */
#define JDR_OFFSET                ((uint8_t)0x28) 

/* ADC CDR register base address */
#define CDR_ADDRESS               ((uint32_t)0x40012308)   

/* ADC CCR register Mask */
#define CR_CLEAR_MASK             ((uint32_t)0xFFFC30E0)  

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup ADC_Private_Functions
  * @{
  */ 

/** @defgroup ADC_Group1 Initialization and Configuration functions
 *  @brief    Initialization and Configuration functions 
 *
@verbatim    
 ===============================================================================
                      初始化和配置功能
 ===============================================================================  
  本节规定允许的功能：
   - Initialize and configure the ADC Prescaler   	初始化和配置ADC预分频器
   - ADC Conversion Resolution (12bit..6bit)		 ADC转换精度(12bit..6bit)
   - Scan Conversion Mode (multichannels or one channel) for regular group  扫描转换模式(多通道或一个通道)常规组
   - ADC Continuous Conversion Mode (Continuous or Single conversion) for 	ADC连续转换模式(连续或单次转换)常规组

     regular group
   - External trigger Edge and source of regular group, 外部触发边缘和源的常规组
   - Converted data alignment (left or right)			转换后的数据对齐（左或右）
   - The number of ADC conversions that will be done using the sequencer for    ADC转换将使用音序器的数量,正规渠道组

     regular channel group
   - Multi ADC mode selection	 多ADC模式选择
   - Direct memory access mode selection for multi ADC mode   多ADC模式的直接记忆体存取模式选择
   - Delay between 2 sampling phases (used in dual or triple interleaved modes)	2个采样相位之间的延迟（用于双重或三重交错模式）
   - Enable or disable the ADC peripheral	  启用或禁用ADC外设
   
@endverbatim
  * @{
  */

/**
  * @功能 :  将ADCs 的外设的寄存器复位默认值。
  *         
  * @param  None
  * @retval None
  */
void ADC_DeInit(void)
{
  /* Enable all ADCs reset state */
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC, ENABLE);
  
  /* Release all ADCs from reset state */
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC, DISABLE);
}

/**
  * @brief  根据指定的参数初始化使用ADCx外设 
  *         在ADC_InitStruct结构体中。
  * @note   这个函数是用来配置全局功能的 ADC (分辨率和数据)
  *         但是, 其余的配置的具体的参数参见正规通道组  
  *         (扫描模式激活, 连续模式下激活, 外部触发源和边缘，
  *          在正规渠道组定序数转换).  
  * @param  ADCx: 其中x可以是1，2或3，选择ADC外围设备。
  * @param  ADC_InitStruct: 是一个ADC_InitTypeDef的结构体指针，它包含
  *         指定的ADC外设的配置信息.
  * @retval None
  */
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct)
{
  uint32_t tmpreg1 = 0;
  uint8_t tmpreg2 = 0;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_RESOLUTION(ADC_InitStruct->ADC_Resolution)); 
  assert_param(IS_FUNCTIONAL_STATE(ADC_InitStruct->ADC_ScanConvMode));
  assert_param(IS_FUNCTIONAL_STATE(ADC_InitStruct->ADC_ContinuousConvMode)); 
  assert_param(IS_ADC_EXT_TRIG_EDGE(ADC_InitStruct->ADC_ExternalTrigConvEdge)); 
  assert_param(IS_ADC_EXT_TRIG(ADC_InitStruct->ADC_ExternalTrigConv));    
  assert_param(IS_ADC_DATA_ALIGN(ADC_InitStruct->ADC_DataAlign)); 
  assert_param(IS_ADC_REGULAR_LENGTH(ADC_InitStruct->ADC_NbrOfConversion));
  
  /*---------------------------- ADCx CR1 Configuration -----------------*/
  /* Get the ADCx CR1 value */
  tmpreg1 = ADCx->CR1;
  
  /* Clear RES and SCAN bits */
  tmpreg1 &= CR1_CLEAR_MASK;
  
  /* Configure ADCx: scan conversion mode and resolution */
  /* Set SCAN bit according to ADC_ScanConvMode value */
  /* Set RES bit according to ADC_Resolution value */ 
  tmpreg1 |= (uint32_t)(((uint32_t)ADC_InitStruct->ADC_ScanConvMode << 8) | \
                                   ADC_InitStruct->ADC_Resolution);
  /* Write to ADCx CR1 */
  ADCx->CR1 = tmpreg1;
  /*---------------------------- ADCx CR2 Configuration -----------------*/
  /* Get the ADCx CR2 value */
  tmpreg1 = ADCx->CR2;
  
  /* Clear CONT, ALIGN, EXTEN and EXTSEL bits */
  tmpreg1 &= CR2_CLEAR_MASK;
  
  /* Configure ADCx: external trigger event and edge, data alignment and 
     continuous conversion mode */
  /* Set ALIGN bit according to ADC_DataAlign value */
  /* Set EXTEN bits according to ADC_ExternalTrigConvEdge value */ 
  /* Set EXTSEL bits according to ADC_ExternalTrigConv value */
  /* Set CONT bit according to ADC_ContinuousConvMode value */
  tmpreg1 |= (uint32_t)(ADC_InitStruct->ADC_DataAlign | \
                        ADC_InitStruct->ADC_ExternalTrigConv | 
                        ADC_InitStruct->ADC_ExternalTrigConvEdge | \
                        ((uint32_t)ADC_InitStruct->ADC_ContinuousConvMode << 1));
                        
  /* Write to ADCx CR2 */
  ADCx->CR2 = tmpreg1;
  /*---------------------------- ADCx SQR1 Configuration -----------------*/
  /* Get the ADCx SQR1 value */
  tmpreg1 = ADCx->SQR1;
  
  /* Clear L bits */
  tmpreg1 &= SQR1_L_RESET;
  
  /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfConversion value */
  tmpreg2 |= (uint8_t)(ADC_InitStruct->ADC_NbrOfConversion - (uint8_t)1);
  tmpreg1 |= ((uint32_t)tmpreg2 << 20);
  
  /* Write to ADCx SQR1 */
  ADCx->SQR1 = tmpreg1;
}

/**
  * @brief  填充每个 ADC_InitStruct 默认的成员值.
  * @note   此功能用于初始化的ADC的全局特征ADC ( Resolution and Data Alignment) 
  *        	然而，其余的配置的参数请参见正规通道道组配置
  *         
  *         (扫描模式激活, 连续模式下激活, 外部触发源和边沿在正规渠道组定序数转换). 
  *           
  * @param  ADC_InitStruct: 是一个ADC_InitTypeDef的结构指针被初始化。
  *         
  * @retval None
  */
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
{
  /* Initialize the ADC_Mode member */
  ADC_InitStruct->ADC_Resolution = ADC_Resolution_12b;

  /* initialize the ADC_ScanConvMode member */
  ADC_InitStruct->ADC_ScanConvMode = DISABLE;

  /* Initialize the ADC_ContinuousConvMode member */
  ADC_InitStruct->ADC_ContinuousConvMode = DISABLE;

  /* Initialize the ADC_ExternalTrigConvEdge member */
  ADC_InitStruct->ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;

  /* Initialize the ADC_ExternalTrigConv member */
  ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;

  /* Initialize the ADC_DataAlign member */
  ADC_InitStruct->ADC_DataAlign = ADC_DataAlign_Right;

  /* Initialize the ADC_NbrOfConversion member */
  ADC_InitStruct->ADC_NbrOfConversion = 1;
}

/**
  * @brief  根据指定的参数初始化ADC的外设在 ADC_CommonInitStruct中. 
  *         
  * @param  ADC_CommonInitStruct: 是一个ADC_CommonInitTypeDef 结构体指针 
  *         包含所有ADC外设的配置信息。
  * @retval None
  */
void ADC_CommonInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct)
{
  uint32_t tmpreg1 = 0;
  /* Check the parameters */
  assert_param(IS_ADC_MODE(ADC_CommonInitStruct->ADC_Mode));
  assert_param(IS_ADC_PRESCALER(ADC_CommonInitStruct->ADC_Prescaler));
  assert_param(IS_ADC_DMA_ACCESS_MODE(ADC_CommonInitStruct->ADC_DMAAccessMode));
  assert_param(IS_ADC_SAMPLING_DELAY(ADC_CommonInitStruct->ADC_TwoSamplingDelay));
  /*---------------------------- ADC CCR Configuration -----------------*/
  /* Get the ADC CCR value */
  tmpreg1 = ADC->CCR;
  
  /* Clear MULTI, DELAY, DMA and ADCPRE bits */
  tmpreg1 &= CR_CLEAR_MASK;
  
  /* Configure ADCx: Multi mode, Delay between two sampling time, ADC prescaler,
     and DMA access mode for multimode */
  /* Set MULTI bits according to ADC_Mode value */
  /* Set ADCPRE bits according to ADC_Prescaler value */
  /* Set DMA bits according to ADC_DMAAccessMode value */
  /* Set DELAY bits according to ADC_TwoSamplingDelay value */    
  tmpreg1 |= (uint32_t)(ADC_CommonInitStruct->ADC_Mode | 
                        ADC_CommonInitStruct->ADC_Prescaler | 
                        ADC_CommonInitStruct->ADC_DMAAccessMode | 
                        ADC_CommonInitStruct->ADC_TwoSamplingDelay);
                        
  /* Write to ADC CCR */
  ADC->CCR = tmpreg1;
}

/**
  * @brief  Fills each ADC_CommonInitStruct member with its default value.
  * @param  ADC_CommonInitStruct: pointer to an ADC_CommonInitTypeDef structure
  *         which will be initialized.
  * @retval None
  */
void ADC_CommonStructInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct)
{
  /* Initialize the ADC_Mode member */
  ADC_CommonInitStruct->ADC_Mode = ADC_Mode_Independent;

  /* initialize the ADC_Prescaler member */
  ADC_CommonInitStruct->ADC_Prescaler = ADC_Prescaler_Div2;

  /* Initialize the ADC_DMAAccessMode member */
  ADC_CommonInitStruct->ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;

  /* Initialize the ADC_TwoSamplingDelay member */
  ADC_CommonInitStruct->ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
}

/**
  * @brief  启用或禁用指定的ADC外设。
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  NewState: new state of the ADCx peripheral. 
  *         这个参数可以是 ENABLE 或 DISABLE .
  * @retval None
  */
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Set the ADON bit to wake up the ADC from power down mode */
    ADCx->CR2 |= (uint32_t)ADC_CR2_ADON;
  }
  else
  {
    /* Disable the selected ADC peripheral */
    ADCx->CR2 &= (uint32_t)(~ADC_CR2_ADON);
  }
}
/**
  * @}
  */

/** @defgroup ADC_Group2模拟看门狗配置功能
 *  @brief    模拟看门狗配置功能
 *
@verbatim   
 ===============================================================================
                    模拟看门狗配置功能
 ===============================================================================  

本节提供了配置模拟看门狗功能（AWD）的ADC功能。


  
  一个典型的配置按照以下步骤进行模拟看门狗:
   1. 看守的ADC通道（s）是（are）的选择 使用的是
      ADC_AnalogWatchdogSingleChannelConfig() 函数.
   2. 模拟看门狗低门槛较高的配置使用的是 
     ADC_AnalogWatchdogThresholdsConfig() 函数.
   3. 模拟看门狗启用和配置，使检查，在一个

      或多个通道中，使用的是 ADC_AnalogWatchdogCmd（）函数

@endverbatim
  * @{
  */
  
/**
  * @brief  启用或禁用 单/所有 常规的模拟看门狗或注入通道 
  *         
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  ADC_AnalogWatchdog: ADC模拟看门狗配置。
  *         这个参数可以取下列值之一：
  *            @arg ADC_AnalogWatchdog_SingleRegEnable: Analog watchdog on a single regular channel
  *            @arg ADC_AnalogWatchdog_SingleInjecEnable: Analog watchdog on a single injected channel
  *            @arg ADC_AnalogWatchdog_SingleRegOrInjecEnable: Analog watchdog on a single regular or injected channel
  *            @arg ADC_AnalogWatchdog_AllRegEnable: Analog watchdog on all regular channel
  *            @arg ADC_AnalogWatchdog_AllInjecEnable: Analog watchdog on all injected channel
  *            @arg ADC_AnalogWatchdog_AllRegAllInjecEnable: Analog watchdog on all regular and injected channels
  *            @arg ADC_AnalogWatchdog_None: No channel guarded by the analog watchdog
  * @retval None	  
  */
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog)
{
  uint32_t tmpreg = 0;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_ANALOG_WATCHDOG(ADC_AnalogWatchdog));
  
  /* Get the old register value */
  tmpreg = ADCx->CR1;
  
  /* Clear AWDEN, JAWDEN and AWDSGL bits */
  tmpreg &= CR1_AWDMode_RESET;
  
  /* Set the analog watchdog enable mode */
  tmpreg |= ADC_AnalogWatchdog;
  
  /* Store the new register value */
  ADCx->CR1 = tmpreg;
}

/**
  * @brief  配置高或低阈值的模拟看门狗。
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  HighThreshold: ADC模拟看门狗的高阈值。
  *          这个参数必须是一个12位的值。
  * @param  LowThreshold:  ADC模拟看门狗低阈值。
  *          这个参数必须是一个12位的值。
  * @retval None
  */
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold,
                                        uint16_t LowThreshold)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_THRESHOLD(HighThreshold));
  assert_param(IS_ADC_THRESHOLD(LowThreshold));
  
  /* Set the ADCx high threshold */
  ADCx->HTR = HighThreshold;
  
  /* Set the ADCx low threshold */
  ADCx->LTR = LowThreshold;
}

/**
  * @brief  配置模拟看门狗保护单一通道
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  ADC_Channel: ADC通道配置为模拟看门狗。
  *         这个参数可以取下列值之一：
  *            @arg ADC_Channel_0: ADC Channel0 selected
  *            @arg ADC_Channel_1: ADC Channel1 selected
  *            @arg ADC_Channel_2: ADC Channel2 selected
  *            @arg ADC_Channel_3: ADC Channel3 selected
  *            @arg ADC_Channel_4: ADC Channel4 selected
  *            @arg ADC_Channel_5: ADC Channel5 selected
  *            @arg ADC_Channel_6: ADC Channel6 selected
  *            @arg ADC_Channel_7: ADC Channel7 selected
  *            @arg ADC_Channel_8: ADC Channel8 selected
  *            @arg ADC_Channel_9: ADC Channel9 selected
  *            @arg ADC_Channel_10: ADC Channel10 selected
  *            @arg ADC_Channel_11: ADC Channel11 selected
  *            @arg ADC_Channel_12: ADC Channel12 selected
  *            @arg ADC_Channel_13: ADC Channel13 selected
  *            @arg ADC_Channel_14: ADC Channel14 selected
  *            @arg ADC_Channel_15: ADC Channel15 selected
  *            @arg ADC_Channel_16: ADC Channel16 selected
  *            @arg ADC_Channel_17: ADC Channel17 selected
  *            @arg ADC_Channel_18: ADC Channel18 selected
  * @retval None
  */
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel)
{
  uint32_t tmpreg = 0;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_CHANNEL(ADC_Channel));
  
  /* Get the old register value */
  tmpreg = ADCx->CR1;
  
  /* Clear the Analog watchdog channel select bits */
  tmpreg &= CR1_AWDCH_RESET;
  
  /* Set the Analog watchdog channel */
  tmpreg |= ADC_Channel;
  
  /* Store the new register value */
  ADCx->CR1 = tmpreg;
}
/**
  * @}
  */

/** @defgroup ADC_Group3温度传感器，Vrefint（内部参考电压） 
 *            电池（VBAT）管理功能
 *  @brief   温度的传感器，Vrefint和VBAT管理功能
 *
@verbatim   
 ===============================================================================
               温度的传感器，内部参考电压和电池管理功能
 ===============================================================================  

本节提供了允许启用/禁用内置的功能之间的连接ADC和温度传感器，Vrefint和Vbat的来源。


     
  一个典型的配置，得到的温度传感器和Vrefint的通道电压按照以下步骤进行:
   1.启用的内部连接温度传感器和Vrefint的来源

     	ADC通道，使用ADC_TempSensorVrefintCmd（）函数。
   2. 选择的ADC_Channel_TempSensor或ADC_Channel_Vrefint的使用

		ADC_RegularChannelConfig（）或ADC_InjectedChannelConfig（）
   3. 获取的电压值，使用ADC_GetConversionValue（）或

		ADC_GetInjectedConversionValue（）。

  A 一个典型的配置，以获取的VBAT通道电压后下列步骤操作：

 
   1. 启用内部的连接的VBAT源与ADC通道使用	ADC_VBATCmd（）函数。

 	
   2. 使用ADC_RegularChannelConfig（）或选择ADC_Channel_Vbat

 			ADC_InjectedChannelConfig（）函数
   3. 获取的电压值，使用ADC_GetConversionValue（）或

			ADC_GetInjectedConversionValue（）。
 
@endverbatim
  * @{
  */
  
  
/**
  * @brief  启用或禁用温度传感器和Vrefint的通道。
  * @param  NewState: 新的状态温度传感器和Vrefint的渠道.
  *          这个参数可以是：ENABLE或DISABLE。
  * @retval None
  */
void ADC_TempSensorVrefintCmd(FunctionalState NewState)                
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the temperature sensor and Vrefint channel*/
    ADC->CCR |= (uint32_t)ADC_CCR_TSVREFE;
  }
  else
  {
    /* Disable the temperature sensor and Vrefint channel*/
    ADC->CCR &= (uint32_t)(~ADC_CCR_TSVREFE);
  }
}

/**
  * @brief  启用或禁用VBAT（电池电压）通道。
  * @param  NewState: 新状态的VBAT通道。
  *          这个参数可以是：ENABLE或DISABLE。
  * @retval None
  */
void ADC_VBATCmd(FunctionalState NewState)                             
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the VBAT channel*/
    ADC->CCR |= (uint32_t)ADC_CCR_VBATE;
  }
  else
  {
    /* Disable the VBAT channel*/
    ADC->CCR &= (uint32_t)(~ADC_CCR_VBATE);
  }
}

/**
  * @}
  */

/** @defgroup ADC_Group4正规渠道配置功能
 *  @brief    正规渠道配置功能
 *
@verbatim   
 ===============================================================================
                  正规渠道配置功能
 ===============================================================================  

本节允许管理ADC的正规渠道，提供的功能， 它是由2个子部分: 
  
  1. 正规渠道的配置和管理功能:  
     本款提供的功能允许配置的ADC正规渠道的：   
          - 每个通道配置的排名在常规组测序
          - 配置为每个通道的采样时间
          - 选择正规渠道的转换触发
          - 选择所需的EOC事件的行为配置
          - 启动连续模式（*）
          - 启动非连续模式
	请注意，正规渠道的功能，可配置使用ADC_Init（）函数：

	 
          - 扫描模式激活
          - 连续模式激活（**） 
          - 外部触发源 
          - 外部触发边缘
          - 正规渠道组定序的转换数。
     
     @注：（*）（**）执行相同的配置，
     
  2. 获取转换数据: 本小节提供了一个重要的功能
     ADC的外围设备，因为它的电流转换后的数据返回正规渠道当读取转换值，EOC标志自动清零。


     
     @note 对于多ADC模式,在过去 ADC1, ADC2 and ADC3 定期转换 
 		   可以在相同的返回结果数据（在选定的多模）

 	  	   选用ADC_GetMultiModeConversionValue（）函数。
       
  
@endverbatim
  * @{
  */
/**
  * @brief  为选定的ADC正规渠道及其相应的配置
  *         排在音序器和采样时间。
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  ADC_Channel: ADC通道配置 
  *          这个参数可以取下列值之一：
  *            @arg ADC_Channel_0: ADC Channel0 selected
  *            @arg ADC_Channel_1: ADC Channel1 selected
  *            @arg ADC_Channel_2: ADC Channel2 selected
  *            @arg ADC_Channel_3: ADC Channel3 selected
  *            @arg ADC_Channel_4: ADC Channel4 selected
  *            @arg ADC_Channel_5: ADC Channel5 selected
  *            @arg ADC_Channel_6: ADC Channel6 selected
  *            @arg ADC_Channel_7: ADC Channel7 selected
  *            @arg ADC_Channel_8: ADC Channel8 selected
  *            @arg ADC_Channel_9: ADC Channel9 selected
  *            @arg ADC_Channel_10: ADC Channel10 selected
  *            @arg ADC_Channel_11: ADC Channel11 selected
  *            @arg ADC_Channel_12: ADC Channel12 selected
  *            @arg ADC_Channel_13: ADC Channel13 selected
  *            @arg ADC_Channel_14: ADC Channel14 selected
  *            @arg ADC_Channel_15: ADC Channel15 selected
  *            @arg ADC_Channel_16: ADC Channel16 selected
  *            @arg ADC_Channel_17: ADC Channel17 selected
  *            @arg ADC_Channel_18: ADC Channel18 selected                       
  * @param  Rank: 常规组定序的排名中。
  *          此参数必须在1到16之间。
  * @param  ADC_SampleTime: 采样时间的值被设定为选定的通道。 
  *          这个参数可以取下列值之一:
  *            @arg ADC_SampleTime_3Cycles: Sample time equal to 3 cycles
  *            @arg ADC_SampleTime_15Cycles: Sample time equal to 15 cycles
  *            @arg ADC_SampleTime_28Cycles: Sample time equal to 28 cycles
  *            @arg ADC_SampleTime_56Cycles: Sample time equal to 56 cycles	
  *            @arg ADC_SampleTime_84Cycles: Sample time equal to 84 cycles	
  *            @arg ADC_SampleTime_112Cycles: Sample time equal to 112 cycles	
  *            @arg ADC_SampleTime_144Cycles: Sample time equal to 144 cycles	
  *            @arg ADC_SampleTime_480Cycles: Sample time equal to 480 cycles	
  * @retval None
  */
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime)
{
  uint32_t tmpreg1 = 0, tmpreg2 = 0;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_CHANNEL(ADC_Channel));
  assert_param(IS_ADC_REGULAR_RANK(Rank));
  assert_param(IS_ADC_SAMPLE_TIME(ADC_SampleTime));
  
  /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
  if (ADC_Channel > ADC_Channel_9)
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR1;
    
    /* Calculate the mask to clear */
    tmpreg2 = SMPR1_SMP_SET << (3 * (ADC_Channel - 10));
    
    /* Clear the old sample time */
    tmpreg1 &= ~tmpreg2;
    
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)ADC_SampleTime << (3 * (ADC_Channel - 10));
    
    /* Set the new sample time */
    tmpreg1 |= tmpreg2;
    
    /* Store the new register value */
    ADCx->SMPR1 = tmpreg1;
  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR2;
    
    /* Calculate the mask to clear */
    tmpreg2 = SMPR2_SMP_SET << (3 * ADC_Channel);
    
    /* Clear the old sample time */
    tmpreg1 &= ~tmpreg2;
    
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)ADC_SampleTime << (3 * ADC_Channel);
    
    /* Set the new sample time */
    tmpreg1 |= tmpreg2;
    
    /* Store the new register value */
    ADCx->SMPR2 = tmpreg1;
  }
  /* For Rank 1 to 6 */
  if (Rank < 7)
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SQR3;
    
    /* Calculate the mask to clear */
    tmpreg2 = SQR3_SQ_SET << (5 * (Rank - 1));
    
    /* Clear the old SQx bits for the selected rank */
    tmpreg1 &= ~tmpreg2;
    
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)ADC_Channel << (5 * (Rank - 1));
    
    /* Set the SQx bits for the selected rank */
    tmpreg1 |= tmpreg2;
    
    /* Store the new register value */
    ADCx->SQR3 = tmpreg1;
  }
  /* For Rank 7 to 12 */
  else if (Rank < 13)
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SQR2;
    
    /* Calculate the mask to clear */
    tmpreg2 = SQR2_SQ_SET << (5 * (Rank - 7));
    
    /* Clear the old SQx bits for the selected rank */
    tmpreg1 &= ~tmpreg2;
    
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)ADC_Channel << (5 * (Rank - 7));
    
    /* Set the SQx bits for the selected rank */
    tmpreg1 |= tmpreg2;
    
    /* Store the new register value */
    ADCx->SQR2 = tmpreg1;
  }
  /* For Rank 13 to 16 */
  else
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SQR1;
    
    /* Calculate the mask to clear */
    tmpreg2 = SQR1_SQ_SET << (5 * (Rank - 13));
    
    /* Clear the old SQx bits for the selected rank */
    tmpreg1 &= ~tmpreg2;
    
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)ADC_Channel << (5 * (Rank - 13));
    
    /* Set the SQx bits for the selected rank */
    tmpreg1 |= tmpreg2;
    
    /* Store the new register value */
    ADCx->SQR1 = tmpreg1;
  }
}

/**
  * @brief  Enables the selected ADC software start conversion of the regular channels.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval None
  */
void ADC_SoftwareStartConv(ADC_TypeDef* ADCx)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  
  /* Enable the selected ADC conversion for regular group */
  ADCx->CR2 |= (uint32_t)ADC_CR2_SWSTART;
}

/**
  * @brief  Gets the selected ADC Software start regular conversion Status.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval The new state of ADC software start conversion (SET or RESET).
  */
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  
  /* Check the status of SWSTART bit */
  if ((ADCx->CR2 & ADC_CR2_JSWSTART) != (uint32_t)RESET)
  {
    /* SWSTART bit is set */
    bitstatus = SET;
  }
  else
  {
    /* SWSTART bit is reset */
    bitstatus = RESET;
  }
  
  /* Return the SWSTART bit status */
  return  bitstatus;
}


/**
  * @brief 启用和禁用  EOC 在每个正规渠道转换
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  NewState: 选定的ADC的新状态 EOC 标志 rising
  *          这个参数可以是：ENABLE或DISABLE。
  * @retval None
  */
void ADC_EOCOnEachRegularChannelCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected ADC EOC rising on each regular channel conversion */
    ADCx->CR2 |= (uint32_t)ADC_CR2_EOCS;
  }
  else
  {
    /* Disable the selected ADC EOC rising on each regular channel conversion */
    ADCx->CR2 &= (uint32_t)(~ADC_CR2_EOCS);
  }
}

/**
  * @brief  启用或禁用ADC连续转换模式
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  NewState: 新选定的状态 ADC 连续转换模式。
  *         这个参数可以是：ENABLE或DISABLE。
  * @retval None
  */
void ADC_ContinuousModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected ADC continuous conversion mode */
    ADCx->CR2 |= (uint32_t)ADC_CR2_CONT;
  }
  else
  {
    /* Disable the selected ADC continuous conversion mode */
    ADCx->CR2 &= (uint32_t)(~ADC_CR2_CONT);
  }
}

/**
  * @brief  配置选定的ADC常规组的不连续模式通道。 
  *        
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  Number: 指定的非连续模式正规渠道的计数值。
  *          此数字必须在1和8之间。
  * @retval None
  */
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number)
{
  uint32_t tmpreg1 = 0;
  uint32_t tmpreg2 = 0;
  
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_REGULAR_DISC_NUMBER(Number));
  
  /* Get the old register value */
  tmpreg1 = ADCx->CR1;
  
  /* Clear the old discontinuous mode channel count */
  tmpreg1 &= CR1_DISCNUM_RESET;
  
  /* Set the discontinuous mode channel count */
  tmpreg2 = Number - 1;
  tmpreg1 |= tmpreg2 << 13;
  
  /* Store the new register value */
  ADCx->CR1 = tmpreg1;
}

/**
  * @brief  启用或禁用常规组信道的非连续模式为指定的ADC
  *        
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  NewState: 新选定的状态 ADC 非连续模式常规组信道。
  *         
  *          这个参数可以是：ENABLE或DISABLE。
  * @retval None
  */
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected ADC regular discontinuous mode */
    ADCx->CR1 |= (uint32_t)ADC_CR1_DISCEN;
  }
  else
  {
    /* Disable the selected ADC regular discontinuous mode */
    ADCx->CR1 &= (uint32_t)(~ADC_CR1_DISCEN);
  }
}

/**
  * @brief  返回的最后一个正规渠道的使用ADCx转换的结果数据。
  * @param  ADCx:其中x可以是1，2或3选择ADC外围。
  * @retval 数据转换值。
  */
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  
  /* Return the selected ADC conversion value */
  return (uint16_t) ADCx->DR;
}

/**
  * @brief  返回过去ADC1，ADC2和ADC3定期的转换结果在选定的多模式数据。
  *         
  * @param  None  
  * @retval 数据转换值。
  * @note   在双模式，这个函数返回的值如下：
  *           Data[15:0] : these bits contain the regular data of ADC1.
  *           Data[31:16]: these bits contain the regular data of ADC2.
  * @note   在三重模式，这个函数返回的值如下：
  *           Data[15:0] : these bits contain alternatively the regular data of ADC1, ADC3 and ADC2.
  *           Data[31:16]: these bits contain alternatively the regular data of ADC2, ADC1 and ADC3.           
  */
uint32_t ADC_GetMultiModeConversionValue(void)
{
  /* Return the multi mode conversion value */
  return (*(__IO uint32_t *) CDR_ADDRESS);
}
/**
  * @}
  */

/** @defgroup ADC_Group5正规渠道DMA配置功能
 *  @brief    正规渠道DMA配置功能
 *
@verbatim   
 ===============================================================================
                   正规通道DMA配置功能
 ===============================================================================  

 本节提供的功能，可以配置为ADC的DMA定期通道。


 由于转换的正规通道的值存储成一个独特的数据寄存器，它是有用的使用的DMA多于一个常规信道的转换。

这避免了已经存储在ADC数据寄存器中的数据丢失。
  
当DMA模式被使能（使用的ADC_DMACmd（）函数），在每个的正规通道的转换后，产生一个DMA请求。


  
根据“DMA禁止选择独立的ADC模式”
配置（使用ADC_DMARequestAfterLastTransferCmd（）函数），
在最后的DMA传输的结束，两种可能性是允许的：
  - 没有新的DMA请求发出的DMA控制器（功能关闭）
  - 可以继续请求的产生（功能启用）。
  
根据“DMA禁止选择多ADC模式”配置
（使用的空隙ADC_MultiModeDMARequestAfterLastTransferCmd（）函数），
在最后的DMA传输的结束，两种可能性是允许的：
  - 没有新的DMA请求发出的DMA控制器（功能关闭）
  - 可以继续请求的产生（功能启用）。

@endverbatim
  * @{
  */
  
 /**
  * @brief  启用或禁用指定的ADC DMA请求。
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  NewState: 新选定的ADC DMA传输的状态。
  *          这个参数可以是：ENABLE或DISABLE。
  * @retval None
  */
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected ADC DMA request */
    ADCx->CR2 |= (uint32_t)ADC_CR2_DMA;
  }
  else
  {
    /* Disable the selected ADC DMA request */
    ADCx->CR2 &= (uint32_t)(~ADC_CR2_DMA);
  }
}

/**
  * @brief 启用或禁用ADC DMA请求后，最后转移（单ADC模式）
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  NewState: 新选定的ADC DMA请求后，最后一次传输的状态。
  *         这个参数可以是：ENABLE或DISABLE。
  * @retval None
  */
void ADC_DMARequestAfterLastTransferCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected ADC DMA request after last transfer */
    ADCx->CR2 |= (uint32_t)ADC_CR2_DDS;
  }
  else
  {
    /* Disable the selected ADC DMA request after last transfer */
    ADCx->CR2 &= (uint32_t)(~ADC_CR2_DDS);
  }
}

/**
  * @brief  启用或禁用ADC DMA请求后，最后一次传输多ADC模式

  * @param  NewState: 新选定的ADC DMA请求后，最后一次传输的状态。

  *         这个参数可以是：ENABLE或DISABLE。
  * @note   如果启用，发出DMA请求，只要数据被转换和 
  *         DMA模式的多ADC模式（选择使用ADC_CommonInit（）函数
  *         由结构成员ADC_CommonInitStruct.ADC_DMAAccessMode）
  *          ADC_DMAAccessMode_1, ADC_DMAAccessMode_2 or ADC_DMAAccessMode_3.     
  * @retval None
  */
void ADC_MultiModeDMARequestAfterLastTransferCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected ADC DMA request after last transfer */
    ADC->CCR |= (uint32_t)ADC_CCR_DDS;
  }
  else
  {
    /* Disable the selected ADC DMA request after last transfer */
    ADC->CCR &= (uint32_t)(~ADC_CCR_DDS);
  }
}
/**
  * @}
  */

/** @defgroup ADC_Group6注入通道配置功能
 *  @brief   注入通道配置功能
 *
@verbatim   
 ===============================================================================
                     注入通道配置功能
 ===============================================================================  

本节所提供的功能允许配置的ADC注入通道，它是由2小节：
    
	1.注入通道配置功能：本款规定功能允许配置的ADC注入通道：


		- 注射组定序的排名中，每个通道配置

		- 配置每个通道的采样时间

		- 启动自动注入模式

		- 启动非连续模式

		- 扫描模式激活

		- 外部触发源/软件

		- 外部触发边缘

		- 注入通道定序器。
	2.获得指定的注入通道的转换数据：本款在ADC外设提供了一个重要的功能，	

		因为它返回转换后的数据的具体的注入通道。

@endverbatim
  * @{
  */ 
/**
  * @brief  选定的ADC注入通道的相应配置
  *         rank in the sequencer and its sample time.
  * @param  ADCx:其中x可以是1，2或3选择ADC外围。
  * @param  ADC_Channel:ADC通道配置。 
  *          这个参数可以取下列值之一：
  *            @arg ADC_Channel_0: ADC Channel0 selected
  *            @arg ADC_Channel_1: ADC Channel1 selected
  *            @arg ADC_Channel_2: ADC Channel2 selected
  *            @arg ADC_Channel_3: ADC Channel3 selected
  *            @arg ADC_Channel_4: ADC Channel4 selected
  *            @arg ADC_Channel_5: ADC Channel5 selected
  *            @arg ADC_Channel_6: ADC Channel6 selected
  *            @arg ADC_Channel_7: ADC Channel7 selected
  *            @arg ADC_Channel_8: ADC Channel8 selected
  *            @arg ADC_Channel_9: ADC Channel9 selected
  *            @arg ADC_Channel_10: ADC Channel10 selected
  *            @arg ADC_Channel_11: ADC Channel11 selected
  *            @arg ADC_Channel_12: ADC Channel12 selected
  *            @arg ADC_Channel_13: ADC Channel13 selected
  *            @arg ADC_Channel_14: ADC Channel14 selected
  *            @arg ADC_Channel_15: ADC Channel15 selected
  *            @arg ADC_Channel_16: ADC Channel16 selected
  *            @arg ADC_Channel_17: ADC Channel17 selected
  *            @arg ADC_Channel_18: ADC Channel18 selected                       
  * @param  Rank: The rank in the injected group sequencer. 
  *          This parameter must be between 1 to 4.
  * @param  ADC_SampleTime: 采样时间的值被设定为选定的通道。

  *         这个参数可以取下列值之一：
  *            @arg ADC_SampleTime_3Cycles: Sample time equal to 3 cycles
  *            @arg ADC_SampleTime_15Cycles: Sample time equal to 15 cycles
  *            @arg ADC_SampleTime_28Cycles: Sample time equal to 28 cycles
  *            @arg ADC_SampleTime_56Cycles: Sample time equal to 56 cycles	
  *            @arg ADC_SampleTime_84Cycles: Sample time equal to 84 cycles	
  *            @arg ADC_SampleTime_112Cycles: Sample time equal to 112 cycles	
  *            @arg ADC_SampleTime_144Cycles: Sample time equal to 144 cycles	
  *            @arg ADC_SampleTime_480Cycles: Sample time equal to 480 cycles	
  * @retval None
  */
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime)
{
  uint32_t tmpreg1 = 0, tmpreg2 = 0, tmpreg3 = 0;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_CHANNEL(ADC_Channel));
  assert_param(IS_ADC_INJECTED_RANK(Rank));
  assert_param(IS_ADC_SAMPLE_TIME(ADC_SampleTime));
  /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
  if (ADC_Channel > ADC_Channel_9)
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR1;
    /* Calculate the mask to clear */
    tmpreg2 = SMPR1_SMP_SET << (3*(ADC_Channel - 10));
    /* Clear the old sample time */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)ADC_SampleTime << (3*(ADC_Channel - 10));
    /* Set the new sample time */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR1 = tmpreg1;
  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR2;
    /* Calculate the mask to clear */
    tmpreg2 = SMPR2_SMP_SET << (3 * ADC_Channel);
    /* Clear the old sample time */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)ADC_SampleTime << (3 * ADC_Channel);
    /* Set the new sample time */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR2 = tmpreg1;
  }
  /* Rank configuration */
  /* Get the old register value */
  tmpreg1 = ADCx->JSQR;
  /* Get JL value: Number = JL+1 */
  tmpreg3 =  (tmpreg1 & JSQR_JL_SET)>> 20;
  /* Calculate the mask to clear: ((Rank-1)+(4-JL-1)) */
  tmpreg2 = JSQR_JSQ_SET << (5 * (uint8_t)((Rank + 3) - (tmpreg3 + 1)));
  /* Clear the old JSQx bits for the selected rank */
  tmpreg1 &= ~tmpreg2;
  /* Calculate the mask to set: ((Rank-1)+(4-JL-1)) */
  tmpreg2 = (uint32_t)ADC_Channel << (5 * (uint8_t)((Rank + 3) - (tmpreg3 + 1)));
  /* Set the JSQx bits for the selected rank */
  tmpreg1 |= tmpreg2;
  /* Store the new register value */
  ADCx->JSQR = tmpreg1;
}

/**
  * @brief  Configures the sequencer length for injected channels
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  Length: The sequencer length. 
  *          This parameter must be a number between 1 to 4.
  * @retval None
  */
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length)
{
  uint32_t tmpreg1 = 0;
  uint32_t tmpreg2 = 0;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_INJECTED_LENGTH(Length));
  
  /* Get the old register value */
  tmpreg1 = ADCx->JSQR;
  
  /* Clear the old injected sequence length JL bits */
  tmpreg1 &= JSQR_JL_RESET;
  
  /* Set the injected sequence length JL bits */
  tmpreg2 = Length - 1; 
  tmpreg1 |= tmpreg2 << 20;
  
  /* Store the new register value */
  ADCx->JSQR = tmpreg1;
}

/**
  * @brief  将注入通道的转换值的偏移量
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  ADC_InjectedChannel:ADC注入信道设置它的偏移量。
  *          This parameter can be one of the following values:
  *            @arg ADC_InjectedChannel_1: Injected Channel1 selected
  *            @arg ADC_InjectedChannel_2: Injected Channel2 selected
  *            @arg ADC_InjectedChannel_3: Injected Channel3 selected
  *            @arg ADC_InjectedChannel_4: Injected Channel4 selected
  * @param  Offset: 选定的ADC注入通道的偏移值
  *         这个参数必须是一个12位的值。
  * @retval None
  */
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset)
{
    __IO uint32_t tmp = 0;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_INJECTED_CHANNEL(ADC_InjectedChannel));
  assert_param(IS_ADC_OFFSET(Offset));
  
  tmp = (uint32_t)ADCx;
  tmp += ADC_InjectedChannel;
  
  /* Set the selected injected channel data offset */
 *(__IO uint32_t *) tmp = (uint32_t)Offset;
}

 /**
  * @brief  配置使用ADCx外部触发注入通道转换。
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  ADC_ExternalTrigInjecConv: 指定的启动注入转换的ADC触发。

  *         这个参数可以取下列值之一：                   
  *            @arg ADC_ExternalTrigInjecConv_T1_CC4: Timer1 capture compare4 selected 
  *            @arg ADC_ExternalTrigInjecConv_T1_TRGO: Timer1 TRGO event selected 
  *            @arg ADC_ExternalTrigInjecConv_T2_CC1: Timer2 capture compare1 selected 
  *            @arg ADC_ExternalTrigInjecConv_T2_TRGO: Timer2 TRGO event selected 
  *            @arg ADC_ExternalTrigInjecConv_T3_CC2: Timer3 capture compare2 selected 
  *            @arg ADC_ExternalTrigInjecConv_T3_CC4: Timer3 capture compare4 selected 
  *            @arg ADC_ExternalTrigInjecConv_T4_CC1: Timer4 capture compare1 selected                       
  *            @arg ADC_ExternalTrigInjecConv_T4_CC2: Timer4 capture compare2 selected 
  *            @arg ADC_ExternalTrigInjecConv_T4_CC3: Timer4 capture compare3 selected                        
  *            @arg ADC_ExternalTrigInjecConv_T4_TRGO: Timer4 TRGO event selected 
  *            @arg ADC_ExternalTrigInjecConv_T5_CC4: Timer5 capture compare4 selected                        
  *            @arg ADC_ExternalTrigInjecConv_T5_TRGO: Timer5 TRGO event selected                        
  *            @arg ADC_ExternalTrigInjecConv_T8_CC2: Timer8 capture compare2 selected
  *            @arg ADC_ExternalTrigInjecConv_T8_CC3: Timer8 capture compare3 selected                        
  *            @arg ADC_ExternalTrigInjecConv_T8_CC4: Timer8 capture compare4 selected 
  *            @arg ADC_ExternalTrigInjecConv_Ext_IT15: External interrupt line 15 event selected                          
  * @retval None
  */
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv)
{
  uint32_t tmpreg = 0;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_EXT_INJEC_TRIG(ADC_ExternalTrigInjecConv));
  
  /* Get the old register value */
  tmpreg = ADCx->CR2;
  
  /* Clear the old external event selection for injected group */
  tmpreg &= CR2_JEXTSEL_RESET;
  
  /* Set the external event selection for injected group */
  tmpreg |= ADC_ExternalTrigInjecConv;
  
  /* Store the new register value */
  ADCx->CR2 = tmpreg;
}

/**
  * @brief  配置使用ADCx外部触发边缘注入通道转换。

  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  ADC_ExternalTrigInjecConvEdge: 指定ADC的外部触发边缘

  *         开始注入转换。
  *         这个参数可以取下列值之一：
  *            @arg ADC_ExternalTrigInjecConvEdge_None: external trigger disabled for 
  *                                                     injected conversion
  *            @arg ADC_ExternalTrigInjecConvEdge_Rising: detection on rising edge
  *            @arg ADC_ExternalTrigInjecConvEdge_Falling: detection on falling edge
  *            @arg ADC_ExternalTrigInjecConvEdge_RisingFalling: detection on both rising 
  *                                                               and falling edge
  * @retval None
  */
void ADC_ExternalTrigInjectedConvEdgeConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConvEdge)
{
  uint32_t tmpreg = 0;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_EXT_INJEC_TRIG_EDGE(ADC_ExternalTrigInjecConvEdge));
  /* Get the old register value */
  tmpreg = ADCx->CR2;
  /* Clear the old external trigger edge for injected group */
  tmpreg &= CR2_JEXTEN_RESET;
  /* Set the new external trigger edge for injected group */
  tmpreg |= ADC_ExternalTrigInjecConvEdge;
  /* Store the new register value */
  ADCx->CR2 = tmpreg;
}

/**
  * @brief 启用所选的ADC软件开始转换的注入通道。
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @retval None
  */
void ADC_SoftwareStartInjectedConv(ADC_TypeDef* ADCx)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  /* Enable the selected ADC conversion for injected group */
  ADCx->CR2 |= (uint32_t)ADC_CR2_JSWSTART;
}

/**
  * @brief  获取选定的ADC软件开始注入转换状态。

  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @retval 新的国家开始注入转换的ADC软件（SET或RESET）。

  */
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  
  /* Check the status of JSWSTART bit */
  if ((ADCx->CR2 & ADC_CR2_JSWSTART) != (uint32_t)RESET)
  {
    /* JSWSTART bit is set */
    bitstatus = SET;
  }
  else
  {
    /* JSWSTART bit is reset */
    bitstatus = RESET;
  }
  /* Return the JSWSTART bit status */
  return  bitstatus;
}

/**
  * @brief  启用或禁用选定的ADC自动注射组转换后定期。
  *        
  * @param  ADCx:其中x可以是1，2或3选择ADC外围。
  * @param  NewState:选定的ADC自动注入转换的新状态
  *         这个参数可以是：ENABLE或DISABLE。
  * @retval None
  */
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected ADC automatic injected group conversion */
    ADCx->CR1 |= (uint32_t)ADC_CR1_JAUTO;
  }
  else
  {
    /* Disable the selected ADC automatic injected group conversion */
    ADCx->CR1 &= (uint32_t)(~ADC_CR1_JAUTO);
  }
}

/**
  * @brief  启用或禁用注射组的不连续模式指定的ADC通道
  *        
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  NewState: ADC选定的非连续模式注入新的状态通道组。
  *         
  *         这个参数可以是：ENABLE或DISABLE。
  * @retval None
  */
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected ADC injected discontinuous mode */
    ADCx->CR1 |= (uint32_t)ADC_CR1_JDISCEN;
  }
  else
  {
    /* Disable the selected ADC injected discontinuous mode */
    ADCx->CR1 &= (uint32_t)(~ADC_CR1_JDISCEN);
  }
}

/**
  * @brief  返回ADC注入信道转换结果
  * @param  ADCx:其中x可以是1，2或3选择ADC外围。
  * @param  ADC_InjectedChannel: 转换的ADC注入信道。
  *         这个参数可以取下列值之一：
  *            @arg ADC_InjectedChannel_1: Injected Channel1 selected
  *            @arg ADC_InjectedChannel_2: Injected Channel2 selected
  *            @arg ADC_InjectedChannel_3: Injected Channel3 selected
  *            @arg ADC_InjectedChannel_4: Injected Channel4 selected
  * @retval The Data conversion value.
  */
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel)
{
  __IO uint32_t tmp = 0;
  
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_INJECTED_CHANNEL(ADC_InjectedChannel));

  tmp = (uint32_t)ADCx;
  tmp += ADC_InjectedChannel + JDR_OFFSET;
  
  /* Returns the selected injected channel conversion data value */
  return (uint16_t) (*(__IO uint32_t*)  tmp); 
}
/**
  * @}
  */

/** @defgroup ADC_Group7中断和标志管理功能
 *  @brief    中断和标志管理功能
 *
@verbatim   
 ===============================================================================
                   中断和标志管理功能
 ===============================================================================  

本节提供的功能可配置的ADC中断， 得到的地位和明确的标志和中断标志位。


  
每个ADC提供了4个中断源和6标志可分为：
  3 groups:
  
  I. 标志和中断ADC正规渠道
  =================================================
  Flags :
  ---------- 
     1. ADC_FLAG_OVR : 超限检测，定期转换后的数据丢失

     2. ADC_FLAG_EOC :常规通道转换结束==> 来表示 (根据EOCS位，管理 
          ADC_EOCOnEachRegularChannelCmd() )结束：
               ==>一个正规渠道转换
               ==>常规组转换序列。

     3. ADC_FLAG_STRT: 正规渠道启动 ==>正规渠道开始转换。




  Interrupts :
  ------------
     1. ADC_IT_OVR : 指定中断源超限检测事件。 
     2. ADC_IT_EOC : 指定正规渠道端的中断源转换事件。 
                     
  
  
  II.标志和中断ADC注入通道
  =================================================
  Flags :
  ---------- 
     1. ADC_FLAG_JEOC : 注入通道转换结束 ==> to indicate at 
               the end of injected GROUP conversion  
              
     2. ADC_FLAG_JSTRT: 注入通道启动 ==> 指示硬件时注射组转换开始.
              

  Interrupts :
  ------------
     1. ADC_IT_JEOC： 指定注入的通道尽头的中断源转换事件。
                     

  III. 一般标志和中断的ADC
  ================================================= 
  Flags :
  ---------- 
     1. ADC_FLAG_AWD:模拟看门狗==> to indicate 如果转换的电压越过设定的阈值。
             
              
  Interrupts :
  ------------
     1. ADC_IT_AWD : 指定中断源的模拟看门狗事件。

  
 用户应确定哪种模式将被用于在自己的应用程序管理ADC控制器的事件：查询模式或中断模式。

  
在查询模式下，建议使用以下功能：
      - ADC_GetFlagStatus() : 检查标志的事件发生。
      - ADC_ClearFlag()     : 清除的标志事件。
      
在中断模式下，建议使用以下功能：
     - ADC_ITConfig()          : 启用或禁用中断源。
     - ADC_GetITStatus()       : 来检查，如果发生中断。
     - ADC_ClearITPendingBit() : 清除的中断挂起位（相应的标志）。
                               
@endverbatim
  * @{
  */ 
/**
  * @brief  启用或禁用指定的ADC中断。
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  ADC_IT: 指定要启用或禁用的ADC中断源。
  *         这个参数可以取下列值之一：
  *            @arg ADC_IT_EOC: End of conversion interrupt mask
  *            @arg ADC_IT_AWD: Analog watchdog interrupt mask
  *            @arg ADC_IT_JEOC: End of injected conversion interrupt mask
  *            @arg ADC_IT_OVR: Overrun interrupt enable                       
  * @param  NewState: 指定的ADC中断的新状态。
  *         这个参数可以是：ENABLE或DISABLE。
  * @retval None
  */
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState)  
{
  uint32_t itmask = 0;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_ADC_IT(ADC_IT)); 

  /* Get the ADC IT index */
  itmask = (uint8_t)ADC_IT;
  itmask = (uint32_t)0x01 << itmask;    

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC interrupts */
    ADCx->CR1 |= itmask;
  }
  else
  {
    /* Disable the selected ADC interrupts */
    ADCx->CR1 &= (~(uint32_t)itmask);
  }
}

/**
  * @brief  检查是否设置或指定的ADC标志。
  * @param  ADCx: 其中x可以是1，2或3选择ADC外围。
  * @param  ADC_FLAG: 指定要检查的标志。
  *         这个参数可以取下列值之一：
  *            @arg ADC_FLAG_AWD: Analog watchdog flag
  *            @arg ADC_FLAG_EOC: End of conversion flag
  *            @arg ADC_FLAG_JEOC: End of injected group conversion flag
  *            @arg ADC_FLAG_JSTRT: Start of injected group conversion flag
  *            @arg ADC_FLAG_STRT: Start of regular group conversion flag
  *            @arg ADC_FLAG_OVR: Overrun flag                                                 
  * @retval ADC_FLAG的新状态（SET或RESET）。
  */
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_GET_FLAG(ADC_FLAG));

  /* Check the status of the specified ADC flag */
  if ((ADCx->SR & ADC_FLAG) != (uint8_t)RESET)
  {
    /* ADC_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* ADC_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the ADC_FLAG status */
  return  bitstatus;
}

/**
  * @brief  清除的使用ADCx的挂起标志的。
  * @param  ADCx:其中x可以是1，2或3选择ADC外围。
  * @param  ADC_FLAG: 指定清除的标志。
  *         这个参数可以是下面的值的任何组合：
  *            @arg ADC_FLAG_AWD: Analog watchdog flag
  *            @arg ADC_FLAG_EOC: End of conversion flag
  *            @arg ADC_FLAG_JEOC: End of injected group conversion flag
  *            @arg ADC_FLAG_JSTRT: Start of injected group conversion flag
  *            @arg ADC_FLAG_STRT: Start of regular group conversion flag
  *            @arg ADC_FLAG_OVR: Overrun flag                          
  * @retval None
  */
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_CLEAR_FLAG(ADC_FLAG));

  /* Clear the selected ADC flags */
  ADCx->SR = ~(uint32_t)ADC_FLAG;
}

/**
  * @brief 检查指定的ADC中断发生与否。
  * @param  ADCx:其中x可以是1，2或3选择ADC外围。
  * @param  ADC_IT:指定的ADC中断源进行检查。
  *         这个参数可以取下列值之一：
  *            @arg ADC_IT_EOC: End of conversion interrupt mask
  *            @arg ADC_IT_AWD: Analog watchdog interrupt mask
  *            @arg ADC_IT_JEOC: End of injected conversion interrupt mask
  *            @arg ADC_IT_OVR: Overrun interrupt mask                        
  * @retval ADC_IT的新状态（SET或RESET）。
  */
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT)
{
  ITStatus bitstatus = RESET;
  uint32_t itmask = 0, enablestatus = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_IT(ADC_IT));

  /* Get the ADC IT index */
  itmask = ADC_IT >> 8;

  /* Get the ADC_IT enable bit status */
  enablestatus = (ADCx->CR1 & ((uint32_t)0x01 << (uint8_t)ADC_IT)) ;

  /* Check the status of the specified ADC interrupt */
  if (((ADCx->SR & itmask) != (uint32_t)RESET) && enablestatus)
  {
    /* ADC_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* ADC_IT is reset */
    bitstatus = RESET;
  }
  /* Return the ADC_IT status */
  return  bitstatus;
}

/**
  * @brief  清除的使用ADCx的中断标志位。
  * @param  ADCx:其中x可以是1，2或3选择ADC外围。
  * @param  ADC_IT:指定ADC中断标志位来清除。
  *         这个参数可以取下列值之一：
  *            @arg ADC_IT_EOC: End of conversion interrupt mask
  *            @arg ADC_IT_AWD: Analog watchdog interrupt mask
  *            @arg ADC_IT_JEOC: End of injected conversion interrupt mask
  *            @arg ADC_IT_OVR: Overrun interrupt mask                         
  * @retval None
  */
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT)
{
  uint8_t itmask = 0;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_IT(ADC_IT)); 
  /* Get the ADC IT index */
  itmask = (uint8_t)(ADC_IT >> 8);
  /* Clear the selected ADC interrupt pending bits */
  ADCx->SR = ~(uint32_t)itmask;
}                    
/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
