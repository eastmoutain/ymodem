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
  *                                   ���ʹ���������
  *          ===================================================================

  *          1.  ʹ�ܣ��ģýӿڵ�ʱ�� 
  *                  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADCx, ENABLE); 
  *     
  *          2. ADC��������
  *               ����ʹ�����¹��ܵ�ADC��ʱ�ӣ�
  *                 �� RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);   
  *               �� ��ʹ��gpio_init()����������ЩADC���ţ�
  *
  *          3. ����ADC��Ƶ����ת���ķֱ��ʺ�����
  *              ʹ��  ADC_Init() ������
  *          4. ʹ�� ADC_Cmd() �����������ģ�����.
  *
  *          ����ADCͨ��������
  *          ====================================    
  *            - Ҫ����ADC����ͨ���鹦�ܣ�ʹ��
  *              ADC_Init()��ʼ�� �� ADC_RegularChannelConfig() ����.
  *            - Ҫʹ������ģʽ,��ʹ�� ADC_continuousModeCmd()������
  *     
  *            - Ҫ���ú�ʹ�÷�����ģʽ, ��ʹ��
  *              ADC_DiscModeChannelCountConfig() �� ADC_DiscModeCmd() ����.
  *            - Ҫ��ȡADCת�����ֵ,��ʹ��ADC_GetConversionValue()������
  *             
  *
  *          ��ģʽADC��������������
  *          ===============================================
  *            - ����� "����ADCͨ��������" ˵��
  *              ���õ�ADC1��ADC2��ADC3���ڵ�ͨ����      
  *            - ѡ���ģʽADC����ͨ�����ܣ�˫�ػ�����ģʽ��
  *              ʹ��ADC_CommonInit��������������
  *              DMA ģʽʹ�� ADC_MultiModeDMARequestAfterLastTransferCmd()���� 
  *                  
  *            - ADC��ת��ֵ��ʹ��	ADC_GetMultiModeConversionValue() ����
  *              
  *
  *          ����ͨ�����DMA��������
  *          ====================================================== 
  *           - Ҫ����DMAģʽΪ����������, ��ʹ��  ADC_DMACmd() ������
  *             
  *           - Ϊʹ DMA ����������DMA����
  *             ��ʹ�� ADC_DMARequestAfterLastTransferCmd() ����
  *           
  *
  *          ע��ͨ��������
  *          =====================================    
  *            - Ҫ����ADCע��ͨ���鹦��, ��ʹ��
  *              ADC_InjectedChannelConfig() ��  ADC_InjectedSequencerLengthConfig()
  *              ������
  *            - Ҫʹ������ģʽ,��ʹ�� ADC_continuousModeCmd() ����
  *              
  *            - Ҫʹ��ע��ķ�����ģʽ, ��ʹ��
  *               ADC_InjectedDiscModeCmd() ����
  *            - Ҫʹ�õ�AutoInjectedģʽ, ��ʹ�� ADC_AutoInjectedConvCmd()  ����
  *              
  *            - Ҫ��ȡADCת�����ֵ, ��ʹ�� ADC_GetInjectedConversionValue()���� 
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
                      ��ʼ�������ù���
 ===============================================================================  
  ���ڹ涨����Ĺ��ܣ�
   - Initialize and configure the ADC Prescaler   	��ʼ��������ADCԤ��Ƶ��
   - ADC Conversion Resolution (12bit..6bit)		 ADCת������(12bit..6bit)
   - Scan Conversion Mode (multichannels or one channel) for regular group  ɨ��ת��ģʽ(��ͨ����һ��ͨ��)������
   - ADC Continuous Conversion Mode (Continuous or Single conversion) for 	ADC����ת��ģʽ(�����򵥴�ת��)������

     regular group
   - External trigger Edge and source of regular group, �ⲿ������Ե��Դ�ĳ�����
   - Converted data alignment (left or right)			ת��������ݶ��루����ң�
   - The number of ADC conversions that will be done using the sequencer for    ADCת����ʹ��������������,����������

     regular channel group
   - Multi ADC mode selection	 ��ADCģʽѡ��
   - Direct memory access mode selection for multi ADC mode   ��ADCģʽ��ֱ�Ӽ������ȡģʽѡ��
   - Delay between 2 sampling phases (used in dual or triple interleaved modes)	2��������λ֮����ӳ٣�����˫�ػ����ؽ���ģʽ��
   - Enable or disable the ADC peripheral	  ���û����ADC����
   
@endverbatim
  * @{
  */

/**
  * @���� :  ��ADCs ������ļĴ�����λĬ��ֵ��
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
  * @brief  ����ָ���Ĳ�����ʼ��ʹ��ADCx���� 
  *         ��ADC_InitStruct�ṹ���С�
  * @note   �����������������ȫ�ֹ��ܵ� ADC (�ֱ��ʺ�����)
  *         ����, ��������õľ���Ĳ����μ�����ͨ����  
  *         (ɨ��ģʽ����, ����ģʽ�¼���, �ⲿ����Դ�ͱ�Ե��
  *          �����������鶨����ת��).  
  * @param  ADCx: ����x������1��2��3��ѡ��ADC��Χ�豸��
  * @param  ADC_InitStruct: ��һ��ADC_InitTypeDef�Ľṹ��ָ�룬������
  *         ָ����ADC�����������Ϣ.
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
  * @brief  ���ÿ�� ADC_InitStruct Ĭ�ϵĳ�Աֵ.
  * @note   �˹������ڳ�ʼ����ADC��ȫ������ADC ( Resolution and Data Alignment) 
  *        	Ȼ������������õĲ�����μ�����ͨ����������
  *         
  *         (ɨ��ģʽ����, ����ģʽ�¼���, �ⲿ����Դ�ͱ��������������鶨����ת��). 
  *           
  * @param  ADC_InitStruct: ��һ��ADC_InitTypeDef�Ľṹָ�뱻��ʼ����
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
  * @brief  ����ָ���Ĳ�����ʼ��ADC�������� ADC_CommonInitStruct��. 
  *         
  * @param  ADC_CommonInitStruct: ��һ��ADC_CommonInitTypeDef �ṹ��ָ�� 
  *         ��������ADC�����������Ϣ��
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
  * @brief  ���û����ָ����ADC���衣
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  NewState: new state of the ADCx peripheral. 
  *         ������������� ENABLE �� DISABLE .
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

/** @defgroup ADC_Group2ģ�⿴�Ź����ù���
 *  @brief    ģ�⿴�Ź����ù���
 *
@verbatim   
 ===============================================================================
                    ģ�⿴�Ź����ù���
 ===============================================================================  

�����ṩ������ģ�⿴�Ź����ܣ�AWD����ADC���ܡ�


  
  һ�����͵����ð������²������ģ�⿴�Ź�:
   1. ���ص�ADCͨ����s���ǣ�are����ѡ�� ʹ�õ���
      ADC_AnalogWatchdogSingleChannelConfig() ����.
   2. ģ�⿴�Ź����ż��ϸߵ�����ʹ�õ��� 
     ADC_AnalogWatchdogThresholdsConfig() ����.
   3. ģ�⿴�Ź����ú����ã�ʹ��飬��һ��

      ����ͨ���У�ʹ�õ��� ADC_AnalogWatchdogCmd��������

@endverbatim
  * @{
  */
  
/**
  * @brief  ���û���� ��/���� �����ģ�⿴�Ź���ע��ͨ�� 
  *         
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_AnalogWatchdog: ADCģ�⿴�Ź����á�
  *         �����������ȡ����ֵ֮һ��
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
  * @brief  ���ø߻����ֵ��ģ�⿴�Ź���
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  HighThreshold: ADCģ�⿴�Ź��ĸ���ֵ��
  *          �������������һ��12λ��ֵ��
  * @param  LowThreshold:  ADCģ�⿴�Ź�����ֵ��
  *          �������������һ��12λ��ֵ��
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
  * @brief  ����ģ�⿴�Ź�������һͨ��
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_Channel: ADCͨ������Ϊģ�⿴�Ź���
  *         �����������ȡ����ֵ֮һ��
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

/** @defgroup ADC_Group3�¶ȴ�������Vrefint���ڲ��ο���ѹ�� 
 *            ��أ�VBAT��������
 *  @brief   �¶ȵĴ�������Vrefint��VBAT������
 *
@verbatim   
 ===============================================================================
               �¶ȵĴ��������ڲ��ο���ѹ�͵�ع�����
 ===============================================================================  

�����ṩ����������/�������õĹ���֮�������ADC���¶ȴ�������Vrefint��Vbat����Դ��


     
  һ�����͵����ã��õ����¶ȴ�������Vrefint��ͨ����ѹ�������²������:
   1.���õ��ڲ������¶ȴ�������Vrefint����Դ

     	ADCͨ����ʹ��ADC_TempSensorVrefintCmd����������
   2. ѡ���ADC_Channel_TempSensor��ADC_Channel_Vrefint��ʹ��

		ADC_RegularChannelConfig������ADC_InjectedChannelConfig����
   3. ��ȡ�ĵ�ѹֵ��ʹ��ADC_GetConversionValue������

		ADC_GetInjectedConversionValue������

  A һ�����͵����ã��Ի�ȡ��VBATͨ����ѹ�����в��������

 
   1. �����ڲ������ӵ�VBATԴ��ADCͨ��ʹ��	ADC_VBATCmd����������

 	
   2. ʹ��ADC_RegularChannelConfig������ѡ��ADC_Channel_Vbat

 			ADC_InjectedChannelConfig��������
   3. ��ȡ�ĵ�ѹֵ��ʹ��ADC_GetConversionValue������

			ADC_GetInjectedConversionValue������
 
@endverbatim
  * @{
  */
  
  
/**
  * @brief  ���û�����¶ȴ�������Vrefint��ͨ����
  * @param  NewState: �µ�״̬�¶ȴ�������Vrefint������.
  *          ������������ǣ�ENABLE��DISABLE��
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
  * @brief  ���û����VBAT����ص�ѹ��ͨ����
  * @param  NewState: ��״̬��VBATͨ����
  *          ������������ǣ�ENABLE��DISABLE��
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

/** @defgroup ADC_Group4�����������ù���
 *  @brief    �����������ù���
 *
@verbatim   
 ===============================================================================
                  �����������ù���
 ===============================================================================  

�����������ADC�������������ṩ�Ĺ��ܣ� ������2���Ӳ���: 
  
  1. �������������ú͹�����:  
     �����ṩ�Ĺ����������õ�ADC���������ģ�   
          - ÿ��ͨ�����õ������ڳ��������
          - ����Ϊÿ��ͨ���Ĳ���ʱ��
          - ѡ������������ת������
          - ѡ�������EOC�¼�����Ϊ����
          - ��������ģʽ��*��
          - ����������ģʽ
	��ע�⣬���������Ĺ��ܣ�������ʹ��ADC_Init����������

	 
          - ɨ��ģʽ����
          - ����ģʽ���**�� 
          - �ⲿ����Դ 
          - �ⲿ������Ե
          - ���������鶨���ת������
     
     @ע����*����**��ִ����ͬ�����ã�
     
  2. ��ȡת������: ��С���ṩ��һ����Ҫ�Ĺ���
     ADC����Χ�豸����Ϊ���ĵ���ת��������ݷ���������������ȡת��ֵ��EOC��־�Զ����㡣


     
     @note ���ڶ�ADCģʽ,�ڹ�ȥ ADC1, ADC2 and ADC3 ����ת�� 
 		   ��������ͬ�ķ��ؽ�����ݣ���ѡ���Ķ�ģ��

 	  	   ѡ��ADC_GetMultiModeConversionValue����������
       
  
@endverbatim
  * @{
  */
/**
  * @brief  Ϊѡ����ADC��������������Ӧ������
  *         �����������Ͳ���ʱ�䡣
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_Channel: ADCͨ������ 
  *          �����������ȡ����ֵ֮һ��
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
  * @param  Rank: �����鶨��������С�
  *          �˲���������1��16֮�䡣
  * @param  ADC_SampleTime: ����ʱ���ֵ���趨Ϊѡ����ͨ���� 
  *          �����������ȡ����ֵ֮һ:
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
  * @brief ���úͽ���  EOC ��ÿ����������ת��
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  NewState: ѡ����ADC����״̬ EOC ��־ rising
  *          ������������ǣ�ENABLE��DISABLE��
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
  * @brief  ���û����ADC����ת��ģʽ
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  NewState: ��ѡ����״̬ ADC ����ת��ģʽ��
  *         ������������ǣ�ENABLE��DISABLE��
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
  * @brief  ����ѡ����ADC������Ĳ�����ģʽͨ���� 
  *        
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  Number: ָ���ķ�����ģʽ���������ļ���ֵ��
  *          �����ֱ�����1��8֮�䡣
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
  * @brief  ���û���ó������ŵ��ķ�����ģʽΪָ����ADC
  *        
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  NewState: ��ѡ����״̬ ADC ������ģʽ�������ŵ���
  *         
  *          ������������ǣ�ENABLE��DISABLE��
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
  * @brief  ���ص����һ������������ʹ��ADCxת���Ľ�����ݡ�
  * @param  ADCx:����x������1��2��3ѡ��ADC��Χ��
  * @retval ����ת��ֵ��
  */
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  
  /* Return the selected ADC conversion value */
  return (uint16_t) ADCx->DR;
}

/**
  * @brief  ���ع�ȥADC1��ADC2��ADC3���ڵ�ת�������ѡ���Ķ�ģʽ���ݡ�
  *         
  * @param  None  
  * @retval ����ת��ֵ��
  * @note   ��˫ģʽ������������ص�ֵ���£�
  *           Data[15:0] : these bits contain the regular data of ADC1.
  *           Data[31:16]: these bits contain the regular data of ADC2.
  * @note   ������ģʽ������������ص�ֵ���£�
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

/** @defgroup ADC_Group5��������DMA���ù���
 *  @brief    ��������DMA���ù���
 *
@verbatim   
 ===============================================================================
                   ����ͨ��DMA���ù���
 ===============================================================================  

 �����ṩ�Ĺ��ܣ���������ΪADC��DMA����ͨ����


 ����ת��������ͨ����ֵ�洢��һ�����ص����ݼĴ������������õ�ʹ�õ�DMA����һ�������ŵ���ת����

��������Ѿ��洢��ADC���ݼĴ����е����ݶ�ʧ��
  
��DMAģʽ��ʹ�ܣ�ʹ�õ�ADC_DMACmd��������������ÿ��������ͨ����ת���󣬲���һ��DMA����


  
���ݡ�DMA��ֹѡ�������ADCģʽ��
���ã�ʹ��ADC_DMARequestAfterLastTransferCmd������������
������DMA����Ľ��������ֿ�����������ģ�
  - û���µ�DMA���󷢳���DMA�����������ܹرգ�
  - ���Լ�������Ĳ������������ã���
  
���ݡ�DMA��ֹѡ���ADCģʽ������
��ʹ�õĿ�϶ADC_MultiModeDMARequestAfterLastTransferCmd������������
������DMA����Ľ��������ֿ�����������ģ�
  - û���µ�DMA���󷢳���DMA�����������ܹرգ�
  - ���Լ�������Ĳ������������ã���

@endverbatim
  * @{
  */
  
 /**
  * @brief  ���û����ָ����ADC DMA����
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  NewState: ��ѡ����ADC DMA�����״̬��
  *          ������������ǣ�ENABLE��DISABLE��
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
  * @brief ���û����ADC DMA��������ת�ƣ���ADCģʽ��
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  NewState: ��ѡ����ADC DMA��������һ�δ����״̬��
  *         ������������ǣ�ENABLE��DISABLE��
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
  * @brief  ���û����ADC DMA��������һ�δ����ADCģʽ

  * @param  NewState: ��ѡ����ADC DMA��������һ�δ����״̬��

  *         ������������ǣ�ENABLE��DISABLE��
  * @note   ������ã�����DMA����ֻҪ���ݱ�ת���� 
  *         DMAģʽ�Ķ�ADCģʽ��ѡ��ʹ��ADC_CommonInit��������
  *         �ɽṹ��ԱADC_CommonInitStruct.ADC_DMAAccessMode��
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

/** @defgroup ADC_Group6ע��ͨ�����ù���
 *  @brief   ע��ͨ�����ù���
 *
@verbatim   
 ===============================================================================
                     ע��ͨ�����ù���
 ===============================================================================  

�������ṩ�Ĺ����������õ�ADCע��ͨ����������2С�ڣ�
    
	1.ע��ͨ�����ù��ܣ�����涨�����������õ�ADCע��ͨ����


		- ע���鶨��������У�ÿ��ͨ������

		- ����ÿ��ͨ���Ĳ���ʱ��

		- �����Զ�ע��ģʽ

		- ����������ģʽ

		- ɨ��ģʽ����

		- �ⲿ����Դ/���

		- �ⲿ������Ե

		- ע��ͨ����������
	2.���ָ����ע��ͨ����ת�����ݣ�������ADC�����ṩ��һ����Ҫ�Ĺ��ܣ�	

		��Ϊ������ת��������ݵľ����ע��ͨ����

@endverbatim
  * @{
  */ 
/**
  * @brief  ѡ����ADCע��ͨ������Ӧ����
  *         rank in the sequencer and its sample time.
  * @param  ADCx:����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_Channel:ADCͨ�����á� 
  *          �����������ȡ����ֵ֮һ��
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
  * @param  ADC_SampleTime: ����ʱ���ֵ���趨Ϊѡ����ͨ����

  *         �����������ȡ����ֵ֮һ��
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
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
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
  * @brief  ��ע��ͨ����ת��ֵ��ƫ����
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_InjectedChannel:ADCע���ŵ���������ƫ������
  *          This parameter can be one of the following values:
  *            @arg ADC_InjectedChannel_1: Injected Channel1 selected
  *            @arg ADC_InjectedChannel_2: Injected Channel2 selected
  *            @arg ADC_InjectedChannel_3: Injected Channel3 selected
  *            @arg ADC_InjectedChannel_4: Injected Channel4 selected
  * @param  Offset: ѡ����ADCע��ͨ����ƫ��ֵ
  *         �������������һ��12λ��ֵ��
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
  * @brief  ����ʹ��ADCx�ⲿ����ע��ͨ��ת����
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_ExternalTrigInjecConv: ָ��������ע��ת����ADC������

  *         �����������ȡ����ֵ֮һ��                   
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
  * @brief  ����ʹ��ADCx�ⲿ������Եע��ͨ��ת����

  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_ExternalTrigInjecConvEdge: ָ��ADC���ⲿ������Ե

  *         ��ʼע��ת����
  *         �����������ȡ����ֵ֮һ��
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
  * @brief ������ѡ��ADC�����ʼת����ע��ͨ����
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
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
  * @brief  ��ȡѡ����ADC�����ʼע��ת��״̬��

  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @retval �µĹ��ҿ�ʼע��ת����ADC�����SET��RESET����

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
  * @brief  ���û����ѡ����ADC�Զ�ע����ת�����ڡ�
  *        
  * @param  ADCx:����x������1��2��3ѡ��ADC��Χ��
  * @param  NewState:ѡ����ADC�Զ�ע��ת������״̬
  *         ������������ǣ�ENABLE��DISABLE��
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
  * @brief  ���û����ע����Ĳ�����ģʽָ����ADCͨ��
  *        
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  NewState: ADCѡ���ķ�����ģʽע���µ�״̬ͨ���顣
  *         
  *         ������������ǣ�ENABLE��DISABLE��
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
  * @brief  ����ADCע���ŵ�ת�����
  * @param  ADCx:����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_InjectedChannel: ת����ADCע���ŵ���
  *         �����������ȡ����ֵ֮һ��
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

/** @defgroup ADC_Group7�жϺͱ�־������
 *  @brief    �жϺͱ�־������
 *
@verbatim   
 ===============================================================================
                   �жϺͱ�־������
 ===============================================================================  

�����ṩ�Ĺ��ܿ����õ�ADC�жϣ� �õ��ĵ�λ����ȷ�ı�־���жϱ�־λ��


  
ÿ��ADC�ṩ��4���ж�Դ��6��־�ɷ�Ϊ��
  3 groups:
  
  I. ��־���ж�ADC��������
  =================================================
  Flags :
  ---------- 
     1. ADC_FLAG_OVR : ���޼�⣬����ת��������ݶ�ʧ

     2. ADC_FLAG_EOC :����ͨ��ת������==> ����ʾ (����EOCSλ������ 
          ADC_EOCOnEachRegularChannelCmd() )������
               ==>һ����������ת��
               ==>������ת�����С�

     3. ADC_FLAG_STRT: ������������ ==>����������ʼת����




  Interrupts :
  ------------
     1. ADC_IT_OVR : ָ���ж�Դ���޼���¼��� 
     2. ADC_IT_EOC : ָ�����������˵��ж�Դת���¼��� 
                     
  
  
  II.��־���ж�ADCע��ͨ��
  =================================================
  Flags :
  ---------- 
     1. ADC_FLAG_JEOC : ע��ͨ��ת������ ==> to indicate at 
               the end of injected GROUP conversion  
              
     2. ADC_FLAG_JSTRT: ע��ͨ������ ==> ָʾӲ��ʱע����ת����ʼ.
              

  Interrupts :
  ------------
     1. ADC_IT_JEOC�� ָ��ע���ͨ����ͷ���ж�Դת���¼���
                     

  III. һ���־���жϵ�ADC
  ================================================= 
  Flags :
  ---------- 
     1. ADC_FLAG_AWD:ģ�⿴�Ź�==> to indicate ���ת���ĵ�ѹԽ���趨����ֵ��
             
              
  Interrupts :
  ------------
     1. ADC_IT_AWD : ָ���ж�Դ��ģ�⿴�Ź��¼���

  
 �û�Ӧȷ������ģʽ�����������Լ���Ӧ�ó������ADC���������¼�����ѯģʽ���ж�ģʽ��

  
�ڲ�ѯģʽ�£�����ʹ�����¹��ܣ�
      - ADC_GetFlagStatus() : ����־���¼�������
      - ADC_ClearFlag()     : ����ı�־�¼���
      
���ж�ģʽ�£�����ʹ�����¹��ܣ�
     - ADC_ITConfig()          : ���û�����ж�Դ��
     - ADC_GetITStatus()       : ����飬��������жϡ�
     - ADC_ClearITPendingBit() : ������жϹ���λ����Ӧ�ı�־����
                               
@endverbatim
  * @{
  */ 
/**
  * @brief  ���û����ָ����ADC�жϡ�
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_IT: ָ��Ҫ���û���õ�ADC�ж�Դ��
  *         �����������ȡ����ֵ֮һ��
  *            @arg ADC_IT_EOC: End of conversion interrupt mask
  *            @arg ADC_IT_AWD: Analog watchdog interrupt mask
  *            @arg ADC_IT_JEOC: End of injected conversion interrupt mask
  *            @arg ADC_IT_OVR: Overrun interrupt enable                       
  * @param  NewState: ָ����ADC�жϵ���״̬��
  *         ������������ǣ�ENABLE��DISABLE��
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
  * @brief  ����Ƿ����û�ָ����ADC��־��
  * @param  ADCx: ����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_FLAG: ָ��Ҫ���ı�־��
  *         �����������ȡ����ֵ֮һ��
  *            @arg ADC_FLAG_AWD: Analog watchdog flag
  *            @arg ADC_FLAG_EOC: End of conversion flag
  *            @arg ADC_FLAG_JEOC: End of injected group conversion flag
  *            @arg ADC_FLAG_JSTRT: Start of injected group conversion flag
  *            @arg ADC_FLAG_STRT: Start of regular group conversion flag
  *            @arg ADC_FLAG_OVR: Overrun flag                                                 
  * @retval ADC_FLAG����״̬��SET��RESET����
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
  * @brief  �����ʹ��ADCx�Ĺ����־�ġ�
  * @param  ADCx:����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_FLAG: ָ������ı�־��
  *         ������������������ֵ���κ���ϣ�
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
  * @brief ���ָ����ADC�жϷ������
  * @param  ADCx:����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_IT:ָ����ADC�ж�Դ���м�顣
  *         �����������ȡ����ֵ֮һ��
  *            @arg ADC_IT_EOC: End of conversion interrupt mask
  *            @arg ADC_IT_AWD: Analog watchdog interrupt mask
  *            @arg ADC_IT_JEOC: End of injected conversion interrupt mask
  *            @arg ADC_IT_OVR: Overrun interrupt mask                        
  * @retval ADC_IT����״̬��SET��RESET����
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
  * @brief  �����ʹ��ADCx���жϱ�־λ��
  * @param  ADCx:����x������1��2��3ѡ��ADC��Χ��
  * @param  ADC_IT:ָ��ADC�жϱ�־λ�������
  *         �����������ȡ����ֵ֮һ��
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
