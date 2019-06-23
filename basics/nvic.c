#include "stm32f4xx.h"
#include "nvic.h"

void NVIC_Config(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
		/* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
// 	/* Enable and set EXTI Line1 Interrupt t*/
// 	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);
	
// 	/* Enable the USARTx Interrupt */
// 	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);	
}
