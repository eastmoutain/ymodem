
#include "stm32f4xx.h"
#include "uart.h" 
#include "nvic.h"
#include "exti.h"
#include "SysTick.h"
#include "stdio.h"


int main(void)
{	
	SysTick_Init();
	NVIC_Config();
	
	uart_init();
	
	
	while(1)
		printf("hello, the program is running...\r\n ");

	
}
