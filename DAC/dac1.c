#include "stm32f4xx.h"

int main(void)
{
	int data;

	RCC->AHB1ENR |= (1<<0);			// enable GPIO port A clock
	GPIOA->MODER &= (0x3<<8);
	GPIOA->MODER |= (0x3<<8);		// set PA4 as analog mode

	RCC->APB1ENR |= (1<<29);		// enable DAC clock
	DAC->DAC_CR |= (1<<0);			// enable DAC

	while(1)
	{
		DAC->DAC_DHR12R1 = data++ & 0x0FFF;
	}
	return 0;
}
