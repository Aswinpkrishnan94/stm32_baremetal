#include "stm32f4xx.h"

int main(void)
{
	RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2);		// enable clock on GPIO Port A and Port B and Port C
	GPIOA->MODER &= ~(0x3<<10);
	GPIOA->MODER |=  (0x2<<10);	// set GPIO port A pin 5 in alternate function mode
	GPIOA->AFR[0] &= ~(0xF << (4*5));	// clear AF1
	GPIOA->AFR[0] |= (1 << (4*5));	// set AF1 for PA5

	GPIOC->MODER &= ~(0x3<<26);

	// configure PB8 as input of Timer 2 ETR

	RCC->APB1ENR |= (1<<0);		// enable clock for Timer
	GPIOB->MODER &= ~(0x3<<16);
	GPIOB->MODER |=  (0x2<<16);	// set GPIO port A pin 5 in alternate function mode
	GPIOB->AFR[1] &= ~(0xF << (4*0));	// clear AF1
	GPIOB->AFR[1] |= (1 << (4*0));	// set AF1 for Timer 2 ETR

	TIM2->SMCR = 0x4377;		// use ETR as clock source
	TIM2->CNT = 0;				// clear count value
	TIM2->CR1 |= (1<<0);		// enable Timer 2

	while(1)
	{
		if(TIM2->CNT & (1<<0))
		{
			GPIOA->ODR |= 0x20;	// turn on  LED
		}
		else
		{
			GPIOA->ODR &= ~(0x20);	// turn off  LED
		}
	}
	return 0;
}
