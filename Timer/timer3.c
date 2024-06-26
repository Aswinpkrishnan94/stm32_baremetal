#include "stm32f4xx.h"

int timestamp = 0;

int main(void)
{
	RCC->AHB1ENR |= (1<<0);		// enable clock on GPIO Port A
	GPIOA->MODER &= ~(0x3<<10)|(0x3<<12);
	GPIOA->MODER |=  (0x2<<10)|(0x2<<12);	// set GPIO port A pin 5 in alternate function mode
	GPIOA->AFR[0] &= ~(0xF << (4*5));	// clear AF1
	GPIOA->AFR[0] |= (1 << (4*5));	// set AF1 for PA5

	GPIOA->AFR[0] &= ~(0xF << (4*6));	// clear AF1
	GPIOA->AFR[0] |= (1 << (4*6));	// set AF1 for PA5

	RCC->APB1ENR |= (1<<0);		// enable clock for Timer
	TIM2->PSC = 1600-1;			// timer prescalar value
	TIM2->ARR = 10000-1;		// timer auto reload value
	TIM2->CCMR1 = 0x30;			// set output to toggle on match
	TIM2->CCR1 = 0;				// set match value
	TIM2->CCER |= (1<<0);		// enable ch1 compare mode
	TIM2->CNT = 0;				// clear count value
	TIM2->CR1 |= (1<<0);		// enable Timer 2

	RCC->APB1ENR |= (1<<1);		// enable clock for Timer
	TIM3->PSC = 1600-1;			// timer prescalar value
	TIM3->ARR = 10000-1;		// timer auto reload value
	TIM3->CCMR1 = 0x41;			// set ch1 to capture at every edge
	TIM3->CCR1 = 0;				// set match value
	TIM3->CCER |= (1<<0);		// enable ch1 compare mode
	TIM3->CNT = 0;				// clear count value
	TIM3->CR1 |= (1<<0);		// enable Timer 3

	while(1)
	{
		while(!(TIM3->SR & (1<<1)));
		timestamp = TIM3->CCR1;		// read captured counter value
	}
	return 0;
}
