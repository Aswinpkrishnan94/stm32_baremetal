#include "stm32f4xx.h"

int main(void)
{
	RCC->AHB1ENR |= (1<<0);		// enable clock on GPIO Port A
	GPIOA->MODER &= ~(0x3<<10);
	GPIOA->MODER |=  (0x1<<10);	// set GPIO port A pin 5 in output mode

	// Timer 2 initialization
	/*	system clock: 16 Mhz
	 *  output: 1 Hz
	 *  PSC prescalar = 1600
	 *  ARR value = 10000
	 *  16MHz/1600/10000 = 1 Hz
	 */
	RCC->APB1ENR |= (1<<0);		// enable Timer 2 clock
	TIM2->PSC = 1600 - 1;		// divided by 1600
	TIM2->ARR = 10000 - 1;		// divided by 10000
	TIM2->CNT = 0;				// clear timer counter
	TIM2->CR1 |= (1<<0);		// enable the timer;
	while(1)
	{
		while(!(TIM2->SR & 1));		// wait until UIF bit is set
		TIM2->SR &= ~1;				//clear UIF flag bit
		GPIOA->ODR ^= (1<<5);	//toggle the led
	}
	return 0;
}
