#include "stm32f4xx.h"

int main(void)
{
	RCC->AHB1ENR |= (1<<0);		// enable clock on GPIO Port A
	GPIOA->MODER &= ~(0x3<<10);
	GPIOA->MODER |=  (0x1<<10);	// set GPIO port A pin 5 in output mode

	// Timer 3 initialization
	/*	system clock: 16 Mhz
	 *  output: 1 Hz
	 *  PSC prescalar = 1600
	 *  ARR value = 10000
	 *  16MHz/1600/10000 = 1 Hz
	 */
	RCC->APB1ENR |= (1<<1);		// enable Timer 3 clock
	TIM3->PSC = 1600 - 1;		// divided by 1600
	TIM3->ARR = 10000 - 1;		// divided by 10000
	TIM3->CNT = 0;				// clear timer counter
	TIM3->CR1 |= (1<<0);		// enable the timer;
	while(1)
	{
		while(!(TIM3->SR & 1));		// wait until UIF bit is set
		TIM3->SR &= ~1;				//clear UIF flag bit
		GPIOA->ODR ^= (1<<5);	//toggle the led
	}
	return 0;
}
