#include "main.h"

int main(void)
{
	RCC->AHB1ENR |=(1 << 0);		// GPIOA
	RCC->APB1ENR |= (1 << 0);		// Timer 2

	GPIOA->MODER &= ~(3 << 10);
	GPIOA->MODER |= (2 << 10);
	GPIOA->AFR[0] |= (1 << (5*4));

	TIM2->PSC = 1599;	// 10KHz
	TIM2->ARR = 999;	// 10 Hz PWM
	TIM2->CCR1 = 0;		// 0% duty cycle
	TIM2->CCMR1 |= (6 << 4);	// PWM mode 1
	TIM2->CCER |= (1 << 0);		// output on ch1
	TIM2->CR1 |= (1 << 0);	// enable counter

	while(1)
	{
		for(int i=0;i<1000;i+=10)
		{
			TIM2->CCR1 = i;
			for(volatile int d=0;d<50000; d++);	// delay
		}
		for(int i=1000;i>=0;i-=10)
		{
			TIM2->CCR1 = i;
			for(volatile int d=0;d<50000; d++);	// delay
		}
	}
}
