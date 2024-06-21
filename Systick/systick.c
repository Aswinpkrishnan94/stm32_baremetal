#include "stm32f4xx.h"

void delay(int n);

int main(void)
{
	RCC->AHB1ENR |= (1 << 0);		 	// enable GPIO Port A clock
	GPIOA->MODER &= ~(0xFFF<<0);
	GPIOA->MODER |=  (0x400<<0);	// set GPIO Port A Pin 5 for led

	SYSTICK->RVR = 0xFFFFFF;			// reload with max value
	SYSTICK->CSR = 5;
	while(1)
	{
		GPIOA->ODR = (SYSTICK->CVR >> (23 - 5)) & 0x00000020;
	}
	return 0;
}
