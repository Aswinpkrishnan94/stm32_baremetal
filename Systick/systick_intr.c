#include "stm32f4xx.h"

int main(void)
{
	RCC->AHB1ENR |= (1<<0);			// enable GPIO Port A clock
	GPIOA->MODER &= ~(0x3<<10);
	GPIOA->MODER |= (0x1<<10);		// set pin 5 to output mode for LED

	SYSTICK->RVR = 16000000-1;
	SYSTICK->CVR = 0;
	SYSTICK->CSR = 7;

// Enable SysTick Interrupt
	*NVIC_ISER0 |= (1<<15);

	while(1)
	{
	}
	return 0;
}

void SysTick_Handler(void)
{
	GPIOA->ODR ^= (1<<5);			// toggle the led
}
