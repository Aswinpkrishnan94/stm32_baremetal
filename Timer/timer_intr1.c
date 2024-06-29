#include "stm32f4xx.h"

void delay(int n)
{
	for(;n>0;n--)
		for(int i=0;i<3195;i++);
}
int main(void)
{
	RCC->AHB1ENR |= (1<<2)|(1<<0);	// Enable clock for GPIO Ports A and C

	GPIOA->MODER &= ~(0x3 << 10);
	GPIOA->MODER |= (0x1<<10);		// set to output mode

	GPIOC->MODER &= ~(0x3 << 26);	// set to input mode
	GPIOC->PUPDR &= ~(0x3<<26);
	GPIOC->PUPDR |=  (0x3<<26);			// enable pull up resistor on the pin

	RCC->APB2ENR |= (1<<14);		// enable SYSCFG clock

	SYSCFG->EXTICR[3] &= ~(0xF<<(1*4));	// clear port for EXTI13
	SYSCFG->EXTICR[3] |= (2<<(1*4));	// select port C for EXTI13
	EXTI->IMR |= (1<<13); 				// unmask EXTI13
	EXTI->FTSR |= (1 <<13);				// falling edge trigger

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);
	return 0;
}

void EXTI15_10_IRQHandler(void)
{
		GPIO_IRQHandling(GPIO_PIN_NO_13);
		delay(250);
		GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);
		delay(250);
}
