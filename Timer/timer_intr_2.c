
#include "stm32f4xx.h"

void delay(int n)
{
	for(;n>0;n--)
		for(int i=0;i<3195;i++);
}
int main(void)
{
	RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2);	// Enable clock for GPIO Ports A, B and C

	GPIOA->MODER &= ~(0x3 << 10);
	GPIOA->MODER |= (0x1<<10);			// set to output mode

	GPIOB->MODER &= ~(0x3<<20);			// set to input mode
	GPIOB->PUPDR &= ~(0x3<<20);
	GPIOB->PUPDR |= (0x1<<20);			// enable pull up resistor

	GPIOC->MODER &= ~(0x3 << 26);		// set to input mode
	GPIOC->PUPDR &= ~(0x3<<26);
	GPIOC->PUPDR |=  (0x3<<26);			// enable pull up resistor on the pin

	RCC->APB2ENR |= (1<<14);			// enable SYSCFG clock

	SYSCFG->EXTICR[3] &= ~(0xF<<(1*4));	// clear port for EXTI13
	SYSCFG->EXTICR[3] |= (2<<(1*4));	// select port C for EXTI13
	EXTI->IMR |= (1<<13); 				// unmask EXTI13
	EXTI->FTSR |= (1 <<13);				// falling edge trigger

	SYSCFG->EXTICR[2] &= ~(0xF<<(2*4));	// clear port B for EXTI10
	SYSCFG->EXTICR[2] |= (2<<(2*4));	// select port B for EXTI10
	EXTI->IMR |= (1<<10); 				// unmask EXTI10
	EXTI->FTSR |= (1 <<10);				// falling edge trigger

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);
	return 0;
}

void EXTI15_10_IRQHandler(void)
{
	// Check if EXTI line 13 caused the interrupt
	    if (EXTI->PR & (1 << 13))
	    {
	        // Clear the pending interrupt flag
	        EXTI->PR |= (1 << 13);

	        // Toggle the LED
	        GPIOA->ODR ^= (1 << 5);

	        // Simple delay for debounce
	        delay(250);
	    }
	    // Check if EXTI line 10 caused the interrupt
	    else if(EXTI->PR & (1<<10))
	    {
	        // Clear the pending interrupt flag
	        EXTI->PR |= (1 << 10);

	        // Toggle the LED
	        GPIOA->ODR ^= (1 << 5);

	        // Simple delay for debounce
	        delay(50);
	    }
}

