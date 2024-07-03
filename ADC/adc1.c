#include "stm32f4xx.h"

void gpio_init()
{
	GPIOA->MODER &= ~(0x3<<10);
	GPIOA->MODER |= (1<<10);			// set PA5 to output mode (led)

	GPIOA->MODER &= ~(0x3<<2);
	GPIOA->MODER |= (0x3<<2);			// set PA1 into input mode (input) for ADC
}

void adc_init()
{
	// Reset ADC configuration to default
	ADC1->CR1 = 0;
	ADC1->CR2 = 0;

	// Configure ADC1
	ADC1->CR2 |= (1 << 1);          // Continuous conversion mode
	ADC1->CR1 |= (1 << 8);          // Scan mode enabled
	ADC1->SQR3 = 1;                 // Conversion sequence starts at channel 1 (PA1)
	ADC1->SQR1 = 0;                 // Conversion sequence length 1
	ADC1->SMPR2 |= (7 << 0);        // Set sample time for channel 1 to 480 cycles
	ADC1->CR2 |= (1 << 0);          // Enable ADC1
}
int main(void)
{
	int result;

	RCC->AHB1ENR |= (1<<0);				// enable GPIO port A clock
	RCC->APB2ENR |= (1 <<8);			// enable ADC1 clock

	gpio_init();
	adc_init();
	while(1)
	{
		ADC1->CR2 |=(1<<30);			// start conversion
			while(!(ADC1->SR & (1<<1)));	// wait for conversion to complete
			result = ADC1->DR;			// read data
			printf("%d\n", result);
			if(result > 2048)
			{
				GPIOA->ODR |= (1<<5);	// turn on led
			}
			else
				GPIOA->ODR &= ~(1<<5);	// turn off led
	}
	return 0;
}
