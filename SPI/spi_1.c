#include "stm32f4xx.h"

void delay(int n)
{
	for(;n>0;n--)
		for(int i=0;i<1000;i++);
}

void gpio_init()
{
	// PA5 for MOSI, PA7 for SPI clock, PA4 for output for SPI
	GPIOA->MODER &= ~((0x3<<14)|(0x3<<10)|(0x3<<8));
	GPIOA->MODER |= (0x3<<10)|(0x3<<14);			// set pin to alternate function mode
	GPIOA->MODER |= (0x1<<8);

	GPIOA->AFR[0]&= ~((0xF<<28)|(0xF<<20));
	GPIOA->AFR[0] |= (0x5<<28)|(0x5<<20);			// set alt mode for SPI1
}

void spi_init()
{
	SPI1->CR1 = 0x31C;		// set baud rate, 8-bit data frame
	SPI1->CR2 = 0;
	SPI1->CR1 |= 0x40;		// enable SPI1
}

void spi_write(unsigned char ch)
{
	GPIOA->ODR |= (1<<4);		// assert slave select
	while(!(SPI1->SR & 0x80));	// wait for tx to be done
	GPIOA->ODR &= ~(1<<4);
}

int main(void)
{
	RCC->APB2ENR |= (1<<12);			// enable SPI1 clock
	RCC->AHB1ENR |= (1<<0);				// enable GPIO A clock

	gpio_init();
	spi_init();

	while(1)
	{
		for(char c='A';c<='Z';c++)
		{
			spi_write(c);				// transmit using SPI1
			delay(100);
		}
	}
	return 0;
}
