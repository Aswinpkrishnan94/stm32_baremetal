#include "stm32f4xx.h"

#define SLAVE_ADDR		0x68

void delay(int n)
{
	for(;n>0;n--)
		for(int i=0;i<2000;i++);
}

void gpio_init()
{
	GPIOB->MODER &= ~((0x3<<16)|(0x3<<18));
	GPIOB->MODER |= (0x3<<16)|(0x3<<18);			// PB8, PB9 alt function mode

	GPIOB->AFR[1] &= ~((0xF<<0)|(0xf<<4));
	GPIOB->AFR[1] |= (0x4<<0)|(0x4<<4);

	GPIOB->OTYPER |= (0x3<<8);		//output open drain
	GPIOB->PUPDR &= ~(0xF<<16);
	GPIOB->PUPDR |= (0x5<<16);		//pull up
}
void i2c_init()
{
	I2C1->CR1 = (0x1<<15);		// software reset for I2C1
	I2C1->CR1 &= ~(0x1<<15);	// out of reset
	I2C1->CR2 |= (0x1<<4);		// peripheral clock is 16 MHz
	I2C1->CCR = 80;				// standard mode, 100 KHz clock
	I2C1->TRISE = 17;			// max rise time
	I2C1->CR1 |= (1<<0);		// enable I2C1
}

void i2c_bytewrite(char saddr, char maddr, char data)
{
	volatile int temp;

	while(!(I2C1->SR2 & 2));		// wait until bus not busy
	I2C1->CR1 |= (0x1<<8);			// generate start
	while(!(I2C1->SR1 & 1));		// wait until flag is set

	I2C1->DR = saddr <<1;			// transmit slave address
	while(!(I2C1->SR1 & 2));

	temp = I2C1->SR2;				// clear addr flag

	while(!(I2C1->SR1 & 0x80));		// wait until data register is empty
	I2C1->DR = maddr;				// send master address
	while(!(I2C1->SR1 & 0x80));
	I2C1->DR = data;				// transmit data
	while(!(I2C1->SR1 & 4));		// wait until finished
	I2C1->CR1 &= ~(0x1<<9);			// generate stop

}

int main(void)
{
	RCC->AHB1ENR |= (1<<1);		// enable GPIO Port B clock
	RCC->APB1ENR |= (1<<21);	// enable i2c1 clock

	gpio_init();
	i2c_init();
	i2c_bytewrite(SLAVE_ADDR, 0x0E, 0);

	while(1)
	{

	}
	return 0;
}
