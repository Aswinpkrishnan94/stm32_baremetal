#include "stm32f4xx.h"

USART_Handle_t usart2;

void delay(int n)
{
	for(;n>0;n--)
		for(int i=0;i<2000;i++);
}

void led_blink(int value)
{
	value=value%16;		// max count to 15
	for(;value>0;value--)
	{
		GPIOA->ODR |= (1<<5);
		delay(200);
		GPIOA->ODR &= ~(1<<5);
		delay(200);
	}
	delay(800);
}

void USART2_init()
{
	RCC->APB1ENR |= (1<<17);			// enable usart2 clock

	GPIOA->MODER &= ~(0x3<<6);
	GPIOA->MODER |= (0x2<<6);		// set PA3 to alternate function mode
	GPIOA->AFR[0] &= ~(0xF<<12);
	GPIOA->AFR[0] |= (0x7<<12);		// set PA3 alternate function to USART 2

	usart2.pUSARTx = USART2;
	usart2.USARTConfig.Mode = USART_MODE_TXRX;
	usart2.USARTConfig.HwFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2.USARTConfig.ParityControl = USART_PARITY_DISABLE;
	usart2.USARTConfig.WordLength = USART_WORDLEN_8BITS;
	usart2.USARTConfig.NoStopBits = USART_STOPBITS_1;
	usart2.USARTConfig.Baudrate = USART_STD_BAUD_115200;
	USART_Init(&usart2);

	USART2->BRR = 0x008B;			// USARTDIV Mantissa and fraction part
	USART2->CR1 |= (1<<2)|(1<<5);	// receiver is enabled, RXNE interrupt enable	usart2.pUSARTx = USART2;
	USART2->CR1 |= (1<<13);			// enable usart2
}

void usart2_irqhandler(void)
{
	char ch;
	if(USART2->SR & (1<<5))			// check if RNXE flag is set
	{
		ch = USART2->DR;			// read character from USART 2
		led_blink(ch);
	}
}

void USART2_SendChar(char ch) {
    while (!(USART2->SR & USART_SR_TXE));  // Wait until TXE (Transmit data register empty)
    USART2->DR = ch;
}

int main(void)
{
	RCC->AHB1ENR |= (1<<0);			// enable clock for GPIO Port A
	GPIOA->MODER &= ~(0x3<<10);
	GPIOA->MODER |= (0x1<<10);		// set output mode

	USART2_init();					// initialize usart2
	USART2->CR1 |= (1<<5);			// RXNE interrupt enable

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI3, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI3, ENABLE);

	while(1)
	{
		USART2_SendChar('A');  // Send character 'A'
		 for (int i = 0; i < 100000; i++);  // Simple delay
	}
	return 0;
}
