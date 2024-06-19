#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

char msg[100] = "HELLO UART TX TESTING";      // random message

USART_Handle_t usart2;

void usart2_Init(void)
{
  // Initialize USART communication 
	usart2.pUSARTx = USART2;
	usart2.USARTConfig.Mode = USART_MODE_ONLY_TX;
	usart2.USARTConfig.HwFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2.USARTConfig.ParityControl = USART_PARITY_DISABLE;
	usart2.USARTConfig.WordLength = USART_WORDLEN_8BITS;
	usart2.USARTConfig.NoStopBits = USART_STOPBITS_1;
	usart2.USARTConfig.Baudrate = USART_STD_BAUD_115200;
	USART_Init(&usart2);
}

void usart2_gpio(void)
{
  // Enable GPIO Port A pin 2 for Tx
	GPIO_Handle_t usartgpio;
	usartgpio.pGPIOx = GPIOA;
	usartgpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usartgpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usartgpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usartgpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usartgpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	usartgpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usartgpio);
}

void gpiobutton_Init(void)
{
  // Initialize GPIO Port C Pin 13 for on-board user button
	GPIO_Pclk_Ctrl(GPIOC, ENABLE);

	GPIO_Handle_t gpiobutton;
	gpiobutton.pGPIOx = GPIOC;
	gpiobutton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpiobutton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpiobutton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpiobutton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&gpiobutton);
}

void gpioled_Init(void)
{
// Initialize GPIO Port A Pin 5 for on-board LED
	GPIO_Handle_t gpioled;
	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&gpioled);
}

void delay(void)
{
	for(uint32_t i=0;i<10000;i++);
}

int main(void)
{
	// Enable Clocks for GPIOA
	GPIO_Pclk_Ctrl(GPIOA, ENABLE);

	gpiobutton_Init();
	gpioled_Init();
	usart2_Init();
	usart2_gpio();
  
// Enable Clocks for USART2
	USART_PeriClockControl(USART2 ,ENABLE);

	while(1)
	{
		while(!(GPIO_ReadFromPin(GPIOC, GPIO_PIN_NO_13)));      // read fstatus of on-board user button 

		delay();    // small delay

		USART_SendData(&usart2, (uint8_t*)msg, strlen(msg));    // transmit characters
		GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);      
	}
	return 0;
}
