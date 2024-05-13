#include "stm32f4xx.h"

void delay()
{
	for(uint32_t i=0;i<500000;i++);
}

int main()
{
	GPIO_Handle_t gpioled, gpiobutton;

  // LED connected to GPIO Port A Pin 5
	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&gpioled);

  // Button connected to GPIO Port C Pin 13
	gpiobutton.pGPIOx = GPIOC;
	gpiobutton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpiobutton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpiobutton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&gpiobutton);

	// clock enable
	GPIO_Pclk_Ctrl(GPIOA, ENABLE);
	GPIO_Pclk_Ctrl(GPIOC, ENABLE);

	while(1)
	{
		if(!GPIO_ReadFromPin(GPIOC, GPIO_PIN_NO_13))    // read from the button
		{
			GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);        // toggle led with delay
			delay();
		}
	}
	return 0;
}
