#include "stm32f4xx.h"

int main()
{
	GPIO_Handle_t gpioled, gpiobutton;

	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&gpioled);

	gpiobutton.pGPIOx = GPIOC;
	gpiobutton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpiobutton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpiobutton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_Init(&gpiobutton);

	// clock enable
	GPIO_Pclk_Ctrl(GPIOA, ENABLE);
	GPIO_Pclk_Ctrl(GPIOC, ENABLE);

	while(1)
	{
		if(!GPIO_ReadFromPin(GPIOC, GPIO_PIN_NO_0))
		{
			GPIO_WriteToPin(GPIOA, GPIO_PIN_NO_5, SET);
		}
		else
		{
			GPIO_WriteToPin(GPIOA, GPIO_PIN_NO_5, RESET);
		}
	}
	return 0;
}
