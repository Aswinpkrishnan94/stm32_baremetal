#include "stm32f4xx.h"
#include "stm32f4xx_gpio_driver.h"

void delay()
{
	for(uint32_t i=0;i<500000/2;i++);
}
int main(void)
{
	GPIO_Handle_t gpioled, gpiobutton;

	gpioled.pGPIOx = GPIOA;
	gpioled.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	gpioled.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIOPinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioled.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioled.GPIOPinConfig.GPIO_PinSpeed =  GPIO_SPEED_FAST;

	gpiobutton.pGPIOx = GPIOC;
	gpiobutton.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpiobutton.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpiobutton.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	gpiobutton.GPIOPinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOA, ENABLE);    // Enable clock for GPIO Port A
	GPIO_Init(&gpioled);                     // GPIO Port A Initialize
	GPIO_PeriClockControl(GPIOC, ENABLE);    // Enable Clock for GPIO Port C
	GPIO_Init(&gpiobutton);                  // GPIO Port B Initialize

	while(1)
	{
    // When button is pressed, turn on the led connected GPIO Port A Pin 6. Otherwise turn off the led
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)==LOW)
		{
			delay();      // button debounce time
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_6, HIGH);
		}
		else
		{
			delay();
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_6, LOW);
		}
	}
	return 0;
}
