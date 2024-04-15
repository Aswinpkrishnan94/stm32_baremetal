/*
 * stm32f4xx_gpio_driver.c
 *
 * Created on: Feb 24, 2024
 * Author: Aswin P Krishnan
 */

#include<stm32f4xx_gpio_driver.h>

void GPIO_Pclk_Ctrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_EN();
				}
		else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_EN();
				}
		else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_EN();
				}
		else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_EN();
				}
		else if (pGPIOx == GPIOE)
				{
					GPIOE_PCLK_EN();
				}
		else if (pGPIOx == GPIOF)
				{
					GPIOF_PCLK_EN();
				}
		else if (pGPIOx == GPIOG)
				{
					GPIOG_PCLK_EN();
				}
		else if (pGPIOx == GPIOH)
				{
					GPIOH_PCLK_EN();
				}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}
		else if(pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}
		else if(pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}
		else if(pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}
		else if(pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}
		else if(pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}
		else if(pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}
		else if(pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	// Enable clock
	GPIO_Pclk_Ctrl(pGPIOHandle->pGPIOx, ENABLE);

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Non Interrupt Mode

	 //1. Configure Mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		// Interrupt Mode
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
			{
				EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// clear RTSR bit
				EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}
			else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
			{
				EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		// clear FTSR bit
				EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}
			else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
			{
				EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}

			// 2. Configure GPIO port selection in SYSCFG EXTICR
				uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
				uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
				uint8_t portcode = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOx);
				SYSCFG_PCLK_EN();
				SYSCFG->EXTICR[temp1] = portcode << (temp2 *4);

			// 3. Enable EXTI intr using IMR
				EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	//2. Configure the speed
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. Configure the pupd settings
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->PUPDR |= temp;


	//4. Configure the output type
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. Configure the alternate functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN)
		{
			uint32_t temp1, temp2 = 0;
			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFR[temp1]&=  ~(0xF << (4*temp2));  	// clearing register
			pGPIOHandle->pGPIOx->AFR[temp1]|= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << 4*temp2);
		}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		{
				GPIOA_REG_RST();
		}
	else if (pGPIOx == GPIOB)
		{
			GPIOB_REG_RST();
		}
	else if (pGPIOx == GPIOC)
		{
			GPIOC_REG_RST();
		}
	else if (pGPIOx == GPIOD)
		{
			GPIOD_REG_RST();
		}
	else if (pGPIOx == GPIOE)
		{
			GPIOE_REG_RST();
		}
	else if (pGPIOx == GPIOF)
		{
			GPIOF_REG_RST();
		}
	else if (pGPIOx == GPIOG)
		{
			GPIOG_REG_RST();
		}
	else if (pGPIOx == GPIOH)
		{
			GPIOH_REG_RST();
		}
}

uint8_t GPIO_ReadFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber)&0x00000001);
	return value;
}

uint16_t GPIO_ReadFromPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber < 32)
			{
				*NVIC_ICER0 |= (1 << IRQNumber);
			}
		else if(IRQNumber > 31 && IRQNumber < 64)
			{
				*NVIC_ICER1 |= (1 << (IRQNumber % 32));
			}
		else if(IRQNumber > 63 && IRQNumber < 96)
			{
				*NVIC_ICER2 |= (1 << (IRQNumber % 64));
			}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
