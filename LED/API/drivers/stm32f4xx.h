/*
 * stm32f4xx.h
 * Description: MCU specific header file to include all peripheral and their memory addresses
 * Created on: Feb 24, 2024
 * Author: Aswin P Krishnan
 */

#ifndef STM32F4XX_H_
#define STM32F4XX_H_
#include <stdint.h>
#include <stdio.h>

#define __vo volatile

#define NVIC_ISER0           			(__vo uint32_t*)0xE000E100
#define NVIC_ISER1         				(__vo uint32_t*)0xE000E104
#define NVIC_ISER2         			  	(__vo uint32_t*)0xE000E108
#define NVIC_ISER3           			(__vo uint32_t*)0xE000E10c

#define NVIC_ICER0 						(__vo uint32_t*)0XE000E180
#define NVIC_ICER1						(__vo uint32_t*)0XE000E184
#define NVIC_ICER2  					(__vo uint32_t*)0XE000E188
#define NVIC_ICER3						(__vo uint32_t*)0XE000E18C

#define NVIC_PR_BASE_ADDR 				((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED  		4

#define FLASH_BASE_ADDR					0x08000000U
#define ROM_BASE_ADDR					0x1FFF0000U
#define SRAM1_BASE_ADDR					0x20000000U
#define SRAM2_BASE_ADDR					0x2001C000U
#define SRAM_BASE_ADDR					SRAM1_BASE_ADDR

#define APB1_PERI_BASE_ADDR				0x40000000U
#define APB2_PERI_BASE_ADDR				0x40010000U
#define AHB1_PERI_BASE_ADDR				0x40020000U
#define AHB2_PERI_BASE_ADDR				0x50000000U
#define AHB3_PERI_BASE_ADDR				0x60000000U

#define GPIOA_BASE_ADDR					AHB1_PERI_BASE_ADDR
#define GPIOB_BASE_ADDR					AHB1_PERI_BASE_ADDR + 0x0400
#define GPIOC_BASE_ADDR					AHB1_PERI_BASE_ADDR + 0x0800
#define GPIOD_BASE_ADDR					AHB1_PERI_BASE_ADDR + 0x0C00
#define GPIOE_BASE_ADDR					AHB1_PERI_BASE_ADDR + 0x1000
#define GPIOF_BASE_ADDR					AHB1_PERI_BASE_ADDR + 0x1400
#define GPIOG_BASE_ADDR					AHB1_PERI_BASE_ADDR + 0x1800
#define GPIOH_BASE_ADDR					AHB1_PERI_BASE_ADDR + 0x1C00

#define RCC_BASE_ADDR					AHB1_PERI_BASE_ADDR + 0x3800

#define I2C1_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x5400
#define I2C2_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x5800
#define I2C3_BASE_ADDR					APB1_PERI_BASE_ADDR	+ 0x5C00

#define CAN1_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x6400
#define CAN2_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x6800

#define DAC_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x7400

#define UART4_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x4C00
#define UART5_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x5000

#define USART2_BASE_ADDR				APB1_PERI_BASE_ADDR + 0x4400
#define USART3_BASE_ADDR				APB1_PERI_BASE_ADDR + 0x4800

#define SPI2_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x3800
#define SPI3_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x3C00

#define TIM2_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x0000
#define TIM3_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x0400
#define TIM4_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x0800
#define TIM5_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x0C00
#define TIM6_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x1000
#define TIM7_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x1400
#define TIM12_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x1800
#define TIM13_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x1C00
#define TIM14_BASE_ADDR					APB1_PERI_BASE_ADDR + 0x2000

#define TIM1_BASE_ADDR					APB2_PERI_BASE_ADDR + 0x0000
#define TIM8_BASE_ADDR					APB2_PERI_BASE_ADDR + 0x0400
#define USART1_BASE_ADDR				APB2_PERI_BASE_ADDR + 0x1000
#define USART6_BASE_ADDR				APB2_PERI_BASE_ADDR + 0x1400
#define ADC1_2_3_BASE_ADDR				APB2_PERI_BASE_ADDR + 0x2000
#define SPI1_BASE_ADDR					APB2_PERI_BASE_ADDR + 0x3000
#define SPI4_BASE_ADDR					APB2_PERI_BASE_ADDR + 0x3400
#define SYSCFG_BASE_ADDR				APB2_PERI_BASE_ADDR + 0x3800
#define EXTI_BASE_ADDR					APB2_PERI_BASE_ADDR + 0x3C00
#define TIM9_BASE_ADDR					APB2_PERI_BASE_ADDR + 0x4000
#define TIM10_BASE_ADDR					APB2_PERI_BASE_ADDR + 0x4400
#define TIM11_BASE_ADDR					APB2_PERI_BASE_ADDR + 0x4800


// GPIO Peripheral Definition Structure
typedef struct
{
	__vo uint32_t MODER;				// Mode Register
	__vo uint32_t OTYPER;				// Output Type Register
	__vo uint32_t OSPEEDR;				// Output Speed Register
	__vo uint32_t PUPDR;				// Pull up Pull down Register
	__vo uint32_t IDR;					// Input Data Register
	__vo uint32_t ODR;					// Output Data Register
	__vo uint32_t BSRR;					// Bit Set Reset Register
	__vo uint32_t LCKR;					// Lock Register
	__vo uint32_t AFR[2];				// Alternate Function Register Low and High
}GPIO_RegDef_t;

#define GPIOA  (GPIO_RegDef_t*)GPIOA_BASE_ADDR
#define GPIOB  (GPIO_RegDef_t*)GPIOB_BASE_ADDR
#define GPIOC  (GPIO_RegDef_t*)GPIOC_BASE_ADDR
#define GPIOD  (GPIO_RegDef_t*)GPIOD_BASE_ADDR
#define GPIOE  (GPIO_RegDef_t*)GPIOE_BASE_ADDR
#define GPIOF  (GPIO_RegDef_t*)GPIOF_BASE_ADDR
#define GPIOG  (GPIO_RegDef_t*)GPIOG_BASE_ADDR
#define GPIOH  (GPIO_RegDef_t*)GPIOH_BASE_ADDR

// RCC Register Definition Structure
typedef struct
{
	  __vo uint32_t CR;            // Clock Control Register
	  __vo uint32_t PLLCFGR;       // PLL Configuration Register
	  __vo uint32_t CFGR;          // Clock Configuration Register
	  __vo uint32_t CIR;           // Clock Interrupt Register
	  __vo uint32_t AHB1RSTR;      // AHB1 Reset Register,
	  __vo uint32_t AHB2RSTR;      // AHB2 Reset Register,
	  __vo uint32_t AHB3RSTR;      // AHB3 Reset Register,
	  uint32_t      RESERVED0;     // Reserved
	  __vo uint32_t APB1RSTR;      // APB1 Reset Register
	  __vo uint32_t APB2RSTR;      // APB2 Reset Register
	  uint32_t      RESERVED1[2];  // Reserved
	  __vo uint32_t AHB1ENR;       // AHB1 Enable Register,
	  __vo uint32_t AHB2ENR;       // AHB2 Enable Register,
	  __vo uint32_t AHB3ENR;       // AHB3 Enable Register,
	  uint32_t      RESERVED2;     // Reserved
	  __vo uint32_t APB1ENR;       // APB1 Enable Register
	  __vo uint32_t APB2ENR;       // APB2 Enable Register
	  uint32_t      RESERVED3[2];  // Reserved
	  __vo uint32_t AHB1LPENR;     // AHB1 Low Power Mode Enable Register
	  __vo uint32_t AHB2LPENR;     // AHB2 Low Power Mode Enable Register
	  __vo uint32_t AHB3LPENR;     // AHB3 Low Power Mode Enable Register
	  uint32_t      RESERVED4;     // Reserved
	  __vo uint32_t APB1LPENR;     // APB1 Low Power Mode Enable Register
	  __vo uint32_t APB2LPENR;     // APB2 Low Power Mode Enable Register
	  uint32_t      RESERVED5[2];  // Reserved
	  __vo uint32_t BDCR;          // Backup Domain Control Register
	  __vo uint32_t CSR;           // Clock Control And Status Register
	  uint32_t      RESERVED6[2];  // Reserved
	  __vo uint32_t SSCGR;         // Spread Spectrum Clock Generation Register
	  __vo uint32_t PLLI2SCFGR;    // PLL I2S Configuration Register
	  __vo uint32_t PLLSAICFGR;    // PLL Configuration Register
	  __vo uint32_t DCKCFGR;       // Dedicated Clock Configuration Register,
	  __vo uint32_t CKGATENR;      // Clock Gated Enable Register
	  __vo uint32_t DCKCFGR2;      // Dedicated Clock Configuration Register 2
	} RCC_RegDef_t;

#define RCC							((RCC_RegDef_t*)RCC_BASE_ADDR)

// EXTI Peripheral Register Definition
	typedef struct
	{
		__vo uint32_t IMR;    // Interrupt Mask Register
		__vo uint32_t EMR;    // Event Mask Register
		__vo uint32_t RTSR;   // Rising Trigger Selection Register
		__vo uint32_t FTSR;   // Falling Trigger Selection Register
		__vo uint32_t SWIER;  // Software Interrupt Event Register,
		__vo uint32_t PR;     // Pending Register,

	}EXTI_RegDef_t;

#define EXTI						((EXTI_RegDef_t*)EXTI_BASE_ADDR)

// SYSCFG Definition Registers
	typedef struct
	{
		__vo uint32_t MEMRMP;       // Memory Map Register
		__vo uint32_t PMC;          // Peripheral Mode Configuration Register
		__vo uint32_t EXTICR[4];    // External Configuration Registers
		uint32_t      RESERVED1[2];
		__vo uint32_t CMPCR;        // Compensation Cell Control Register
		uint32_t      RESERVED2[2];
		__vo uint32_t CFGR;         // Configuration Register
	} SYSCFG_RegDef_t;

#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)


// SPIx Peripheral Register Definition
	typedef struct
	{
		__vo uint32_t CR1;        // Control Register 1
		__vo uint32_t CR2;        // Control Register 2
		__vo uint32_t SR;         // Status Register
		__vo uint32_t DR;         // Data Register
		__vo uint32_t CRCPR;      // CRC Polynomial Register
		__vo uint32_t RXCRCR;     // Receive CRC Register
		__vo uint32_t TXCRCR;     // Transmit CRC Register
		__vo uint32_t I2SCFGR;    // I2S Configuration Register
		__vo uint32_t I2SPR;      // I2S Prescaler Register
	}SPI_RegDef_t;

// I2C Peripheral Register Definition
	typedef struct
	{
	  __vo uint32_t CR1;        // Control Register 1
	  __vo uint32_t CR2;        // Control Register 2
	  __vo uint32_t OAR1;       // Own Address Register 1
	  __vo uint32_t OAR2;       // Own Address Register 2
	  __vo uint32_t DR;         // Data Register
	  __vo uint32_t SR1;        // Status register 1
	  __vo uint32_t SR2;        // Status Register 2
	  __vo uint32_t CCR;        // Clock Control Register
	  __vo uint32_t TRISE;      // TRISE Register
	  __vo uint32_t FLTR;       // FLTR Register
	}I2C_RegDef_t;

// USART Peripheral Definition Structure
	typedef struct
	{
		__vo uint32_t SR;         // Baud Rate Register
		__vo uint32_t DR;         // Data Register
		__vo uint32_t BRR;        // Baud Rate Register
		__vo uint32_t CR1;        // Control Register 1
		__vo uint32_t CR2;        // Control Register 2
		__vo uint32_t CR3;        // Control Register 3
		__vo uint32_t GTPR;       // Guard Time and Prescalar Register
	}USART_RegDef_t;

#define SPI1  				((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASE_ADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASE_ADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASE_ADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASE_ADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASE_ADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASE_ADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASE_ADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASE_ADDR)

// Clock Enable Macros
#define GPIOA_PCLK_EN()    			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1 << 7))
#define DMA1_PCLK_EN()				(RCC->AHB1ENR |= (1 << 21))
#define DMA2_PCLK_EN()				(RCC->AHB1ENR |= (1 << 22))

#define	TIM2_PCLK_EN()				(RCC->APB1ENR |= (1 << 0))
#define	TIM3_PCLK_EN()				(RCC->APB1ENR |= (1 << 1))
#define	TIM4_PCLK_EN()				(RCC->APB1ENR |= (1 << 2))
#define	TIM5_PCLK_EN()				(RCC->APB1ENR |= (1 << 3))
#define	TIM6_PCLK_EN()				(RCC->APB1ENR |= (1 << 4))
#define	TIM7_PCLK_EN()				(RCC->APB1ENR |= (1 << 5))
#define	TIM12_PCLK_EN()				(RCC->APB1ENR |= (1 << 6))
#define	TIM13_PCLK_EN()				(RCC->APB1ENR |= (1 << 7))
#define	TIM14_PCLK_EN()				(RCC->APB1ENR |= (1 << 8))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 23))
#define CAN1_PCLK_EN()				(RCC->APB1ENR |= (1 << 25))
#define CAN2_PCLK_EN()				(RCC->APB1ENR |= (1 << 26))
#define DAC_PCLK_EN()				(RCC->APB1ENR |= (1 << 29))

#define TIM1_PCLK_EN()				(RCC->APB2ENR |= (1 << 0))
#define TIM8_PCLK_EN()				(RCC->APB2ENR |= (1 << 1))
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))
#define ADC1_PCLK_EN()				(RCC->APB2ENR |= (1 << 8))
#define ADC2_PCLK_EN()				(RCC->APB2ENR |= (1 << 9))
#define ADC3_PCLK_EN()				(RCC->APB2ENR |= (1 << 10))
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
#define SPI4_PCLK_EN()				(RCC->APB2ENR |= (1 << 13))
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))
#define TIM9_PCLK_EN()				(RCC->APB2ENR |= (1 << 16))
#define TIM10_PCLK_EN()				(RCC->APB2ENR |= (1 << 17))
#define TIM11_PCLK_EN()				(RCC->APB2ENR |= (1 << 18))

// Clock Disable Macros
#define GPIOA_PCLK_DI()    			(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 7))
#define DMA1_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 21))
#define DMA2_PCLK_DI()				(RCC->AHB1ENR &= ~(1 << 22))

#define	TIM2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 0))
#define	TIM3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 1))
#define	TIM4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 2))
#define	TIM5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 3))
#define	TIM6_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 4))
#define	TIM7_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 5))
#define	TIM12_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 6))
#define	TIM13_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 7))
#define	TIM14_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 8))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 23))
#define CAN1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 25))
#define CAN2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 26))
#define DAC_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 29))

#define TIM1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 0))
#define TIM8_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 1))
#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 5))
#define ADC1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 8))
#define ADC2_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 9))
#define ADC3_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 10))
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI4_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 13))
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))
#define TIM9_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 16))
#define TIM10_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 17))
#define TIM11_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 18))

#define GPIOA_REG_RST()				do{(RCC->AHB1RSTR |= (1 << 0));(RCC->AHB1RSTR &=~(1<<0));}while(0)
#define GPIOB_REG_RST()				do{(RCC->AHB1RSTR |= (1 << 1));(RCC->AHB1RSTR &=~(1<<1));}while(0)
#define GPIOC_REG_RST()				do{(RCC->AHB1RSTR |= (1 << 2));(RCC->AHB1RSTR &=~(1<<2));}while(0)
#define GPIOD_REG_RST()				do{(RCC->AHB1RSTR |= (1 << 3));(RCC->AHB1RSTR &=~(1<<3));}while(0)
#define GPIOE_REG_RST()				do{(RCC->AHB1RSTR |= (1 << 4));(RCC->AHB1RSTR &=~(1<<4));}while(0)
#define GPIOF_REG_RST()				do{(RCC->AHB1RSTR |= (1 << 5));(RCC->AHB1RSTR &=~(1<<5));}while(0)
#define GPIOG_REG_RST()				do{(RCC->AHB1RSTR |= (1 << 6));(RCC->AHB1RSTR &=~(1<<6));}while(0)
#define GPIOH_REG_RST()				do{(RCC->AHB1RSTR |= (1 << 7));(RCC->AHB1RSTR &=~(1<<7));}while(0)

#define	GPIO_BASE_ADDR_TO_CODE(x)   ((x==GPIOA)?0:\
									 (x==GPIOB)?1:\
									 (x==GPIOC)?2:\
									 (x==GPIOD)?3:\
									 (x==GPIOE)?4:\
									 (x==GPIOF)?5:\
									 (x==GPIOG)?6:\
									 (x==GPIOH)?7:0)

// Vector Table
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     	31
#define IRQ_NO_I2C1_ER     	32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

// Priority Levels
#define NVIC_IRQ_PRI0    	0
#define NVIC_IRQ_PRI15    	15



#define SET			1
#define RESET		0
#define ENABLE		SET
#define DISABLE		RESET
#define FLAG_RESET 	RESET
#define FLAG_SET	SET

#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_spi_driver.h"
#include "stm32f4xx_i2c_driver.h"
#include "stm32f4xx_usart_driver.h"

#endif /* STM32F4XX_H_ */
