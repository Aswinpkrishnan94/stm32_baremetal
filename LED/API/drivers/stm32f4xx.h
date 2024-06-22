/*
 * stm32f4xx.h
 * Description: MCU specific header file to include all peripheral and their memory addresses
 * Created on: Feb 24, 2024
 * Author: Aswin P Krishnan
 */

#ifndef STM32F4XX_H_
#define STM32F4XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))


/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U   		/*!<explain this macro briefly here  */
#define SRAM1_BASEADDR						0x20000000U  		/*!<explain this macro briefly here  */
#define SRAM2_BASEADDR						0x2001C000U 		/*!<explain this macro briefly here  */
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM 								SRAM1_BASEADDR
#define SYSTICK_BASEADDR					0xE000E010U

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */

#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)
/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)

#define TIM2_BASEADDR						(APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR						(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR						(APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR						(APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR						(APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR						(APB1PERIPH_BASEADDR + 0x1400)
#define TIM12_BASEADDR						(APB1PERIPH_BASEADDR + 0x1800)
#define TIM13_BASEADDR						(APB1PERIPH_BASEADDR + 0x1C00)
#define TIM14_BASEADDR						(APB1PERIPH_BASEADDR + 0x2000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)

#define ADC1_BASEADDR						(APB2PERIPH_BASEADDR + 0x2000)
#define ADC2_BASEADDR						(APB2PERIPH_BASEADDR + 0x2000)
#define ADC3_BASEADDR						(APB2PERIPH_BASEADDR + 0x2000)

#define TIM1_BASEADDR						(APB2PERIPH_BASEADDR + 0x0000)
#define TIM8_BASEADDR						(APB2PERIPH_BASEADDR + 0x0400)
#define TIM9_BASEADDR						(APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR						(APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR						(APB2PERIPH_BASEADDR + 0x4800)



/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;                        // Mode Register                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;                       // Output Type Register     			Address offset: 0x04      */
	__vo uint32_t OSPEEDR;						// Output Speed Register				Address offset: 0x08      */
	__vo uint32_t PUPDR;						// Pull up Pull down Register			Address offset: 0x0C
	__vo uint32_t IDR;							// Input Data Register					Address offset: 0x10
	__vo uint32_t ODR;							// Output Data Register					Address offset: 0x14
	__vo uint32_t BSRR;							// Bit Set Bit Reset Register			Address offset: 0x18
	__vo uint32_t LCKR;							// Lock Register						Address offset: 0x1C
	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;



/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  __vo uint32_t CR;            // Clock Control Register     										Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       // PLL Configuration Register   										Address offset: 0x04 */
  __vo uint32_t CFGR;          // Clock Configuration Register 										Address offset: 0x08 */
  __vo uint32_t CIR;           // Clock Interrupt Register    										Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      // AHB1 Reset Register     											Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      // AHB2 Reset Register     											Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      // AHB3 Reset Register     											Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                   	 */
  __vo uint32_t APB1RSTR;      // APB1 Reset Register     											Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      // APB2 Reset Register     											Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                          		 */
  __vo uint32_t AHB1ENR;       // AHB1 Enable Register     											Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       // AHB2 Enable Register     											Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       // AHB3 Enable Register     											Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       // APB1 Enable Register     											Address offset: 0x40 */
  __vo uint32_t APB2ENR;       // APB2 Enable Register     											Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     // AHB1 Low Power Mode Enable Register    							Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     // AHB2 Low Power Mode Enable Register    							Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     // AHB3 Low Power Mode Enable Register     							Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     // APB1 Low Power Mode Enable Register,     							Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     // APB2 Low Power Mode Enable Register     							Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          // Backup Domain Control Register									Address offset: 0x70 */
  __vo uint32_t CSR;           // Clock Control And Status Register     							Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         // Spread Spectrum Clock Generation Register     					Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    // PLL I2S Configuration Register     								Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    // PLL Configuration Register    									Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       // Dedicated Clock Configuration Register 1							Address offset: 0x8C */
  __vo uint32_t CKGATENR;      // Clock Gated Enable Register     									Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      // Dedicated Clock Configuration Register 2							Address offset: 0x94 */

} RCC_RegDef_t;



/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    // Memory Map Register          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    // Event Mask Register                	Address offset: 0x04 */
	__vo uint32_t RTSR;   // Rising Trigger Selection Register  	Address offset: 0x08 */
	__vo uint32_t FTSR;   // Falling Trigger Selection Register		Address offset: 0x0C */
	__vo uint32_t SWIER;  // Software Interrupt Event Register      Address offset: 0x10 */
	__vo uint32_t PR;     // Pending Register,                      Address offset: 0x14 */

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        // Control Register 1     										Address offset: 0x00 */
	__vo uint32_t CR2;        // Control Register 2     										Address offset: 0x04 */
	__vo uint32_t SR;         // Status Register     											Address offset: 0x08 */
	__vo uint32_t DR;         // Data Register,     											Address offset: 0x0C */
	__vo uint32_t CRCPR;      // CRC Polynomial Register     									Address offset: 0x10 */
	__vo uint32_t RXCRCR;     // Receive CRC Register     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;     // Transmit CRC Register    										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    // I2S Configuration Register     								Address offset: 0x1C */
	__vo uint32_t I2SPR;      // I2S Prescaler Register    										Address offset: 0x20 */
} SPI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       // Memory Map Register                   		  Address offset: 0x00      */
	__vo uint32_t PMC;          // Peripheral Mode Configuration Register     	  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    // External Configuration Registers				  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        // Compensation Cell Control Register        	  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	__vo uint32_t CFGR;         // Configuration Register                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo uint32_t CR1;        // Control Register 1     									Address offset: 0x00 */
  __vo uint32_t CR2;        // Control Register 2       								Address offset: 0x04 */
  __vo uint32_t OAR1;       // Own Address Register 1     								Address offset: 0x08 */
  __vo uint32_t OAR2;       // Own Address Register 2     								Address offset: 0x0C */
  __vo uint32_t DR;         // Data Register     										Address offset: 0x10 */
  __vo uint32_t SR1;        // Status register 1     									Address offset: 0x14 */
  __vo uint32_t SR2;        // Status register 2    							 		Address offset: 0x18 */
  __vo uint32_t CCR;        // Clock Control Register     								Address offset: 0x1C */
  __vo uint32_t TRISE;      // TRISE Register     										Address offset: 0x20 */
  __vo uint32_t FLTR;       // FLTR Register     										Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;         // Baud Rate Register,     								Address offset: 0x00 */
	__vo uint32_t DR;         // Data Register     										Address offset: 0x04 */
	__vo uint32_t BRR;        // Baud Rate Register     								Address offset: 0x08 */
	__vo uint32_t CR1;        // Control Register 1    									Address offset: 0x0C */
	__vo uint32_t CR2;        // Control Register 2     								Address offset: 0x10 */
	__vo uint32_t CR3;        // Control Register 3   									Address offset: 0x14 */
	__vo uint32_t GTPR;       // Guard Time and Prescalar Register   					Address offset: 0x18 */
}USART_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t SR;		   // Status Register										Address offset: 0x00 */
	__vo uint32_t CR1;		   // Control Register 1									Address offset: 0x04 */
	__vo uint32_t CR2;		   // Control Register 2									Address offset: 0x08 */
	__vo uint32_t SMPR1;	   // Sample Time Register 1 								Address offset: 0x0C */
	__vo uint32_t SMPR2;	   // Sample Time Register 2								Address offset: 0x10 */
	__vo uint32_t JOFRx;	   // Injected channel data offset Register x				Address offset: 0x14-0x20 */
	__vo uint32_t HTR;  	   // Watchdog higher threshold Register 					Address offset: 0x24 */
	__vo uint32_t LTR;  	   // Watchdog lower threshold Register 					Address offset: 0x28 */
	__vo uint32_t SQR1;  	   // Regular Sequence Register 1 	 	 					Address offset: 0x2C */
	__vo uint32_t SQR2;  	   // Regular Sequence Register 2 	 	 					Address offset: 0x30 */
	__vo uint32_t SQR3;  	   // Regular Sequence Register 3 	 	 					Address offset: 0x34 */
	__vo uint32_t JSQR;  	   // Injected sequence Register x  	 	 				Address offset: 0x38 */
	__vo uint32_t JDRx;  	   // Injected data Register x 	 	 						Address offset: 0x3C-0x48 */
	__vo uint32_t DR;	  	   // Regular data Register  	 	 						Address offset: 0x4C */
	__vo uint32_t CSR;  	   // Common status Register  	 	 						Address offset: ADC1_BASEADDR+0x300 */
	__vo uint32_t CCR;  	   // Common control Register  	 	 						Address offset: ADC1_BASEADDR)+0x300+0x04 */
	__vo uint32_t CDR;  	   // Common regular data Register for dual, triple mode 	Address offset: ADC1_BASEADDR)+0x300+0x08 */
}ADC_RegDef_t;

/*
 * peripheral register definition structure for SYSTICK
 */
typedef struct{
	__vo uint32_t CSR;		// control & status register
	__vo uint32_t RVR;		// reload value register
	__vo uint32_t CVR;		// current value register
	__vo uint32_t CAL;		// calibration value register
}SysTick_RegDef_t;


/*
 * peripheral register definition structure for Timer 6 and 7
 */

typedef struct{
	__vo uint32_t CR1;		// control register 1										Address offset: 0x00 */
	__vo uint32_t CR2;		// control register 2										Address offset: 0x04 */
	__vo uint32_t SMCR;		// slave mode control register								Address offset: 0x08 */
	__vo uint32_t DIER;		// DMA/interrupt enable register							Address offset: 0x0C */
	__vo uint32_t SR;		// status register											Address offset: 0x10 */
	__vo uint32_t EGR;		// event generation register								Address offset: 0x14 */
	__vo uint32_t CCMR1;	// capture/compare mode register 1							Address offset: 0x18 */
	__vo uint32_t CCMR2;	// capture/compare mode register 2							Address offset: 0x1C */
	__vo uint32_t CCER;		// capture/compare enable register							Address offset: 0x20 */
	__vo uint32_t CNT;		//counter register											Address offset: 0x24 */
	__vo uint32_t PSC;		// prescalar register										Address offset: 0x28 */
	__vo uint32_t ARR;		// auto-reload register										Address offset: 0x2C */
	__vo uint32_t CCR1;		// capture/compare register 1								Address offset: 0x30 */
	__vo uint32_t CCR2;		// capture/compare register 2								Address offset: 0x34 */
	__vo uint32_t CCR3;		// capture/compare register 3								Address offset: 0x38 */
	__vo uint32_t CCR4;		// capture/compare register 4								Address offset: 0x3C */
	__vo uint32_t BDTR;		// break and dead time register								Address offset: 0x40 */
	__vo uint32_t DCR;		// DMA control register										Address offset: 0x44 */
	__vo uint32_t DMAR;		// DMA address for full transfer							Address offset: 0x48 */
	__vo uint32_t OR;		// option register											Address offset: 0x4C */
}Tim1_RegDef_t;


/*
 * peripheral register definition structure for Timer 2 and Timer 5
 */
typedef struct{
	__vo uint32_t CR1;		// control register 1										Address offset: 0x00 */
	__vo uint32_t CR2;		// control register 2										Address offset: 0x04 */
	__vo uint32_t SMCR;		// slave mode control register								Address offset: 0x08 */
	__vo uint32_t DIER;		// DMA/interrupt enable register							Address offset: 0x0C */
	__vo uint32_t SR;		// status register											Address offset: 0x10 */
	__vo uint32_t EGR;		// event generation register								Address offset: 0x14 */
	__vo uint32_t CCMR1;	// capture/compare mode register 1							Address offset: 0x18 */
	__vo uint32_t CCMR2;	// capture/compare mode register 2							Address offset: 0x1C */
	__vo uint32_t CCER;		// capture/compare enable register							Address offset: 0x20 */
	__vo uint32_t CNT;		//counter register											Address offset: 0x24 */
	__vo uint32_t PSC;		// prescalar register										Address offset: 0x28 */
	__vo uint32_t ARR;		// auto-reload register										Address offset: 0x2C */
	uint32_t Reserved1;		// reserved													Address offset: 0x30 */
	__vo uint32_t CCR1;		// capture/compare register 1								Address offset: 0x34 */
	__vo uint32_t CCR2;		// capture/compare register 2								Address offset: 0x38 */
	__vo uint32_t CCR3;		// capture/compare register 3								Address offset: 0x3C */
	__vo uint32_t CCR4;		// capture/compare register 4								Address offset: 0x40 */
	uint32_t Reserved2;		// reserved													Address offset: 0x44 */
	__vo uint32_t DCR;		// DMA control register										Address offset: 0x48 */
	__vo uint32_t DMAR;		// DMA address for full transfer							Address offset: 0x4C */
	__vo uint32_t OR;		// option register											Address offset: 0x50 */
}Tim2_RegDef_t;

/*
 * peripheral register definition structure for Timer 9 and Timer 12
 */

typedef struct{
	__vo uint32_t CR1;		// control register 1									    Address offset: 0x00 */
	uint32_t Reserved1;		// reserved													Address offset: 0x04 */
	__vo uint32_t SMCR;		// slave mode control register								Address offset: 0x08 */
	__vo uint32_t DIER;		// DMA/interrupt enable register							Address offset: 0x0C */
	__vo uint32_t SR;		// status register											Address offset: 0x10 */
	__vo uint32_t EGR;		// event generation register								Address offset: 0x14 */
	__vo uint32_t CCMR1;	// capture/compare mode register 1							Address offset: 0x18 */
	uint32_t Reserved2;		// reserved													Address offset: 0x1C */
	__vo uint32_t CCER;		// capture/compare enable register							Address offset: 0x20 */
	__vo uint32_t CNT;		//counter register											Address offset: 0x24 */
	__vo uint32_t PSC;		// prescalar register										Address offset: 0x28 */
	__vo uint32_t ARR;		// auto-reload register										Address offset: 0x2C */
	uint32_t Reserved3;		// reserved													Address offset: 0x30 */
	__vo uint32_t CCR1;		// capture/compare register 1								Address offset: 0x34 */
	__vo uint32_t CCR2;		// capture/compare register 2								Address offset: 0x38 */
}Tim9_RegDef_t;

/*
 * peripheral register definition structure for Timer 10, 11, 13 and 14
 */

typedef struct{
	__vo uint32_t CR1;		// control register 1									    Address offset: 0x00 */
	uint32_t Reserved1;		// reserved													Address offset: 0x04 */
	__vo uint32_t SMCR;		// slave mode control register								Address offset: 0x08 */
	__vo uint32_t DIER;		// DMA/interrupt enable register							Address offset: 0x0C */
	__vo uint32_t SR;		// status register											Address offset: 0x10 */
	__vo uint32_t EGR;		// event generation register								Address offset: 0x14 */
	__vo uint32_t CCMR1;	// capture/compare mode register 1							Address offset: 0x18 */
	uint32_t Reserved2;		// reserved													Address offset: 0x1C */
	__vo uint32_t CCER;		// capture/compare enable register							Address offset: 0x20 */
	__vo uint32_t CNT;		//counter register											Address offset: 0x24 */
	__vo uint32_t PSC;		// prescalar register										Address offset: 0x28 */
	__vo uint32_t ARR;		// auto-reload register										Address offset: 0x2C */
	uint32_t Reserved3;		// reserved													Address offset: 0x30 */
	__vo uint32_t CCR1;		// capture/compare register 1								Address offset: 0x34 */
	uint32_t Reserved4[6];  // reserved													Address offset: 0x38-0x4C */
	__vo uint32_t OR;		// option register 											Address offset: 0x50 */
}Tim10_RegDef_t;


/*
 * peripheral register definition structure for Timer 6 and 7
 */

typedef struct{
	__vo uint32_t CR1;		// control register 1										Address offset: 0x00 */
	__vo uint32_t CR2;		// control register 2										Address offset: 0x04 */
	uint32_t reserved1;		// reserved													Address offset: 0x08 */
	__vo uint32_t DIER;		// DMA/interrupt enable register							Address offset: 0x0C */
	__vo uint32_t SR;		// status register											Address offset: 0x10 */
	__vo uint32_t EGR;		// event generation register								Address offset: 0x14 */
	uint32_t reserved2[3];	// reserved													Address offset: 0x18-20 */
	__vo uint32_t CNT;		//counter register											Address offset: 0x24 */
	__vo uint32_t PSC;		// prescalar register										Address offset: 0x28 */
	__vo uint32_t ARR;		// auto-reload register										Address offset: 0x2C */
}Tim6_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)

#define ADC1				((ADC_RegDef_t*)ADC1_BASEADDR)
#define ADC2				((ADC_RegDef_t*)ADC2_BASEADDR)
#define ADC3				((ADC_RegDef_t*)ADC3_BASEADDR)

#define SYSTICK				((SysTick_RegDef_t*)SYSTICK_BASEADDR)

#define TIM1				((Tim1_RegDef_t*)TIM1_BASEADDR)
#define TIM8				((Tim1_RegDef_t*)TIM8_BASEADDR)

#define TIM2				((Tim2_RegDef_t*)TIM2_BASEADDR)
#define TIM3				((Tim2_RegDef_t*)TIM3_BASEADDR)
#define TIM4				((Tim2_RegDef_t*)TIM4_BASEADDR)
#define TIM5				((Tim2_RegDef_t*)TIM5_BASEADDR)

#define TIM6				((Tim6_RegDef_t*)TIM6_BASEADDR)
#define TIM7				((Tim6_RegDef_t*)TIM7_BASEADDR)

#define TIM9				((Tim9_RegDef_t*)TIM9_BASEADDR)
#define TIM12				((Tim9_RegDef_t*)TIM12_BASEADDR)

#define TIM10				((Tim10_RegDef_t*)TIM10_BASEADDR)
#define TIM11				((Tim10_RegDef_t*)TIM11_BASEADDR)
#define TIM13				((Tim10_RegDef_t*)TIM13_BASEADDR)
#define TIM14				((Tim10_RegDef_t*)TIM14_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripheralsbu
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() (RCC->APB1ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))
/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB1ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))
/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)


/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

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
#define IRQ_NO_I2C1_EV  	31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15



/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9



#define SET			1
#define RESET		0
#define HIGH		SET
#define LOW			RESET
#define ENABLE		HIGH
#define DISABLE		LOW
#define FLAG_RESET 	DISABLE
#define FLAG_SET	ENABLE

#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx_i2c_driver.h"
#include "stm32f4xx_spi_driver.h"
#include "stm32f4xx_usart_driver.h"
#include "stm32f4xx_rcc_driver.h"

#endif /* STM32F4XX_H_ */
