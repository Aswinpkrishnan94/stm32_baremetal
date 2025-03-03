#include<stdint.h>
#include<stdio.h>

void GPIO_Init();
void Timer2_Init();

#define __vo			volatile
#define APB1ENR_BA		0x40000000

typedef struct
{
	__vo uint32_t timer2_cr1;
	__vo uint32_t timer2_cr2;
	__vo uint32_t timer2_smcr;
	__vo uint32_t timer2_dier;
	__vo uint32_t timer2_sr;
	__vo uint32_t timer2_egr;
	__vo uint32_t timer2_ccmr1;
	__vo uint32_t timer2_ccmr2;
	__vo uint32_t timer2_ccer;
	__vo uint32_t timer2_cnt;
	__vo uint32_t timer2_psc;
	__vo uint32_t timer2_arr;
		 uint32_t Reserved1;
	__vo uint32_t timer2_ccr1;
	__vo uint32_t timer2_ccr2;
	__vo uint32_t timer2_ccr3;
	__vo uint32_t timer2_ccr4;
		 uint32_t Reserved2;
	__vo uint32_t timer2_dcr;
	__vo uint32_t timer2_dmar;
	__vo uint32_t timer2_or1;
	__vo uint32_t timer2_or2;
}timer2_RegDef_t;

#define TIM2			((timer2_RegDef_t*)APB1ENR_BA)

void GPIO_Init()
{
	volatile uint32_t *pgpioaclock = (uint32_t*)0x40023830;			// RCC->AHB1ENR
	volatile uint32_t *pgpioamode  = (uint32_t*)0x40020000;			// GPIOA->Mode
	volatile uint32_t *pgpioaafrl  = (uint32_t*)0x40020020;			// GPIOA->AFRL

	*pgpioaclock |= (1<<0);			// AHB1ENR -> GPIOA

	*pgpioamode  &= ~(3<<0);
	*pgpioamode  |= (2<<0);			// PA0 = Alternate Function Mode

	*pgpioaafrl  &= ~(1<<0);
	*pgpioaafrl  |=  (1<<0);		// PA0 = AF1 (Timer 2 Ch1)
}

void Timer2_Init()
{
	volatile uint32_t *ptimer2clock = (uint32_t*)0x40000040;

	*ptimer2clock |= (1<<0);

	TIM2->timer2_ccmr1 |= (1<<0);	// CC1 channel - input. IC1 is mapped on TI1
	TIM2->timer2_ccer  |= (1<<0);	// Input capture enabled
	TIM2->timer2_psc    = 89;		// 1 MHz timer clock (1 µs per tick)
	TIM2->timer2_arr    = 0xFFFF;	// maximum time period
	TIM2->timer2_cr1   |= (1<<0);	// enable timer 2

}

int main()
{
	GPIO_Init();
	Timer2_Init();

	while(1)
	{
		// Read captured value (CCR1) to get Hall sensor timing
		while(TIM2->timer2_ccr1);
		uint32_t pulse_time = TIM2->timer2_ccr1;
		printf("Pulse Time: %lu\n", pulse_time);
	}
	return 0;
}
