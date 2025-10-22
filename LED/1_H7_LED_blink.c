#include "main.h"
#include "stm32h7xx.h"

void delay(int ms)
{
	for(uint32_t i=0;i<ms;i++)
	{
		for(uint32_t j=0;j<2000;j++);
	}
}
int main(void)
{
	volatile uint32_t *pclkctrlreg = (uint32_t*)0x580244E0;
	volatile uint32_t *pgpiobmodereg = (uint32_t*)0x58020400;
	volatile uint32_t *pgpiobodrreg = (uint32_t*)0x58020414;

	*pclkctrlreg |= (1 << 1);

	*pgpiobmodereg &= ~(3 << 0);
	*pgpiobmodereg |= (1 << 0);

	while(1)
	{
		*pgpiobodrreg |= (1 << 0);
		delay(5000);
		*pgpiobodrreg &= ~(1 << 0);
		delay(5000);
	}
	return 0;
}
