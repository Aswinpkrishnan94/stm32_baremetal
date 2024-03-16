#include "stm32f4xx.h"

void PrintData(unsigned char data, uint32_t *popdatareg);

void delay()
{
for(uint32_t i=0;i<500000;i++);
}

int main(void)
{
	uint32_t *pclkctrlreg 	 	 = (uint32_t*)0x40023830U;

	uint32_t *pGPIOC_ModeReg	 = (uint32_t*)0x40020800U;
	uint32_t *popdatareg 		 = (uint32_t*)0x40020814U;

	*pclkctrlreg |= (1<<1);
	*pclkctrlreg |= (1<<2);

	*pGPIOC_ModeReg &= ~(0xFFFF<<0);

	*pGPIOC_ModeReg |=  (0x5555<<0);

	while(1)
	{
		PrintData(0x3F, popdatareg);		// Number 0
		delay();
		PrintData(0x06, popdatareg);		// Number 1
		delay();
		PrintData(0x5B, popdatareg);		// Number 2
		delay();
		PrintData(0x4F, popdatareg);		// Number 3
		delay();
		PrintData(0x66, popdatareg);		// Number 4
		delay();
		PrintData(0x6D, popdatareg);		// Number 5
		delay();
		PrintData(0x7D, popdatareg);		// Number 6
		delay();
		PrintData(0x07, popdatareg);		// Number 7
		delay();
		PrintData(0x7F, popdatareg);		// Number 8
		delay();
		PrintData(0x6F, popdatareg);		// Number 9
		delay();
	}

	return 0;
}

void PrintData(unsigned char data, uint32_t *popdatareg)
{
	// zero bit - a
	if((data&0x01) == 0x01)		        
	{
		*popdatareg |= (1 << 0);        // turn on bit 0 pin
	}
	else 						                
	{
		*popdatareg &= ~(1 << 0);      // turn off bit 0 pin
	}

	// first bit - b
	if((data&0x02) == 0x02)           
	{
		*popdatareg |= (1 << 1);       // turn on bit 1 pin
	}
	else
	{
		*popdatareg &= ~(1 << 1);     // turn off bit 1 pin
	}

	// second bit - c
	if((data&0x04) == 0x04)
	{
		*popdatareg |= (1 << 2);       // turn on bit 2 pin
	}
	else
	{
		*popdatareg &= ~(1 << 2);     // turn off bit 2 pin
	}

	// third bit - d
	if((data&0x08) == 0x08)
	{
		*popdatareg |= (1 << 3);       // turn on bit 3 pin
	}
	else
	{
		*popdatareg &= ~(1 << 3);     // turn off bit 3 pin
	}

	// fourth bit - e
	if((data&0x10) == 0x10)
	{
		*popdatareg |= (1 << 4);        // turn on bit 4 pin
	}
	else
	{
		*popdatareg &= ~(1 << 4);      // turn off bit 4 pin
	}

	//fifth bit - f
	if((data&0x20) == 0x20)
	{
		*popdatareg |= (1 << 5);        // turn on bit 5 pin
	}
	else
	{
		*popdatareg &= ~(1 << 5);       // turn off bit 5 pin
	}

	//sixth bit - g
	if((data&0x40) == 0x40)
	{
		*popdatareg |= (1 << 6);        // turn on bit 6 pin
	}
	else
	{
		*popdatareg &= ~(1 << 6);      // turn off bit 6 pin
	}

	// seventh bit - h
	if((data&0x80) == 0x80)
	{
		*popdatareg |= (1 << 7);        // turn on bit 7 pin
	}
	else
	{
		*popdatareg &= ~(1 << 7);       // turn off bit 7 pin
	}

}
