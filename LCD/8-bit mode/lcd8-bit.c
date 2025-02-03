#include "stm32f4xx.h"


void PrintData(unsigned char data, uint32_t *popdatareg);


void delay()
{
for(uint32_t i=0;i<80000;i++);
}

void PrintData(unsigned char data, uint32_t *popdatareg)
{
	// zero bit - d0
	if((data&0x01) == 0x01)		// turn on bit zero pin
	{
		*popdatareg |= (1 << 0);
	}
	else 						// turn off bit zero pin
	{
		*popdatareg &= ~(1 << 0);
	}

	// first bit - d1
	if((data&0x02) == 0x02)
	{
		*popdatareg |= (1 << 1);
	}
	else
	{
		*popdatareg &= ~(1 << 1);
	}

	// second bit - d2
	if((data&0x04) == 0x04)
	{
		*popdatareg |= (1 << 2);
	}
	else
	{
		*popdatareg &= ~(1 << 2);
	}

	// third bit - d3
	if((data&0x08) == 0x08)
	{
		*popdatareg |= (1 << 3);
	}
	else
	{
		*popdatareg &= ~(1 << 3);
	}

	// fourth bit - d4
	if((data&0x10) == 0x10)
	{
		*popdatareg |= (1 << 4);
	}
	else
	{
		*popdatareg &= ~(1 << 4);
	}

	//fifth bit - d5
	if((data&0x20) == 0x20)
	{
		*popdatareg |= (1 << 5);
	}
	else
	{
		*popdatareg &= ~(1 << 5);
	}

	//sixth bit - d6
	if((data&0x40) == 0x40)
	{
		*popdatareg |= (1 << 6);
	}
	else
	{
		*popdatareg &= ~(1 << 6);
	}

	// seventh bit - d7
	if((data&0x80) == 0x80)
	{
		*popdatareg |= (1 << 7);
	}
	else
	{
		*popdatareg &= ~(1 << 7);
	}

}

// Passing Data into LCD
void LCD_Data(unsigned char data, uint32_t *popdatareg)
{
	// Pass Data into data lines
	PrintData(data, popdatareg);

	// Write R/S high
	*popdatareg |= (1 << 8);

	// Write RW low for write
	*popdatareg &= ~(1 << 9);

	// Make Enable High
	*popdatareg |= (1 << 10);

	// Delay
	delay();

	// Make Enable Low
	*popdatareg &= ~(1 << 10);
	delay();
}

// Pass Data into instruction register of LCD
// Pass Commands into LCD
void LCD_Cmd(unsigned char cmd, uint32_t *popdatareg)
{

	// Pass Data into data lines
	PrintData(cmd, popdatareg);

	// Write R/nW low for write
	*popdatareg &= ~(1 << 9);

	// Write R/S low
	*popdatareg &= ~(1 << 8);

	// Make Enable High
	*popdatareg |= (1 << 10);

	// Delay
	delay();

	// Make Enable Low
	*popdatareg &= ~(1 << 10);
	delay();
}

// To print a string or word
void LCD_String(char *str, unsigned char length, uint32_t *popdatareg)
{
	unsigned char i =0;

	for(i=0; i<length;i++)
	{
		LCD_Data(str[i], popdatareg);
	}
}

// Initialize LCD
void LCD_Init(uint32_t *popdatareg)
{
	// LCD Data function to print string or word
	LCD_Cmd(0x02, popdatareg);
	LCD_Cmd(0x38, popdatareg);		//
	LCD_Cmd(0x0C, popdatareg);		//
	LCD_Cmd(0x0F, popdatareg);		//
	LCD_Cmd(0x01, popdatareg);		//
	LCD_Cmd(0x06, popdatareg);		//




}

int main(void)
{
	uint32_t *pclkctrlreg = (uint32_t*)0x40023830;

	uint32_t *pGPIOC_ModeReg = (uint32_t*)0x40020800U;
	uint32_t *popdatareg     = (uint32_t*)0x40020814U;

	*pclkctrlreg |= (1 << 2);

	*pGPIOC_ModeReg &= ~(0xFFFF<<0)|(0x3F<<16);
	*pGPIOC_ModeReg |=  (0x5555<<0)|(0x15<<16);

	LCD_Init(popdatareg);
	LCD_Cmd(0x80, popdatareg);  //Move cursor into first row first column
	LCD_String("HELLO", 5, popdatareg);
	LCD_Cmd(0xC0, popdatareg);  //Move cursor into second row first column
	delay();
	LCD_String("WORLD", 5, popdatareg);

	while(1);
	return 0;
