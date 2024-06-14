#include "stm32f4xx.h"

void delay()
{
	for(uint32_t i=0;i<50000;i++);		// simple delay
}

int key_hit(uint32_t *pgpioaidr, uint32_t *pgpioaodr)
{
	// Set all row pins high initially
	*pgpioaodr |= (1 << 0) | (1 << 1) | (1 << 6) | (1 << 7);

	for(int row=0;row<4;row++)
	{
		// set current row low
		if(row < 2)
			*pgpioaodr &= ~(1 << row);
		else
			*pgpioaodr &= ~(1 << (row + 4));

		delay();	// Small delay to stabilize

		// Check each column
		if (!(*pgpioaidr & (1 << 8)))  return row * 4 + 1;	// PA8
		if (!(*pgpioaidr & (1 << 9)))  return row * 4 + 2;	// PA9
		if (!(*pgpioaidr & (1 << 10))) return row * 4 + 3;	// PA10
		if (!(*pgpioaidr & (1 << 11))) return row * 4 + 4;	// PA11

		// Reset the current row to high
		if (row < 2)
		    *pgpioaodr |= (1 << row);
		else
		    *pgpioaodr |= (1 << (row + 4));
	}
	return 0;
}
void LCD_PrintData(unsigned char data, uint32_t *pgpiocodr)
{
	if((data & 0x01)==0x01)
		*pgpiocodr |= (1 << 0);
	else
		*pgpiocodr &= ~(1 << 0);

	if((data & 0x02)==0x02)
		*pgpiocodr |= (1 << 1);
	else
		*pgpiocodr &= ~(1 << 1);

	if((data & 0x04)==0x04)
		*pgpiocodr |= (1 << 2);
	else
		*pgpiocodr &= ~(1 << 2);

	if((data & 0x08)==0x08)
		*pgpiocodr |= (1 << 3);
	else
		*pgpiocodr &= ~(1 << 3);

	if((data & 0x10)==0x10)
		*pgpiocodr |= (1 << 4);
	else
		*pgpiocodr &= ~(1 << 4);

	if((data & 0x20)==0x20)
		*pgpiocodr |= (1 << 5);
	else
		*pgpiocodr &= ~(1 << 5);

	if((data & 0x40)==0x40)
		*pgpiocodr |= (1 << 6);
	else
		*pgpiocodr &= ~(1 << 6);

	if((data & 0x80)==0x80)
		*pgpiocodr |= (1 << 7);
	else
		*pgpiocodr &= ~(1 << 7);

}

void LCD_Data(unsigned char data, uint32_t *pgpiocodr)
{
	LCD_PrintData(data, pgpiocodr);
	*pgpiocodr |=  (1<<8);			// make RS high
	*pgpiocodr &= ~(1<<9);			// make RnW low
	*pgpiocodr |=  (1<<10);			// make Enable high
	delay();
	*pgpiocodr &= ~(1<<10);			// make Enable low
	delay();
}

void LCD_Cmd(unsigned char cmd, uint32_t *pgpiocodr)
{
	LCD_PrintData(cmd, pgpiocodr);
	*pgpiocodr &= ~(1<<8);			// make RS low
	*pgpiocodr &= ~(1<<9);			// make RnW low
	*pgpiocodr |=  (1<<10);			// make Enable high
	delay();
	*pgpiocodr &= ~(1<<10);			// make Enable low
	delay();
}

void LCD_String(char *str, unsigned char length, uint32_t *pgpiocodr)
{
	for(uint32_t i=0;i<length;i++)
	{
		LCD_Data(str[i], pgpiocodr);
	}
}

void LCD_Init(uint32_t *pgpiocodr)
{
	LCD_Cmd(0x02, pgpiocodr);
	LCD_Cmd(0x38, pgpiocodr);
	LCD_Cmd(0x0C, pgpiocodr);
	LCD_Cmd(0x0F, pgpiocodr);
	LCD_Cmd(0x01, pgpiocodr);
	LCD_Cmd(0x06, pgpiocodr);
}

int main()
{
	// Enable clock for GPIO Port A, B and C
	GPIO_Pclk_Ctrl(GPIOA, ENABLE);
	GPIO_Pclk_Ctrl(GPIOC, ENABLE);

	// Initialize GPIO Port C for LCD Output
	uint32_t *pgpiocmodereg 	= (uint32_t*)0x40020800U;
	uint32_t *pgpiocodr		    = (uint32_t*)0x40020814U;

	*pgpiocmodereg &= ~(0x3FFFFF<<0);
	*pgpiocmodereg |=  (0x155555<<0);		// setting mode register

	// Initialize GPIO Port A and Port B for keypad input
	uint32_t *pgpioamodereg 	= (uint32_t*)0x40020000U;
	uint32_t *pgpioaodr		    = (uint32_t*)0x40020014U;
	uint32_t *pgpioaidr		    = (uint32_t*)0x40020010U;
	uint32_t *pgpioapupd	    = (uint32_t*)0x4002000CU;

	/* keypad interface
	 * Rows - PA0, PA1, PA6, PA7
	 * Columns - PA8, PA9, PA10, PA11
	 */

	// setting the rows
		*pgpioamodereg &= ~((0x3<<0) | (0x3<<2) | (0x3<<12) | (0x3<<14));
		*pgpioamodereg |=  ((0x1<<0) | (0x1<<2) | (0x1<<12) | (0x1<<14));

	// setting the columns
		*pgpioamodereg &= ~((0x3<<16) | (0x3<<18)| (0x3<<20)| (0x3<<22));
		*pgpioamodereg |=  ((0x1<<16) | (0x1<<18)| (0x1<<20)| (0x1<<22));

	// setting the pull up resistor on input columns
		*pgpioapupd &= ~((0xF<<8)|(0xF<<9)|(0xF<<10)|(0xF<<11));
		*pgpioapupd |=  ((0x1<<8)|(0x1<<9)|(0x1<<10)|(0x1<<11));

	LCD_Init(pgpiocodr);

	while(1)
	{
		int key = key_hit(pgpioaidr, pgpioaodr);
		delay();

		switch(key)
		{
		case 1:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("one", 3, pgpiocodr);
			break;

		case 2:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("two", 3, pgpiocodr);
			break;

		case 3:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("three", 5, pgpiocodr);
			break;

		case 4:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("A", 1, pgpiocodr);
			break;

		case 5:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("four", 4, pgpiocodr);
			break;
		case 6:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("five",4, pgpiocodr);
			break;
		case 7:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("six", 3, pgpiocodr);
			break;
		case 8:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("B", 1, pgpiocodr);
			break;

		case 9:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("seven",5, pgpiocodr);
			break;

		case 10:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("eight", 5, pgpiocodr);
			break;

		case 11:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("nine", 4, pgpiocodr);
			break;

		case 12:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("C", 1, pgpiocodr);
			break;

		case 13:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("*", 1, pgpiocodr);
			break;

		case 14:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("zero", 1, pgpiocodr);
			break;

		case 15:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("#", 1, pgpiocodr);
			break;

		case 16:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("D", 1, pgpiocodr);
			break;
		}
	}
	return 0;
}
