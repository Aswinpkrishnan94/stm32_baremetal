#include "stm32f4xx.h"

void delay()
{
	for(uint32_t i=0;i<20000;i++);		// simple delay
}

int key_hit(uint32_t *pgpioaidr, uint32_t *pgpiobidr, uint32_t *pgpioaodr)
{
	// Set all row pins high initially
	*pgpioaodr |= (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);

    for (int row = 0; row < 4; row++)
    {
		// set current row low
		if(row < 2)
			*pgpioaodr &= ~(1 << row);
		else
			*pgpioaodr &= ~(1 << (row + 2));

		delay();	// Small delay to stabilize

		// Check each column
		if (!(*pgpiobidr & (1 << 0))) return row * 4 + 1;	// PB0
		if (!(*pgpiobidr & (1 << 1))) return row * 4 + 2;	// PB1
		if (!(*pgpioaidr & (1 << 6))) return row * 4 + 3;	// PA6
		if (!(*pgpioaidr & (1 << 7))) return row * 4 + 4;	// PA7

		// Reset the current row to high
		if (row < 2)
		    *pgpioaodr |= (1 << row);
		else
		    *pgpioaodr |= (1 << (row + 2));
	}
           return 0;   // No key pressed
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


void LCD_String(char *str, uint32_t *pgpiocodr)
{
	while (*str)
	{
	  LCD_Data(*str++, pgpiocodr);
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
	GPIO_Pclk_Ctrl(GPIOB, ENABLE);
	GPIO_Pclk_Ctrl(GPIOC, ENABLE);

	// Initialize GPIO Port C for LCD Output
	uint32_t *pgpiocmodereg 	= (uint32_t*)0x40020800U;
	uint32_t *pgpiocodr		= (uint32_t*)0x40020814U;

	*pgpiocmodereg &= ~(0x3FFFFF<<0);
	*pgpiocmodereg |=  (0x155555<<0);		// setting mode register

	// Initialize GPIO Port A and Port B for keypad input
	uint32_t *pgpioamodereg 	= (uint32_t*)0x40020000U;
	uint32_t *pgpioaodr		= (uint32_t*)0x40020014U;
	uint32_t *pgpioaidr		= (uint32_t*)0x40020010U;
	uint32_t *pgpioapupdr	 	= (uint32_t*)0x4002000CU;

	// GPIO Port B Registers
	uint32_t *pgpiobmodereg   = (uint32_t *)0x40020400U;
	uint32_t *pgpiobidr   	  = (uint32_t *)0x40020410U;
	uint32_t *pgpiobpupdr     = (uint32_t *)0x4002040CU;


	/* keypad interface
	 * Rows - PA0, PA1, PA4, PA5
	 * Columns - PB0, PB1, PA6, PA7
	 */


	// Configure GPIOA pins 0, 1, 4, 5 as output (rows)
	    *pgpioamodereg &= ~((0x3 << 0) | (0x3 << 2) | (0x3 << 8) | (0x3 << 10));  // Clear mode bits for PA0, PA1, PA4, PA5
	    *pgpioamodereg |=  ((0x1 << 0) | (0x1 << 2) | (0x1 << 8) | (0x1 << 10));   // Set PA0, PA1, PA4, PA5 as output (01)

    // Configure GPIOA pins 6, 7 as input (columns)
	    *pgpioamodereg &= ~((0x3 << 12) | (0x3 << 14));  // Clear mode bits for PA6, PA7
	    *pgpioamodereg |=  ((0x0 << 12) | (0x0 << 14));   // Set PA6, PA7 as input (00)

    // Enable pull-up resistors for PA6, PA7
	    *pgpioapupdr &= ~((0x3 << 12) | (0x3 << 14));    // Clear pull-up/pull-down bits for PA6, PA7
	    *pgpioapupdr |= ((0x1 << 12) | (0x1 << 14));     // Set PA6, PA7 as pull-up (01)

    // Configure GPIOB pins 0, 1 as input (columns)
	    *pgpiobmodereg &= ~((0x3 << 0) | (0x3 << 2));   // Clear mode bits for PB0, PB1
	    *pgpiobmodereg |= ((0x0 << 0) | (0x0 << 2));    // Set PB0, PB1 as input (00)

    // Enable pull-up resistors for PB0, PB1
	    *pgpiobpupdr &= ~((0x3 << 0) | (0x3 << 2));     // Clear pull-up/pull-down bits for PB0, PB1
	    *pgpiobpupdr |= ((0x1 << 0) | (0x1 << 2));      // Set PB0, PB1 as pull-up (01)

	LCD_Init(pgpiocodr);

	while(1)
	{
		int key = key_hit(pgpioaidr, pgpiobidr, pgpioaodr);
		delay();

		switch(key)
		{
		case 1:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("one",pgpiocodr);
			delay();
			break;

		case 2:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("two",pgpiocodr);
			delay();
			break;

		case 3:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("three",pgpiocodr);
			delay();
			break;

		case 4:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("A",pgpiocodr);
			delay();
			break;

		case 5:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("four",pgpiocodr);
			delay();
			break;

		case 6:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("five",pgpiocodr);
			delay();
			break;

		case 7:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("six",pgpiocodr);
			delay();
			break;

		case 8:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("B",pgpiocodr);
			delay();
			break;

		case 9:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("seven",pgpiocodr);
			delay();
			break;

		case 10:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("eight",pgpiocodr);
			delay();
			break;

		case 11:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("nine",pgpiocodr);
			delay();
			break;

		case 12:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("C",pgpiocodr);
			delay();
			break;

		case 13:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("*",pgpiocodr);
			break;

		case 14:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("zero",pgpiocodr);
			delay();
			break;

		case 15:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("#",pgpiocodr);
			delay();
			break;

		case 16:
			LCD_Cmd(0x80, pgpiocodr);
			LCD_String("D",pgpiocodr);
			delay();
			break;
		}
	}
	return 0;
}
