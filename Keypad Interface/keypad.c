#include "stm32f4xx.h"

void delay()
{
	for(uint32_t i =0;i<50000;i++);
}

int key_hit(uint32_t *pgpioaidr, uint32_t *pgpiobidr, uint32_t *pgpioaodr)
{
	// Set all row pins high initially
	*pgpioaodr |= (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5);

	for(int row=0;row<4;row++)
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
	return 0;
}
int main(void)
{
	// Clock Enable
	GPIO_Pclk_Ctrl(GPIOA, ENABLE);
	GPIO_Pclk_Ctrl(GPIOB, ENABLE);

	// GPIO Port A Registers
	uint32_t *pgpioamodereg   = (uint32_t*)0x40020000U;
	uint32_t *pgpioaidr 	  = (uint32_t*)0x40020010U;
	uint32_t *pgpioaodr 	  = (uint32_t*)0x40020014U;
	uint32_t *pgpioapupdr     = (uint32_t*)0x4002000CU;

	// GPIO Port B Registers
	uint32_t *pgpiobmodereg   = (uint32_t *)0x40020400U;
	uint32_t *pgpiobidr   	  = (uint32_t *)0x40020410U;
	uint32_t *pgpiobpupdr     = (uint32_t *)0x4002040CU;



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

	while(1)
	{
		int key = key_hit(pgpioaidr, pgpiobidr, pgpioaodr);

		if(key)
		{
			delay();
			switch(key)
			{
							case 1: printf("Key 1 Pressed\n"); break;
							case 2: printf("Key 2 Pressed\n"); break;
							case 3: printf("Key 3 Pressed\n"); break;
							case 4: printf("Key A Pressed\n"); break;
							case 5: printf("Key 4 Pressed\n"); break;
							case 6: printf("Key 5 Pressed\n"); break;
							case 7: printf("Key 6 Pressed\n"); break;
							case 8: printf("Key B Pressed\n"); break;
							case 9: printf("Key 7 Pressed\n"); break;
							case 10: printf("Key 8 Pressed\n"); break;
							case 11: printf("Key 9 Pressed\n"); break;
							case 12: printf("Key C Pressed\n"); break;
							case 13: printf("Key * Pressed\n"); break;
							case 14: printf("Key 0 Pressed\n"); break;
							case 15: printf("Key # Pressed\n"); break;
							case 16: printf("Key D Pressed\n"); break;
							default: printf("Unknown Key Pressed\n"); break;
			}
		}
	}
	return 0;
}
