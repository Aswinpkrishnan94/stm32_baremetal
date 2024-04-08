/*
 * 001_led_blink.c
 * Hardware: STm32 f446re Nucleo Board
 * Description: The program below blinks on-board led
 * Created on: Feb 22, 2024
 * Author: Aswin P Krishnan
 */

#include <stdint.h>

int main(void)
{
// Initialize RCC Register. The GPIO ports are connected to AHB1 Bus.
uint32_t *pclkctrlreg = (uint32_t*)0x40023830U;

// Initialize GPIO Port Registers
uint32_t *pGPIOAmodereg = (uint32_t*)0x40020000U;	// GPIO Port A Mode Register
uint32_t *popdatareg 	= (uint32_t*)0x40020014U;	// GPIO Port A Output Data Register

*pclkctrlreg   |= 1<<0; 							// Clock Enable
*pGPIOAmodereg &=~(3<<10);							// Reset Register
*pGPIOAmodereg |= 1<<10;							// Set bits in the register

// The on-board led is connected to Port A pin 5. Refer Schematic diagram.
*popdatareg |= 1<<5;

while(1);											// To keep program running
}
