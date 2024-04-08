/*
 * 002_led_blink_button.c
 * Hardware: STm32 f446re Nucleo Board
 * Description: The program below turns on led when user-button is pressed
 * Created on: Feb 22, 2024
 * Author: Aswin P Krishnan
 */

#include <stdint.h>

int main(void)
{
// Initialize RCC Register. The GPIO ports are connected to AHB1 Bus.
uint32_t *pclkctrlreg = (uint32_t*)0x40023830U;

// Initialize GPIO Port A and Port C Registers.
uint32_t *pGPIOAmodereg = (uint32_t*)0x40020000U; 	// GPIO Port A Mode Register
uint32_t *pGPIOCmodereg = (uint32_t*)0x40020800U;	// GPIO Port C Mode Register
uint32_t *pipdatareg 	= (uint32_t*)0x40020810U;	// GPIO Port C Input Data Register
uint32_t *popdatareg 	= (uint32_t*)0x40020014U;	// GPIO Port A Output Data Register

*pclkctrlreg |= (1<<0); 							// Clock Enable for GPIO Port A
*pclkctrlreg |= (1<<2); 							// Clock Enable for GPIO Port C

*pGPIOAmodereg &=~(3<<10);
*pGPIOAmodereg |= (1<<10);							// Set GPIO Port A in Output Mode
*pGPIOCmodereg &=~(3<<26);							// Set GPIO Port C in Input Mode

while(1)
{
// The user-button is connected to GPIO Port C Pin 13. Refer Schematic.
uint8_t status = (*pipdatareg>>13)&0x1;				// The status of input data is checked

if(status==0)
*popdatareg |= 1<<5;
else
*popdatareg &=~1<<5;
}
}
