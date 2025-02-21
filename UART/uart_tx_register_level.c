#include <stdint.h>


void delay()
{
    for (volatile int i = 0; i < 800000; i++);  // Increased for better visibility
}

void usart_init()
{
    volatile uint32_t *usartbr    = (uint32_t*)0x40004408;  // USART Baud Rate
    volatile uint32_t *usartcr1   = (uint32_t*)0x4000440C;  // USART Control Register 1
    volatile uint32_t *usartcr2   = (uint32_t*)0x40004410;  // USART Control Register 2
    volatile uint32_t *usartcr3   = (uint32_t*)0x40004414;  // USART Control Register 3

    *usartbr = 0x683;  // Baud Rate: 9600 @16MHz

    *usartcr1 &= ~(1 << 15);  // Ensure 8-bit data mode
    *usartcr1 |= (1 << 3) | (1 << 13);  // Enable Transmitter & USART

    *usartcr2 &= ~(3 << 12);  // 1 Stop Bit (default)
}

void usart_write(char c)
{
    volatile uint32_t *usartsr = (uint32_t*)0x40004400;  // USART Status Register
    volatile uint32_t *usartdr = (uint32_t*)0x40004404;  // USART Data Register

    while (!(*usartsr & (1 << 7)));  // Wait until TX buffer is empty
    *usartdr = c & 0xFF;
}

int main(void)
{
    volatile uint32_t *pahb1clk   = (uint32_t*)0x40023830;  // RCC AHB1ENR (GPIOA Clock)
    volatile uint32_t *papb1clk   = (uint32_t*)0x40023840;  // RCC APB1ENR (USART2 Clock)
    volatile uint32_t *pgpioamode = (uint32_t*)0x40020000;  // GPIOA Mode Register
    volatile uint32_t *pgpioafr   = (uint32_t*)0x40020020;  // GPIOA Alternate Function Register

    *pahb1clk |= (1 << 0);   // Enable GPIOA clock
    *papb1clk |= (1 << 17);  // Enable USART2 clock

    // Configure PA2 as Alternate Function (USART2_TX)
    *pgpioamode &= ~(3 << 4);
    *pgpioamode |= (2 << 4);

    // Set PA2 to AF7 (USART2_TX)
    *pgpioafr &= ~(0xF << 8);
    *pgpioafr |= (7 << 8);

    usart_init();

    while (1)
    {
        usart_write('Y');
        usart_write('E');
        usart_write('S');
        delay();
    }
}
