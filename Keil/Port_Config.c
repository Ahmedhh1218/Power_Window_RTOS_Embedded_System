#include "Port_Config.h"

void PortA_Config(void) {
    // Enable the clock of PORTA
    SYSCTL_RCGCGPIO_R |= 0x01;
    
    // Configure pins 2, 3, 4, and 5 as inputs and 6 and 7 as outputs
		GPIO_PORTA_DIR_R |= 0xC0;
    GPIO_PORTA_DIR_R &= ~(0x3C);
    
    // Enable digital functionality for pins 2, 3, 4, 5, 6 and 7
    GPIO_PORTA_DEN_R |= 0xFC;
}


void PortC_Config(void) {
    // Enable the clock of PORTC
    SYSCTL_RCGCGPIO_R |= 0x04;
    
    // Configure pins 4, 5, 6, and 7 as inputs
    GPIO_PORTC_DIR_R &= ~(0xF0);
    
    // Enable digital functionality for pins 4, 5, 6, and 7
    GPIO_PORTC_DEN_R |= 0xF0;
}

void PortF_Config(void) {
    // Enable the clock of PORTF
    SYSCTL_RCGCGPIO_R |= 0x20;
    
    // Unlock PORTF
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R = 0x1F;
    
    // Configure pins 0 and 4 as inputs, and pins 1, 2, and 3 as outputs
    GPIO_PORTF_DIR_R = 0x0E;
    
    // Enable digital functionality for all pins
    GPIO_PORTF_DEN_R |= 0x1F;
}

