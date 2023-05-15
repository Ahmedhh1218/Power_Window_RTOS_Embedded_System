#ifndef __PORT_Configuration__
#define __PORT_Configuration__ 


#include "TM4C123GH6PM.h"
#define SYSCTL_RCGCGPIO_R       (*((volatile unsigned long *)0x400FE608))

/* PORTA Registers */
#define GPIO_PORTA_DATA_BITS_R  ((volatile uint32_t *)0x40004000)
#define GPIO_PORTA_DATA_R       (*((volatile uint32_t *)0x400043FC))
#define GPIO_PORTA_DIR_R        (*((volatile uint32_t *)0x40004400))
#define GPIO_PORTA_IS_R         (*((volatile uint32_t *)0x40004404))
#define GPIO_PORTA_IBE_R        (*((volatile uint32_t *)0x40004408))
#define GPIO_PORTA_IEV_R        (*((volatile uint32_t *)0x4000440C))
#define GPIO_PORTA_IM_R         (*((volatile uint32_t *)0x40004410))
#define GPIO_PORTA_RIS_R        (*((volatile uint32_t *)0x40004414))
#define GPIO_PORTA_MIS_R        (*((volatile uint32_t *)0x40004418))
#define GPIO_PORTA_ICR_R        (*((volatile uint32_t *)0x4000441C))
#define GPIO_PORTA_AFSEL_R      (*((volatile uint32_t *)0x40004420))
#define GPIO_PORTA_DR2R_R       (*((volatile uint32_t *)0x40004500))
#define GPIO_PORTA_DR4R_R       (*((volatile uint32_t *)0x40004504))
#define GPIO_PORTA_DR8R_R       (*((volatile uint32_t *)0x40004508))
#define GPIO_PORTA_ODR_R        (*((volatile uint32_t *)0x4000450C))
#define GPIO_PORTA_PUR_R        (*((volatile uint32_t *)0x40004510))
#define GPIO_PORTA_PDR_R        (*((volatile uint32_t *)0x40004514))
#define GPIO_PORTA_SLR_R        (*((volatile uint32_t *)0x40004518))
#define GPIO_PORTA_DEN_R        (*((volatile uint32_t *)0x4000451C))
#define GPIO_PORTA_LOCK_R       (*((volatile uint32_t *)0x40004520))
#define GPIO_PORTA_CR_R         (*((volatile uint32_t *)0x40004524))
#define GPIO_PORTA_AMSEL_R      (*((volatile uint32_t *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile uint32_t *)0x4000452C))
#define GPIO_PORTA_ADCCTL_R     (*((volatile uint32_t *)0x40004530))
#define GPIO_PORTA_DMACTL_R     (*((volatile uint32_t *)0x40004534))
	
/* PORTB Registers */
#define GPIO_PORTB_DATA_BITS_R  ((volatile uint32_t *)0x40005000)
#define GPIO_PORTB_DATA_R       (*((volatile uint32_t *)0x400053FC))
#define GPIO_PORTB_DIR_R        (*((volatile uint32_t *)0x40005400))
#define GPIO_PORTB_IS_R         (*((volatile uint32_t *)0x40005404))
#define GPIO_PORTB_IBE_R        (*((volatile uint32_t *)0x40005408))
#define GPIO_PORTB_IEV_R        (*((volatile uint32_t *)0x4000540C))
#define GPIO_PORTB_IM_R         (*((volatile uint32_t *)0x40005410))
#define GPIO_PORTB_RIS_R        (*((volatile uint32_t *)0x40005414))
#define GPIO_PORTB_MIS_R        (*((volatile uint32_t *)0x40005418))
#define GPIO_PORTB_ICR_R        (*((volatile uint32_t *)0x4000541C))
#define GPIO_PORTB_AFSEL_R      (*((volatile uint32_t *)0x40005420))
#define GPIO_PORTB_DR2R_R       (*((volatile uint32_t *)0x40005500))
#define GPIO_PORTB_DR4R_R       (*((volatile uint32_t *)0x40005504))
#define GPIO_PORTB_DR8R_R       (*((volatile uint32_t *)0x40005508))
#define GPIO_PORTB_ODR_R        (*((volatile uint32_t *)0x4000550C))
#define GPIO_PORTB_PUR_R        (*((volatile uint32_t *)0x40005510))
#define GPIO_PORTB_PDR_R        (*((volatile uint32_t *)0x40005514))
#define GPIO_PORTB_SLR_R        (*((volatile uint32_t *)0x40005518))
#define GPIO_PORTB_DEN_R        (*((volatile uint32_t *)0x4000551C))
#define GPIO_PORTB_LOCK_R       (*((volatile uint32_t *)0x40005520))
#define GPIO_PORTB_CR_R         (*((volatile uint32_t *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile uint32_t *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile uint32_t *)0x4000552C))
#define GPIO_PORTB_ADCCTL_R     (*((volatile uint32_t *)0x40005530))
#define GPIO_PORTB_DMACTL_R     (*((volatile uint32_t *)0x40005534))

	
/* PORTC Registers */
#define GPIO_PORTC_DATA_BITS_R  ((volatile uint32_t *)0x40006000)
#define GPIO_PORTC_DATA_R       (*((volatile uint32_t *)0x400063FC))
#define GPIO_PORTC_DIR_R        (*((volatile uint32_t *)0x40006400))
#define GPIO_PORTC_IS_R         (*((volatile uint32_t *)0x40006404))
#define GPIO_PORTC_IBE_R        (*((volatile uint32_t *)0x40006408))
#define GPIO_PORTC_IEV_R        (*((volatile uint32_t *)0x4000640C))
#define GPIO_PORTC_IM_R         (*((volatile uint32_t *)0x40006410))
#define GPIO_PORTC_RIS_R        (*((volatile uint32_t *)0x40006414))
#define GPIO_PORTC_MIS_R        (*((volatile uint32_t *)0x40006418))
#define GPIO_PORTC_ICR_R        (*((volatile uint32_t *)0x4000641C))
#define GPIO_PORTC_AFSEL_R      (*((volatile uint32_t *)0x40006420))
#define GPIO_PORTC_DR2R_R       (*((volatile uint32_t *)0x40006500))
#define GPIO_PORTC_DR4R_R       (*((volatile uint32_t *)0x40006504))
#define GPIO_PORTC_DR8R_R       (*((volatile uint32_t *)0x40006508))
#define GPIO_PORTC_ODR_R        (*((volatile uint32_t *)0x4000650C))
#define GPIO_PORTC_PUR_R        (*((volatile uint32_t *)0x40006510))
#define GPIO_PORTC_PDR_R        (*((volatile uint32_t *)0x40006514))
#define GPIO_PORTC_SLR_R        (*((volatile uint32_t *)0x40006518))
#define GPIO_PORTC_DEN_R        (*((volatile uint32_t *)0x4000651C))
#define GPIO_PORTC_LOCK_R       (*((volatile uint32_t *)0x40006520))
#define GPIO_PORTC_CR_R         (*((volatile uint32_t *)0x40006524))
#define GPIO_PORTC_AMSEL_R      (*((volatile uint32_t *)0x40006528))
#define GPIO_PORTC_PCTL_R       (*((volatile uint32_t *)0x4000652C))
#define GPIO_PORTC_ADCCTL_R     (*((volatile uint32_t *)0x40006530))
#define GPIO_PORTC_DMACTL_R     (*((volatile uint32_t *)0x40006534))


/* PORTF Registers */
#define GPIO_PORTF_DATA_BITS_R  ((volatile unsigned long *)0x40025000)
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_MIS_R        (*((volatile unsigned long *)0x40025418))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_DR2R_R       (*((volatile unsigned long *)0x40025500))
#define GPIO_PORTF_DR4R_R       (*((volatile unsigned long *)0x40025504))
#define GPIO_PORTF_DR8R_R       (*((volatile unsigned long *)0x40025508))
#define GPIO_PORTF_ODR_R        (*((volatile unsigned long *)0x4002550C))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_PDR_R        (*((volatile unsigned long *)0x40025514))
#define GPIO_PORTF_SLR_R        (*((volatile unsigned long *)0x40025518))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define GPIO_PORTF_ADCCTL_R     (*((volatile unsigned long *)0x40025530))
#define GPIO_PORTF_DMACTL_R     (*((volatile unsigned long *)0x40025534))
	
/* Pins Definition */

#define Leds_Port											GPIO_PORTF_BASE
#define RedLed   											(1<<1)
#define BlueLed  											(1<<2)
#define GreenLed 											(1<<3)


#define Buttons_Motor_Port						GPIO_PORTA_BASE
#define PA0														(1<<0)
#define PA1														(1<<1)
#define Passenger_Elevate_Button			(1<<2)
#define Passenger_Lower_Button			 	(1<<3)
#define Driver_Elevate_Button					(1<<4)
#define Driver_Lower_Button					 	(1<<5)
#define DC_Motor_In1									(1<<6)
#define DC_Motor_In2									(1<<7)


#define Sensors_Port									GPIO_PORTC_BASE
#define PC0														(1<<0)
#define PC1														(1<<1)
#define PC2														(1<<2)
#define PC3														(1<<3)
#define Object_Detection_Sensor				(1<<4)
#define Window_Upper_Limit						(1<<5)
#define Window_Lower_Limit						(1<<6)
#define Window_Lock_Switch						(1<<7)


#define LOW															0
#define HIGH														1

void PortA_Config(void);
void PortC_Config(void);
void PortF_Config(void);


#endif