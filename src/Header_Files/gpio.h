/************************************************************************************
 * @Author: Adam Smrekar
 * @file: gpio.h
 * @brief: Deals with GPIO, Button 0 and 1
 ***********************************************************************************/

//***********************************************************************************
// Includes
//***********************************************************************************
#include "em_gpio.h"

//***********************************************************************************
// Defines
//***********************************************************************************

// LED 0
#define	LED0_port		gpioPortF
#define LED0_pin		4
#define LED0_default	false 		// Default false (0) = off, true (1) = on
// LED 1
#define LED1_port		gpioPortF
#define LED1_pin		5
#define LED1_default	false		// Default false (0) = off, true (1) = on
// Button 0
#define	BTN0_port		gpioPortF
#define BTN0_pin		6
#define BTN0_default	true 		// Default false (0) = off, true (1) = on
// Button 1
#define BTN1_port		gpioPortF
#define BTN1_pin		7
#define BTN1_default	true		// Default false (0) = off, true (1) = on

// Button States
#define NO_BTN_PUSHED	4			// No Button Pushed
#define BTN0_PUSHED		5			// Button 0 is pushed, Button 1 is not pushed
#define BTN1_PUSHED		6			// Button 1 is pushed, Button 0 is not pushed
#define BOTH_BTN_PUSHED	7			// Both Buttons pushed

//***********************************************************************************
// Function Prototypes
//***********************************************************************************
void gpio_open(void);				// Init GPIO
void gpio_interrupt_init(void);		// Init GPIO Interrupts
int8_t sample_pushbutton_0(void);	// Sample Button 0, Return Button 0 State
int8_t sample_pushbutton_1(void);	// Sample Button 1, Return Button 1 State
