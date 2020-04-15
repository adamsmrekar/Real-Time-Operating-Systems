/************************************************************************************
 * @Author: Adam Smrekar
 * @file: gpio.c
 * @brief: Deals with GPIO, Button 0 and 1
 ***********************************************************************************/

//***********************************************************************************
// Include files
//***********************************************************************************
#include "gpio.h"
#include  <kernel/include/os.h>

//***********************************************************************************
// global variables
//***********************************************************************************
extern struct InputFifo_t *fifo_queue;
extern OS_SEM btn_sem;
extern OS_MUTEX queue_mutex;

//***********************************************************************************
// functions
//***********************************************************************************
void gpio_open(void)
{
    // Set LED ports to be standard output drive with default off (cleared)
    GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
    GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, LED0_default);

    GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
    GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, LED1_default);

    // Setup Button0 and Button1
    GPIO_DriveStrengthSet(BTN0_port, gpioDriveStrengthWeakAlternateWeak);
    GPIO_PinModeSet(BTN0_port, BTN0_pin, gpioModeInput, BTN0_default);

    GPIO_DriveStrengthSet(BTN1_port, gpioDriveStrengthWeakAlternateWeak);
    GPIO_PinModeSet(BTN1_port, BTN1_pin, gpioModeInput, BTN1_default);
}

void gpio_interrupt_init(void)
{
    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);   // Clear Interrupts
    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);         // Enable Interrupts
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
    GPIO_IntConfig(BTN0_port, BTN0_pin, true, true, BTN0_default);  // Initialize GPIO Interrupts
    GPIO_IntConfig(BTN1_port, BTN1_pin, true, true, BTN1_default);
}

int8_t sample_pushbutton_0(void)
{
    // If BTN0 is pushed, BTN1 is not pushed
    if(GPIO_PinInGet(BTN0_port, BTN0_pin) == 0 && GPIO_PinInGet(BTN1_port, BTN1_pin) == 1)
    {
        return BTN0_PUSHED;
    }
    // If BTN0 is pushed and BTN1 is pushed
    else if(GPIO_PinInGet(BTN0_port, BTN0_pin) == 0 && GPIO_PinInGet(BTN1_port, BTN1_pin) == 0)
    {
        return BOTH_BTN_PUSHED;
    }
    // If both buttons not pushed
    else if(GPIO_PinInGet(BTN0_port, BTN0_pin) == 1 && GPIO_PinInGet(BTN1_port, BTN1_pin) == 1)
    {
        return NO_BTN_PUSHED;
    }

    return NO_BTN_PUSHED;
}

int8_t sample_pushbutton_1(void)
{
    // If BTN1 is pushed, BTN0 is not pushed
    if(GPIO_PinInGet(BTN1_port, BTN1_pin) == 0 && GPIO_PinInGet(BTN0_port, BTN0_pin) == 1)
    {
        return BTN1_PUSHED;
    }
    // If BTN1 is pushed and BTN0 is pushed
    else if(GPIO_PinInGet(BTN1_port, BTN1_pin) == 0 && GPIO_PinInGet(BTN0_port, BTN0_pin) == 0)
    {
        return BOTH_BTN_PUSHED;
    }
    // If both buttons not pushed
    else if(GPIO_PinInGet(BTN1_port, BTN1_pin) == 1 && GPIO_PinInGet(BTN0_port, BTN0_pin) == 1)
    {
        return NO_BTN_PUSHED;
    }

    return NO_BTN_PUSHED;
}

void GPIO_EVEN_IRQHandler(void)
{
    RTOS_ERR err;
    int8_t button_state;

    button_state = sample_pushbutton_0();       // Update Button Pushed State

    OSMutexPend(&queue_mutex,                   // Acquire Queue Mutex
                0,
                OS_OPT_PEND_BLOCKING,
                DEF_NULL,
                &err);

    InputFifo_Put(fifo_queue, button_state);    // Push Button State to Queue

    OSMutexPost(&queue_mutex,                   // Release Queue Mutex
                OS_OPT_POST_NONE,
                &err);

    OSSemPost(&btn_sem,	                        // Post Button Semaphore
              OS_OPT_POST_1,
              &err);

    GPIO_IntClear(1 << BTN0_pin);               // Clear
}

void GPIO_ODD_IRQHandler(void)
{
    RTOS_ERR err;
    int8_t button_state;

    button_state = sample_pushbutton_1();       // Update Button Pushed State

    OSMutexPend(&queue_mutex,                   // Acquire Queue Mutex
                0,
                OS_OPT_PEND_BLOCKING,
                DEF_NULL,
                &err);

    InputFifo_Put(fifo_queue, button_state);	// Push Button State to Queue

    OSMutexPost(&queue_mutex,                   // Release Queue Mutex
                OS_OPT_POST_NONE,
                &err);

    OSSemPost(&btn_sem,                         // Post Button Semaphore
              OS_OPT_POST_1,
              &err);

    GPIO_IntClear(1 << BTN1_pin);               // Clear
}
