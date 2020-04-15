/*******************************************************************************
 * @Author: Adam Smrekar
 * @file: fifo.h
 * @brief: FIFO Implementation (Implemented like Circular Buffer)
 ******************************************************************************/

//***********************************************************************************
// Includes
//***********************************************************************************
#include <stdint.h>
#include <stdbool.h>

//***********************************************************************************
// Defines
//***********************************************************************************
#define FIFO_DEPTH 10

//***********************************************************************************
// Globals
//***********************************************************************************
typedef int8_t FifoIndex_t;
typedef int8_t InputValue_t;

//***********************************************************************************
// Structs
//***********************************************************************************
struct InputFifo_t {
    FifoIndex_t  head;
    FifoIndex_t  tail;
    InputValue_t input[FIFO_DEPTH];
    int8_t size;
};

//***********************************************************************************
// Function Prototypes
//***********************************************************************************
struct InputFifo_t *fifo_init();	                                 // Init Fifo Queue
void InputFifo_Put(struct InputFifo_t *p_Fifo, InputValue_t value);	 // Push value
InputValue_t InputFifo_Get(struct InputFifo_t *p_Fifo);				 // Pop and return head value