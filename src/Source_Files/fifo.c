/*******************************************************************************
 * @Author: Adam Smrekar
 * @file: fifo.c
 * @brief: FIFO Implementation (Implemented like Circular Buffer)
 * 		   Referenced code online including from TechieDelight
 ******************************************************************************/

//***********************************************************************************
// Includes
//***********************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <common/include/rtos_utils.h>

#include "fifo.h"

/***************************************************************************//**
 * @brief
 *   Initialize FIFO Queue
 *
 * @return
 *   Fifo Queue
 ******************************************************************************/
struct InputFifo_t *fifo_init()
{
    // Init Queue
    struct InputFifo_t *q;
    q = (struct InputFifo_t *)malloc(FIFO_DEPTH * sizeof(int));

    q->head = 0;
    q->tail = -1;
    q->size = 0;

    // Set each element in queue to 0
    for(int i = 0; i < FIFO_DEPTH; ++i)
    {
        q->input[i] = 0;
    }

    return q;

}
/***************************************************************************//**
 * @brief
 *   Add a value to the FIFO
 *
 * @param value
 *   Value to add to the FIFO
 * @param p_Fifo
 * 	 Fifo Queue
 * 
 * @return void
 ******************************************************************************/
void InputFifo_Put(struct InputFifo_t *p_Fifo, InputValue_t value)
{
    // Queue is full, return
    if(p_Fifo->size == FIFO_DEPTH)
    {
        return;
    }

    p_Fifo->tail = (p_Fifo->tail + 1) % FIFO_DEPTH;
    p_Fifo->input[p_Fifo->tail] = value;
    (p_Fifo->size)++;

    return;
}

/***************************************************************************//**
 * @brief
 *   Get the next value from the FIFO
 *
 * @param p_Fifo
 *   Queue
 * 
 * @return
 *   Next value in the FIFO
 ******************************************************************************/
InputValue_t InputFifo_Get(struct InputFifo_t *p_Fifo)
{
    // Queue is Empty
    if(p_Fifo->size == 0)
    {
        return 0;
    }

    InputValue_t p_value = p_Fifo->input[p_Fifo->head];
    p_Fifo->head = (p_Fifo->head + 1) % FIFO_DEPTH;
    (p_Fifo->size)--;
    return p_value;
}
