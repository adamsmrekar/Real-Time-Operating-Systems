/************************************************************************************
 * @Author: Adam Smrekar
 * @file: cap_touch.h
 * @brief: Deals with touch slider. Updates touch slider state
 ***********************************************************************************/

//***********************************************************************************
// defined files
//***********************************************************************************
#include "capsense.h"

//***********************************************************************************
// global variables
//***********************************************************************************
#define CAP0_CHANNEL            0		// Far Left Cap
#define CAP1_CHANNEL            1		// Left Cap
#define CAP2_CHANNEL            2		// Right Cap
#define CAP3_CHANNEL            3		// Far Right Cap

#define CAP_TOUCH_NO_DIR    	0		// Cap Not Touched
#define CAP_TOUCH_DIR_FAR_LEFT  1		// Cap Touched Far Left Side Only
#define CAP_TOUCH_DIR_LEFT  	2		// Cap Touched Left Side Only
#define CAP_TOUCH_DIR_RIGHT  	3		// Cap Touched Right Side Only
#define CAP_TOUCH_DIR_FAR_RIGHT 4		// Cap Touched Far Right Side Only
#define CAP_TOUCH_DIR_BOTH      5		// Cap Touched on Both Sides

//***********************************************************************************
// function prototypes
//***********************************************************************************
void cap_touch(void);		// Update capacitive touch state
