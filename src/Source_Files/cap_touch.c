/************************************************************************************
 * @Author: Adam Smrekar
 * @file: cap_touch.c
 * @brief: Deals with touch slider. Updates touch slider state
 ***********************************************************************************/

//***********************************************************************************
// Includes
//***********************************************************************************
#include "gpio.h"
#include "cap_touch.h"
#include "em_device.h"
#include "em_chip.h"

#include "capsense.h"

//***********************************************************************************
// Globals
//***********************************************************************************
volatile int cap_touch_state = CAP_TOUCH_NO_DIR;  /* State of Capacitive Touch Pad */

/*******************************************************************************
 * @brief 
 * 		Detects which capacitive touch pad is touched and updates 
 * 		if touched far left, left, right, far right pad.
 * 
 * @param[in] None
 * @param[out] None
 ******************************************************************************/
void cap_touch(void)
{
	if(CAPSENSE_getPressed(CAP0_CHANNEL)
		   && !CAPSENSE_getPressed(CAP1_CHANNEL)
		   && !CAPSENSE_getPressed(CAP2_CHANNEL)
		   && !CAPSENSE_getPressed(CAP3_CHANNEL))
	{
		cap_touch_state = CAP_TOUCH_DIR_FAR_LEFT;
	}
	else if(CAPSENSE_getPressed(CAP1_CHANNEL)
			   && !CAPSENSE_getPressed(CAP0_CHANNEL)
			   && !CAPSENSE_getPressed(CAP2_CHANNEL)
			   && !CAPSENSE_getPressed(CAP3_CHANNEL))
	{
		cap_touch_state = CAP_TOUCH_DIR_LEFT;
	}
	else if(CAPSENSE_getPressed(CAP2_CHANNEL)
			   && !CAPSENSE_getPressed(CAP3_CHANNEL)
			   && !CAPSENSE_getPressed(CAP0_CHANNEL)
			   && !CAPSENSE_getPressed(CAP1_CHANNEL))
	{
		cap_touch_state = CAP_TOUCH_DIR_RIGHT;
	}
	else if(CAPSENSE_getPressed(CAP3_CHANNEL)
			   && !CAPSENSE_getPressed(CAP2_CHANNEL)
			   && !CAPSENSE_getPressed(CAP0_CHANNEL)
			   && !CAPSENSE_getPressed(CAP1_CHANNEL))
	{
		cap_touch_state = CAP_TOUCH_DIR_FAR_RIGHT;
	}
	else if((CAPSENSE_getPressed(CAP0_CHANNEL)
			   || CAPSENSE_getPressed(CAP1_CHANNEL))
			   && (CAPSENSE_getPressed(CAP2_CHANNEL)
			   || CAPSENSE_getPressed(CAP3_CHANNEL)))
	{
		cap_touch_state = CAP_TOUCH_DIR_BOTH;
	}
	else if((!CAPSENSE_getPressed(CAP0_CHANNEL)
			   && !CAPSENSE_getPressed(CAP1_CHANNEL))
			   && !CAPSENSE_getPressed(CAP2_CHANNEL)
			   && !CAPSENSE_getPressed(CAP3_CHANNEL))
	{
		cap_touch_state = CAP_TOUCH_NO_DIR;
	}
}



