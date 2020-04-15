/***************************************************************************//**
 * @Author: Adam Smrekar + Example Code from Silicon Labs
 * @file: main.c
 * @brief: Main file that deals with creating and starting tasks
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licenser of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.
 * The software is governed by the sections of the MSLA applicable to Micrium
 * Software.
 *
 ******************************************************************************/

#include  <bsp_os.h>
#include  "bsp.h"
#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>
#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>
#include  <stdio.h>
#include  "em_emu.h"
#include  "cap_touch.h"
#include  "capsense.h"
#include  "gpio.h"
#include  "fifo.h"
#include  "shared_data.h"
#include  "displayconfigapp.h"
#include  "display.h"
#include  "textdisplay.h"
#include  "retargettextdisplay.h"

#define  MAIN_START_TASK_PRIO              21u
#define  MAIN_START_TASK_STK_SIZE         512u
#define  SPEED_SETPOINT_TASK_PRIO          21u
#define  SPEED_SETPOINT_TASK_STK_SIZE     512u
#define  VEHICLE_DIRECTION_TASK_PRIO       21u
#define  VEHICLE_DIRECTION_TASK_STK_SIZE  512u
#define  VEHICLE_MONITOR_TASK_PRIO         21u
#define  VEHICLE_MONITOR_TASK_STK_SIZE    512u
#define  LED_OUTPUT_TASK_PRIO              21u
#define  LED_OUTPUT_TASK_STK_SIZE         512u
#define  LCD_DISPLAY_TASK_PRIO             21u
#define  LCD_DISPLAY_TASK_STK_SIZE        512u
#define  IDLE_TASK_PRIO            		   62u
#define  IDLE_TASK_STK_SIZE       		  512u

#define  LED0_OFF						     0
#define  LED0_ON							 1
#define  LED1_OFF							 2
#define  LED1_ON							 3

// Globals

static CPU_STK MainStartTaskStk[MAIN_START_TASK_STK_SIZE];     			 /* Start Task Stack.                                    */
static OS_TCB MainStartTaskTCB;							       			 /* Start Task TCB.                                      */

static CPU_STK SpeedSetpointTaskStk[SPEED_SETPOINT_TASK_STK_SIZE];       /* Speed Setpoint Stack.                                */
static OS_TCB SpeedSetpointTaskTCB;							   			 /* Speed Setpoint TCB.                                  */

static CPU_STK VehicleDirectionTaskStk[VEHICLE_DIRECTION_TASK_STK_SIZE]; /* Vehicle Direction Stack.                             */
static OS_TCB VehicleDirectionTaskTCB;							  		 /* Vehicle Direction TCB.                               */

static CPU_STK VehicleMonitorTaskStk[VEHICLE_MONITOR_TASK_STK_SIZE];     /* Vehicle Monitor Stack.                               */
static OS_TCB VehicleMonitorTaskTCB;							   	     /* Vehicle Monitor TCB.                                 */

static CPU_STK LEDOutputTaskStk[LED_OUTPUT_TASK_STK_SIZE]; 				 /* LED Output Task Stack.                               */
static OS_TCB LEDOutputTaskTCB;									  	     /* LED Output Task TCB.                                 */

static CPU_STK LCDDisplayTaskStk[LCD_DISPLAY_TASK_STK_SIZE]; 		     /* LCD Display Task Stack.                              */
static OS_TCB LCDDisplayTaskTCB;									     /* LCD Display Task TCB.                                */

static CPU_STK IdleTaskStk[IDLE_TASK_STK_SIZE];				 			 /* Idle Task Stack.                                     */
static OS_TCB IdleTaskTCB;									  			 /* Idle Task TCB.                                       */

static OS_MUTEX Vehicle_Speed_Mutex;									 /* Vehicle Speed Mutex.                                 */
static OS_MUTEX Vehicle_Direction_Mutex;								 /* Vehicle Direction Mutex.                             */
OS_MUTEX queue_mutex;													 /* Queue Mutex.                                         */
OS_SEM btn_sem;														     /* Button Semaphore.                                    */

OS_FLAG_GRP Vehicle_Monitor_Flag_Group;							    	 /* Vehicle Monitor Flag Group.                          */
const OS_FLAGS speed_update_flag 	 = 0x1;								 /* Speed Update Flag.                                   */			
const OS_FLAGS direction_update_flag = 0x2; 							 /* Direction Update Flag.                               */	
OS_FLAG_GRP LED_Output_Flag_Group;										 /* LED Output Flag Group.                               */
const OS_FLAGS alert_event_flag 	 = 0x3;								 /* Alert Event Flag.                                    */

volatile int cur_led0_state			 = LED0_OFF;						 /* Current LED0 State.                                  */
volatile int cur_led1_state			 = LED1_OFF;						 /* Current LED1 State.                                  */
extern volatile int cap_touch_state;									 /* Capacitive Touch State.                              */

struct VehicleSpeedData_t vehicle_speed;								 /* Vehicle Speed.                                       */
struct VehicleDirectionData_t vehicle_dir;								 /* Vehicle Direction.                                   */
struct InputFifo_t *fifo_queue;											 /* Fifo Queue.                                          */

static DISPLAY_Device_t display_device;								     /* Display Device.                                      */

// Function Prototypes
static void MainStartTask(void *p_arg);
static void SpeedSetpointTask(void *p_arg);
static void VehicleDirectionTask(void *p_arg);
static void VehicleMonitorTask(void *p_arg);
static void LEDOutputTask(void *p_arg);
static void LCDDisplayTask(void *p_arg);
static void IdleTask(void *p_arg);
void update_LED(void);

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C applications. It is assumed that your code will
*               call main() once you have performed all necessary initialization.
*
* Argument(s) : None.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
int main(void)
{
    RTOS_ERR err;

    BSP_SystemInit();                                           /* Initialize System.                                   */
    CPU_Init();                                                 /* Initialize CPU.                                      */

    OS_TRACE_INIT();
    OSInit(&err);                                               /* Initialize the Kernel.                               */
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    OSTaskCreate(&MainStartTaskTCB,                        		/* Create the Start Task.                               */
                 "Main Start Task",
                  MainStartTask,
                  DEF_NULL,
                  MAIN_START_TASK_PRIO,
                 &MainStartTaskStk[0],
                 (MAIN_START_TASK_STK_SIZE / 10u),
                  MAIN_START_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    OSStart(&err);                                              /* Start the kernel.                                    */
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    return (1);
}

/*
*********************************************************************************************************
*                                          MainStartTask()
*
* Description : This is the task that will be called by the Startup when all services are initializes
*               successfully.
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static void MainStartTask(void *p_arg)
{
    RTOS_ERR err;

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */


    BSP_TickInit();                                             /* Initialize Kernel tick source.                       */


#if (OS_CFG_STAT_TASK_EN == DEF_ENABLED)
    OSStatTaskCPUUsageInit(&err);                               /* Initialize CPU Usage.                                */
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
#endif

    OSStatTaskCPUUsageInit(&err);

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

    OSStatReset(&err);

    Common_Init(&err);                                          /* Call common module initialization example.           */
    APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);

    BSP_OS_Init();                                              /* Initialize the BSP. It is expected that the BSP ...  */
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */

    CPU_TS_TmrInit();

    gpio_open();												/* Initialize GPIO 									    */
    CAPSENSE_Init();											/* Initialize Capsense								    */

	gpio_interrupt_init(); 										/* Init GPIO Interrupts 								*/
	fifo_queue = fifo_init(); 									/* Init Fifo Queue  									*/

	vehicle_speed = vehicle_speed_data_init(vehicle_speed);		/* Init Vehicle Speed Data 								*/
	vehicle_dir = vehicle_direction_data_init(vehicle_dir);		/* Init Vehicle Direction Data							*/

	OSSemCreate(&btn_sem,										/* Init Button Semaphore  							    */
				"Button Semaphore",
				0,
				&err);

	OSMutexCreate(&queue_mutex,									/* Init Queue Mutex							            */
				  "Queue Mutex",
				  &err);

	OSMutexCreate(&Vehicle_Speed_Mutex,							/* Init Vehicle Speed Mutex					            */
				  "Vehicle Speed Mutex",
				  &err);

	OSMutexCreate(&Vehicle_Direction_Mutex,						/* Init Vehicle Direction Mutex				            */
				  "Vehicle Direction Mutex",
				  &err);

	OSFlagCreate(&Vehicle_Monitor_Flag_Group,					/* Init Vehicle Monitor Flag Group  		            */
				 "Vehicle Monitor Flag Group",
				 speed_update_flag,
				 &err);

	OSFlagCreate(&Vehicle_Monitor_Flag_Group,					/* Init Vehicle Monitor Flag Group  		            */
				 "Vehicle Monitor Flag Group",
				 direction_update_flag,
				 &err);

	OSFlagCreate(&LED_Output_Flag_Group,						/* Init LED Output Flag Group  		                    */
				 "LED Output Flag Group",
				 alert_event_flag,
				 &err);

    OSTaskCreate(&SpeedSetpointTaskTCB,                         /* Create the Speed Setpoint Task.                      */
                 "Speed Setpoint Task",
                 SpeedSetpointTask,
                 DEF_NULL,
				 SPEED_SETPOINT_TASK_PRIO,
                 &SpeedSetpointTaskStk[0],
                 (SPEED_SETPOINT_TASK_STK_SIZE / 10u),
				 SPEED_SETPOINT_TASK_STK_SIZE,
                 0u,
                 0u,
                 DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

    OSTaskCreate(&VehicleDirectionTaskTCB,                      /* Create the Vehicle Direction Task.                   */
                 "Vehicle Direction Task",
                 VehicleDirectionTask,
                 DEF_NULL,
				 VEHICLE_DIRECTION_TASK_PRIO,
                 &VehicleDirectionTaskStk[0],
                 (VEHICLE_DIRECTION_TASK_STK_SIZE / 10u),
				 VEHICLE_DIRECTION_TASK_STK_SIZE,
                 0u,
                 0u,
                 DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

    OSTaskCreate(&VehicleMonitorTaskTCB,                        /* Create the Vehicle Monitor Task.                     */
                 "Vehicle Monitor Task",
                 VehicleMonitorTask,
                 DEF_NULL,
				 VEHICLE_MONITOR_TASK_PRIO,
                 &VehicleMonitorTaskStk[0],
                 (VEHICLE_MONITOR_TASK_STK_SIZE / 10u),
				 VEHICLE_MONITOR_TASK_STK_SIZE,
                 0u,
                 0u,
                 DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

    OSTaskCreate(&LEDOutputTaskTCB,                		       	/* Create the LED Output Task.                          */
                 "LED Output Task",
                 LEDOutputTask,
                 DEF_NULL,
				   LED_OUTPUT_TASK_PRIO,
                 &LEDOutputTaskStk[0],
                 (LED_OUTPUT_TASK_STK_SIZE / 10u),
				   LED_OUTPUT_TASK_STK_SIZE,
                 0u,
                 0u,
                 DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

    OSTaskCreate(&LCDDisplayTaskTCB,                		    /* Create the LCD Display Task.                         */
                 "LCD Display Task",
                 LCDDisplayTask,
                 DEF_NULL,
				   LCD_DISPLAY_TASK_PRIO,
                 &LCDDisplayTaskStk[0],
                 (LCD_DISPLAY_TASK_STK_SIZE / 10u),
				   LCD_DISPLAY_TASK_STK_SIZE,
                 0u,
                 0u,
                 DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

    OSTaskCreate(&IdleTaskTCB,                		            /* Create the Idle Task.                                */
					 "Idle Task",
					  IdleTask,
					  DEF_NULL,
					  IDLE_TASK_PRIO,
					 &IdleTaskStk[0],
					 (IDLE_TASK_STK_SIZE / 10u),
					 IDLE_TASK_STK_SIZE,
					  0u,
					  0u,
					  DEF_NULL,
					 (OS_OPT_TASK_STK_CLR),
					 &err);

}

/*
*********************************************************************************************************
*                                          SpeedSetpointTask()
*
* Description : Task that updates vehicle speed
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static void SpeedSetpointTask(void *p_arg)
{
    RTOS_ERR err;
    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */
    InputValue_t q_pop_val = 0;

    while (DEF_ON)
    {
		OSSemPend(&btn_sem,										/* Pend button semaphore from button interrupts         */
				  0,
				  OS_OPT_PEND_BLOCKING,
				  DEF_NULL,
				  &err);

		OSMutexPend(&queue_mutex,
				    0,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);

		q_pop_val = InputFifo_Get(fifo_queue);					/* Pop queue and get value						        */

		OSMutexPost(&queue_mutex,
					OS_OPT_POST_NONE,
					&err);


		if(q_pop_val > NO_BTN_PUSHED)
		{
			OSMutexPend(&Vehicle_Speed_Mutex,
						0,
						OS_OPT_PEND_BLOCKING,
						DEF_NULL,
						&err);

			if(q_pop_val == BTN0_PUSHED)						/* Increment speed if button 0 is pushed		        */
			{
				vehicle_speed.cur_speed += 5;
				(vehicle_speed.up_count)++;
			}
			else if(q_pop_val == BTN1_PUSHED)					/* Decrement speed if button 1 is pushed		        */
			{
				if(vehicle_speed.cur_speed > 5)
				{
					vehicle_speed.cur_speed -= 5;
				}
				else
				{
					vehicle_speed.cur_speed = 0;
				}

				(vehicle_speed.down_count)++;
			}

			OSMutexPost(&Vehicle_Speed_Mutex,
						OS_OPT_POST_NONE,
						&err);
		}

		OSFlagPost(&Vehicle_Monitor_Flag_Group,					/* Post Speed Update Flag						        */
				   speed_update_flag,
				   OS_OPT_POST_FLAG_SET,
				   &err);
    }
}

/*
*********************************************************************************************************
*                                          VehicleDirectionTask()
*
* Description : Task that deals with slider input/capacitive touch input. Updates vehicle direction
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static void VehicleDirectionTask(void *p_arg)
{
    RTOS_ERR err;

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

    int prev_state = VEHICLE_DIR_NONE;
    int cur_state = VEHICLE_DIR_NONE;
    int cur_dir_cntr = 0;

    while (DEF_ON)
    {
    	CAPSENSE_Sense();
    	cap_touch();

    	if(cap_touch_state > CAP_TOUCH_NO_DIR)					/* If capacitive touch sensor is touched                */
    	{
    		OSMutexPend(&Vehicle_Direction_Mutex,
						0,
						OS_OPT_PEND_BLOCKING,
						DEF_NULL,
						&err);

    		// Update Vehicle Direction
			if(cap_touch_state == CAP_TOUCH_DIR_FAR_LEFT)
			{
				cur_state = VEHICLE_DIR_HARD_LEFT;
				vehicle_dir.cur_direction = VEHICLE_DIR_HARD_LEFT;
				(vehicle_dir.left_count)++;
			}
			else if(cap_touch_state == CAP_TOUCH_DIR_LEFT)
			{
				cur_state = VEHICLE_DIR_LEFT;
				vehicle_dir.cur_direction = VEHICLE_DIR_LEFT;
				(vehicle_dir.left_count)++;
			}
			else if(cap_touch_state == CAP_TOUCH_DIR_RIGHT)
			{
				cur_state = VEHICLE_DIR_RIGHT;
				vehicle_dir.cur_direction = VEHICLE_DIR_RIGHT;
				(vehicle_dir.right_count)++;
			}
			else if(cap_touch_state == CAP_TOUCH_DIR_FAR_RIGHT)
			{
				cur_state = VEHICLE_DIR_HARD_RIGHT;
				vehicle_dir.cur_direction = VEHICLE_DIR_HARD_RIGHT;
				(vehicle_dir.right_count)++;
			}
			else
			{
				vehicle_dir.cur_direction = VEHICLE_DIR_NONE;
			}

			// Check if the direction stays the same
			// Keep track of time that direction is held
			if(cur_state == prev_state)
			{
				cur_dir_cntr++;
				vehicle_dir.cur_direction_ms = cur_dir_cntr * 100;
			}
			else
			{
				cur_dir_cntr = 0;
			}



			OSMutexPost(&Vehicle_Direction_Mutex,
						OS_OPT_POST_NONE,
						&err);

			OSFlagPost(&Vehicle_Monitor_Flag_Group,				/* Post Direction Update Flag					        */
					   direction_update_flag,
					   OS_OPT_POST_FLAG_SET,
					   &err);
    	}
    	else
    	{
    		OSMutexPend(&Vehicle_Direction_Mutex,
						0,
						OS_OPT_PEND_BLOCKING,
						DEF_NULL,
						&err);

    		vehicle_dir.cur_direction = VEHICLE_DIR_NONE;		/* Capacitive touch is not touched, no Vehicle Direction*/

    		OSMutexPost(&Vehicle_Direction_Mutex,
						OS_OPT_POST_NONE,
						&err);

    		OSFlagPost(&Vehicle_Monitor_Flag_Group,				/* Post Direction Update Flag					        */
					   direction_update_flag,
					   OS_OPT_POST_FLAG_SET,
					   &err);
    	}

    	prev_state = cur_state;

    	OSTimeDly(100,                                          /*   100 OS Ticks                                       */
				  OS_OPT_TIME_DLY,                              /*   from now.                                          */
				  &err);
																/*   Check error code.                                  */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/*
*********************************************************************************************************
*                                          VehicleMonitorTask()
*
* Description : Checks for speed and direction violations
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static void VehicleMonitorTask(void *p_arg)
{
    RTOS_ERR err;

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */
    float cur_speed = 0.0;
    int8_t cur_dir = VEHICLE_DIR_NONE;
    uint32_t cur_dir_ms = 0;

    while (DEF_ON)
    {
    	OSFlagPend(&Vehicle_Monitor_Flag_Group,
				   speed_update_flag + direction_update_flag,
				   0,
				   OS_OPT_PEND_FLAG_SET_ANY,
				   DEF_NULL,
				   &err);

    	// Vehicle Speed
    	OSMutexPend(&Vehicle_Speed_Mutex,
					0,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);

    	cur_speed = vehicle_speed.cur_speed;

    	OSMutexPost(&Vehicle_Speed_Mutex,
					OS_OPT_POST_NONE,
					&err);

    	// Vehicle Direction
    	OSMutexPend(&Vehicle_Direction_Mutex,
					0,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);

    	cur_dir = vehicle_dir.cur_direction;
    	cur_dir_ms = vehicle_dir.cur_direction_ms;

    	OSMutexPost(&Vehicle_Direction_Mutex,
					OS_OPT_POST_NONE,
					&err);

    	// Check for speed violations
    	if((cur_speed >= 75) || (cur_speed >= 45 && cur_dir != VEHICLE_DIR_NONE))
    	{
    		cur_led0_state = LED0_ON;
    	}
    	else
    	{
    		cur_led0_state = LED0_OFF;
    	}

    	// Check for direction violations
    	if((cur_dir != VEHICLE_DIR_NONE) && (cur_dir_ms >= 5000))
    	{
    		cur_led1_state = LED1_ON;
    	}
    	else
		{
			cur_led1_state = LED1_OFF;
		}

		OSFlagPost(&Vehicle_Monitor_Flag_Group,
				   speed_update_flag + direction_update_flag,
				   OS_OPT_POST_FLAG_CLR,
				   &err);

    	OSFlagPost(&LED_Output_Flag_Group,
    			   alert_event_flag,
				   OS_OPT_POST_FLAG_SET,
				   &err);

    }
}

/*
*********************************************************************************************************
*                                          LEDOutputTask()
*
* Description : Task that updates LEDs
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static void LEDOutputTask(void *p_arg)
{
    RTOS_ERR err;
    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

    while (DEF_ON)
    {
    	OSFlagPend(&LED_Output_Flag_Group,
				   alert_event_flag,
				   0,
				   OS_OPT_PEND_FLAG_SET_ANY,
				   DEF_NULL,
				   &err);

		OSFlagPost(&LED_Output_Flag_Group,
				   alert_event_flag,
				   OS_OPT_POST_FLAG_CLR,
				   &err);

        update_LED();											/* Update LED state  		                            */
    }
}

/*
*********************************************************************************************************
*                                          LCDDisplayTask()
*
* Description : Task that updates LCD Display
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static void LCDDisplayTask(void *p_arg)
{
    RTOS_ERR err;
    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */
    struct DisplayData_t display_data;
	display_data = display_data_init(display_data);				/* Init Display Data									*/
	int cur_speed;

	DISPLAY_Init();												/* Initialize the display module. 						*/

																/* Retrieve the properties of the display. 				*/
	if (DISPLAY_DeviceGet(0, &display_device) != DISPLAY_EMSTATUS_OK)
	{
		while (1) ;
	}

																/* Retarget stdio to the display. 						*/
	if (TEXTDISPLAY_EMSTATUS_OK != RETARGET_TextDisplayInit())
	{
		while (1) ;
	}

    while (DEF_ON)
    {
    	// Vehicle Speed
		OSMutexPend(&Vehicle_Speed_Mutex,
					0,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);

		display_data.cur_speed = vehicle_speed.cur_speed;

		OSMutexPost(&Vehicle_Speed_Mutex,
					OS_OPT_POST_NONE,
					&err);

		// Vehicle Direction
		OSMutexPend(&Vehicle_Direction_Mutex,
					0,
					OS_OPT_PEND_BLOCKING,
					DEF_NULL,
					&err);

		display_data.cur_direction = vehicle_dir.cur_direction;

		OSMutexPost(&Vehicle_Direction_Mutex,
					OS_OPT_POST_NONE,
					&err);

		// Print to Display
		printf("\f");
		if(display_data.cur_direction == VEHICLE_DIR_NONE)
		{
			printf("\rDir: Straight\n");
		}
		else if(display_data.cur_direction == VEHICLE_DIR_HARD_LEFT)
		{
			printf("\rDir: Hard Left\n");
		}
		else if(display_data.cur_direction == VEHICLE_DIR_LEFT)
		{
			printf("\rDir: Left\n");
		}
		else if(display_data.cur_direction == VEHICLE_DIR_RIGHT)
		{
			printf("\rDir: Right\n");
		}
		else if(display_data.cur_direction == VEHICLE_DIR_HARD_RIGHT)
		{
			printf("\rDir: Hard Right\n");
		}

		cur_speed = (int)display_data.cur_speed;
		printf("\nSpeed: %d MPH\n", cur_speed);

		OSTimeDly(500,                                          /*   500 OS Ticks                                       */
				  OS_OPT_TIME_DLY,                              /*   from now.                                          */
				  &err);
																/*   Check error code.                                  */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);

    }
}


/*
*********************************************************************************************************
*                                          IdleTask()
*
* Description : Idle task
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static void IdleTask(void *p_arg)
{
    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

    while (DEF_ON)
    {
    	EMU_EnterEM1();
    }

}


/*
*********************************************************************************************************
*                                          Update_LED()
*
* Description : Update LED based off of button pushed and capacitive touch
*
* Argument(s) : None.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/
void update_LED(void)
{
	if(cur_led0_state == LED0_ON)
	{
		GPIO_PinOutSet(LED0_port, LED0_pin);
	}
	else
	{
		GPIO_PinOutClear(LED0_port, LED0_pin);
	}

	if(cur_led1_state == LED1_ON)
	{
		GPIO_PinOutSet(LED1_port, LED1_pin);
	}
	else
	{
		GPIO_PinOutClear(LED1_port, LED1_pin);
	}

}



