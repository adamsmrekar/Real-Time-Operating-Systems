/*******************************************************************************
 * @Author: Adam Smrekar
 * @file: shared_data.c
 * @brief: Init of shared data for vehicle speed, direction, and data
 *******************************************************************************/

//***********************************************************************************
// Includes
//***********************************************************************************
#include "shared_data.h"
#include  <kernel/include/os.h>

//***********************************************************************************
// Function Prototypes
//***********************************************************************************
struct VehicleSpeedData_t vehicle_speed_data_init(struct VehicleSpeedData_t vehicle_speed)
{
    vehicle_speed = (struct VehicleSpeedData_t){.up_count = 0, .down_count = 0, .cur_speed = 0};

    return vehicle_speed;
}

struct VehicleDirectionData_t vehicle_direction_data_init(struct VehicleDirectionData_t vehicle_dir)
{
    vehicle_dir = (struct VehicleDirectionData_t){.left_count = 0, .right_count = 0,
                                                  .cur_direction_ms = 0, .cur_direction = 0};
    return vehicle_dir;
}

struct DisplayData_t display_data_init(struct DisplayData_t display_data)
{
    display_data = (struct DisplayData_t){.cur_direction = 0, .cur_speed = 0.0};

    return display_data;
}
