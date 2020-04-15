/*******************************************************************************
 * @Author: Adam Smrekar
 * @file: shared_data.h
 * @brief: Init of shared data for vehicle speed, direction, and data
 ******************************************************************************/

//***********************************************************************************
// Includes
//***********************************************************************************
#include <stdint.h>

//***********************************************************************************
// Defines
//***********************************************************************************
#define VEHICLE_DIR_NONE		0
#define VEHICLE_DIR_HARD_LEFT	1
#define VEHICLE_DIR_LEFT		2
#define VEHICLE_DIR_RIGHT		3
#define VEHICLE_DIR_HARD_RIGHT	4

//***********************************************************************************
// Structs
//***********************************************************************************
struct VehicleSpeedData_t{
    uint32_t up_count;
    uint32_t down_count;
    float    cur_speed;
};

struct VehicleDirectionData_t{
    uint32_t left_count;
    uint32_t right_count;
    uint32_t cur_direction_ms;
    int8_t   cur_direction;
};

struct DisplayData_t {
    int8_t cur_direction;
    float  cur_speed;
};

//***********************************************************************************
// Function Prototypes
//***********************************************************************************

// Init VehicleSpeedData_t struct, VehicleDirectionData_t struct, DisplayData_t struct
struct VehicleSpeedData_t vehicle_speed_data_init(struct VehicleSpeedData_t vehicle_speed);
struct VehicleDirectionData_t vehicle_direction_data_init(struct VehicleDirectionData_t vehicle_dir);
struct DisplayData_t display_data_init(struct DisplayData_t display_data);
