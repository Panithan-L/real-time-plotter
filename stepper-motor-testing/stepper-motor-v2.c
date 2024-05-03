/* File: stepper-motor-v2.c
Notes: Created functions "stepper_forward" and "stepper_reverse", which controls
        the stepper motor for a given number of [inputted] step counts. Steps are 
        actuated at an [inputted] delay time.  
*/

//Header Files
#include <stdio.h>
#include "driver\gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//Function Prototypes
int stepper_forward(int num_steps, int current_step, int delay_time);
int stepper_reverse(int num_steps, int current_step, int delay_time);

//Defining that connect to the stepper motor
//Anwar's Stepper Motor - Blue 1; Red 2; Green 3; Black 4;
#define STEPPER_IN_1    12      //ESP32 Pin D12 - Motor Driver Pin B1 Input - Blue 1
#define STEPPER_IN_2    27      //ESP32 Pin D27 - Motor Driver Pin B2 Input - Red 2

#define STEPPER_IN_3    32      //ESP32 Pin D32 - Motor Driver Pin A1 Input - Green 3
#define STEPPER_IN_4    14      //ESP32 Pin D14 - Motor Driver Pin A2 Input - Black 4

void app_main(void)
{
    //Variable Declaration
    int delay_time;
    int current_step;

    //Data Input - set values for variables
    delay_time = 50;

    //Configure Digital I/O for Stepper Motor
    esp_rom_gpio_pad_select_gpio(STEPPER_IN_1);
    esp_rom_gpio_pad_select_gpio(STEPPER_IN_2);
    esp_rom_gpio_pad_select_gpio(STEPPER_IN_3);
    esp_rom_gpio_pad_select_gpio(STEPPER_IN_4);

    gpio_set_direction(STEPPER_IN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEPPER_IN_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEPPER_IN_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEPPER_IN_4, GPIO_MODE_OUTPUT);

    // Set the outputs to 0 initially
    gpio_set_level(STEPPER_IN_1, 0);
    gpio_set_level(STEPPER_IN_1, 0);
    gpio_set_level(STEPPER_IN_1, 0);
    gpio_set_level(STEPPER_IN_1, 0);

    // Initial Set
    gpio_set_level(STEPPER_IN_1, 1);
    vTaskDelay(1);
    gpio_set_level(STEPPER_IN_1, 0);
    vTaskDelay(1);
    gpio_set_level(STEPPER_IN_3, 1);
    vTaskDelay(1);
    gpio_set_level(STEPPER_IN_3, 0);
    vTaskDelay(1);
    gpio_set_level(STEPPER_IN_2, 1);
    vTaskDelay(1);
    gpio_set_level(STEPPER_IN_2, 0);
    vTaskDelay(1);
    gpio_set_level(STEPPER_IN_3, 1);
    vTaskDelay(1);
    gpio_set_level(STEPPER_IN_3, 0);
    vTaskDelay(1);
    gpio_set_level(STEPPER_IN_1, 1);
    vTaskDelay(1);
    gpio_set_level(STEPPER_IN_1, 0);
    vTaskDelay(1);

    current_step = STEPPER_IN_1;

    // Trying to move the motor forward 10 steps
    current_step = stepper_forward(15, current_step, delay_time);

    current_step = stepper_reverse(15, current_step, delay_time);
}


//Functions
int stepper_forward(int num_steps, int current_step, int delay_time)
{
    // Variable Declaration
    int i;

    for (i = 1; i <= num_steps; i++)
    {
        gpio_set_level(current_step, 1);
        vTaskDelay(delay_time);
        gpio_set_level(current_step, 0);
        vTaskDelay(delay_time);

        // State Machine - Forward Cycle: 1 -> 3 -> 2 -> 4 -> 1
        switch(current_step)
        {
            case STEPPER_IN_1:
                current_step = STEPPER_IN_3;
                break;
            case STEPPER_IN_2:
                current_step = STEPPER_IN_4;
                break;
            case STEPPER_IN_3:
                current_step = STEPPER_IN_2;
                break;
            case STEPPER_IN_4:
                current_step = STEPPER_IN_1;
                break;
        }
    }
    return current_step;
}

int stepper_reverse(int num_steps, int current_step, int delay_time)
{
    //Variable Declaration
    int i;

    for (i = 1; i <= num_steps; i++)
    {
        gpio_set_level(current_step, 1);
        vTaskDelay(delay_time);
        gpio_set_level(current_step, 0);
        vTaskDelay(delay_time);

        // State Machine - Reverse Cycle: 4 -> 2 -> 3 -> 1 -> 4
        switch(current_step)
        {
            case STEPPER_IN_1:
                current_step = STEPPER_IN_4;
                break;
            case STEPPER_IN_2:
                current_step = STEPPER_IN_3;
                break;
            case STEPPER_IN_3:
                current_step = STEPPER_IN_1;
                break;
            case STEPPER_IN_4:
                current_step = STEPPER_IN_2;
                break;
        }
    }
    return current_step;
}
