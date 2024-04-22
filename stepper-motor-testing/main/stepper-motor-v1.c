/* File: stepper-motor-v1.c
Notes: Controlling a stepper motor manually.  
*/

//Header Files
#include <stdio.h>
#include "driver\gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

    while(1)
    {
        // Note: vTaskDelay(tick_input) - time_input is in ticks for RTOS ~ approximately 10 milliseconds? 
        gpio_set_level(STEPPER_IN_1, 1);
        vTaskDelay(delay_time);
        gpio_set_level(STEPPER_IN_1, 0);
        vTaskDelay(delay_time);

        gpio_set_level(STEPPER_IN_3, 1);
        vTaskDelay(delay_time);
        gpio_set_level(STEPPER_IN_3, 0);
        vTaskDelay(delay_time);

        gpio_set_level(STEPPER_IN_2, 1);
        vTaskDelay(delay_time);
        gpio_set_level(STEPPER_IN_2, 0);
        vTaskDelay(delay_time);

        gpio_set_level(STEPPER_IN_4, 1);
        vTaskDelay(delay_time);
        gpio_set_level(STEPPER_IN_4, 0);
        vTaskDelay(delay_time);

    }
}