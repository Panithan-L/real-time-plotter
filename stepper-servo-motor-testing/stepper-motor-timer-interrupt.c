/* File: stepper-motor-timer-interrupt.c
Notes: Trying to control stepper motors using timer interrupts.
        4/25/2024 - Able to control x and y stepper motors with coordinates. [Drawing a Square, looping]
*/

//Header Files
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "esp_timer.h"


//Function Prototypes
void stepper_setup(int coil_in_1, int coil_in_2, int coil_in_3, int coil_in_4);

//Defining pins connected to the stepper motor
// X Axis Stepper Motor
#define STEPPER_IN_X1   33  
#define STEPPER_IN_X3   12

#define STEPPER_IN_X2   15      
#define STEPPER_IN_X4   27     

#define STEPPER_IN_Y1   32
#define STEPPER_IN_Y3   19

#define STEPPER_IN_Y2   14
#define STEPPER_IN_Y4   21

//Global Variables
volatile int current_xstep = STEPPER_IN_X1;
volatile int current_ystep = STEPPER_IN_Y1;

//Pin output step[] = {STEPPER_IN_X1, STEPPER_IN_X2, STEPPER_IN_X3, STEPPER_IN_X1}
volatile int step[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

volatile int x_current = 0;
volatile int cnt_x = 0;

volatile int y_current = 0;
volatile int cnt_y = 0;

// Coordinates to make a rectangle
volatile int x_coordinates[] = {10, 10, 100, 100};
volatile int y_coordinates[] = {10, 100, 100, 10};

volatile int x_size = sizeof(x_coordinates) / sizeof(x_coordinates[0]);
volatile int y_size = sizeof(y_coordinates) / sizeof(y_coordinates[0]);

void timer_callback(void *param)
{
    //Timer callback to drive the stepper motor and blink onboard LED

    //State Machine - X Axis
    if (x_current < x_coordinates[cnt_x])
    {
        //State Machine - Forward 1 -> 4 -> 2 -> 3 -> 1
        switch(current_xstep)
        {
            case STEPPER_IN_X1:
                current_xstep = STEPPER_IN_X4;
                gpio_set_level(STEPPER_IN_X1, step[3][0]);
                gpio_set_level(STEPPER_IN_X2, step[3][1]);
                gpio_set_level(STEPPER_IN_X3, step[3][2]);
                gpio_set_level(STEPPER_IN_X4, step[3][3]);
                break;
            case STEPPER_IN_X2:
                current_xstep = STEPPER_IN_X3;
                gpio_set_level(STEPPER_IN_X1, step[2][0]);
                gpio_set_level(STEPPER_IN_X2, step[2][1]);
                gpio_set_level(STEPPER_IN_X3, step[2][2]);
                gpio_set_level(STEPPER_IN_X4, step[2][3]);
                break;
            case STEPPER_IN_X3:
                current_xstep = STEPPER_IN_X1;
                gpio_set_level(STEPPER_IN_X1, step[0][0]);
                gpio_set_level(STEPPER_IN_X2, step[0][1]);
                gpio_set_level(STEPPER_IN_X3, step[0][2]);
                gpio_set_level(STEPPER_IN_X4, step[0][3]);
                break;
            case STEPPER_IN_X4:
                current_xstep = STEPPER_IN_X2;
                gpio_set_level(STEPPER_IN_X1, step[1][0]);
                gpio_set_level(STEPPER_IN_X2, step[1][1]);
                gpio_set_level(STEPPER_IN_X3, step[1][2]);
                gpio_set_level(STEPPER_IN_X4, step[1][3]);
                break;
        }
        //Increment the x position
        x_current++;
    }
    else if (x_current > x_coordinates[cnt_x])
    {
        //State Machine - Reverse 1 -> 3 -> 2 -> 4 -> 1
        switch(current_xstep)
        {
            case STEPPER_IN_X1:
                current_xstep = STEPPER_IN_X3;
                gpio_set_level(STEPPER_IN_X1, step[2][0]);
                gpio_set_level(STEPPER_IN_X2, step[2][1]);
                gpio_set_level(STEPPER_IN_X3, step[2][2]);
                gpio_set_level(STEPPER_IN_X4, step[2][3]);
                break;
            case STEPPER_IN_X2:
                current_xstep = STEPPER_IN_X4;
                gpio_set_level(STEPPER_IN_X1, step[3][0]);
                gpio_set_level(STEPPER_IN_X2, step[3][1]);
                gpio_set_level(STEPPER_IN_X3, step[3][2]);
                gpio_set_level(STEPPER_IN_X4, step[3][3]);
                break;
            case STEPPER_IN_X3:
                current_xstep = STEPPER_IN_X2;
                gpio_set_level(STEPPER_IN_X1, step[1][0]);
                gpio_set_level(STEPPER_IN_X2, step[1][1]);
                gpio_set_level(STEPPER_IN_X3, step[1][2]);
                gpio_set_level(STEPPER_IN_X4, step[1][3]);
                break;
            case STEPPER_IN_X4:
                current_xstep = STEPPER_IN_X1;
                gpio_set_level(STEPPER_IN_X1, step[0][0]);
                gpio_set_level(STEPPER_IN_X2, step[0][1]);
                gpio_set_level(STEPPER_IN_X3, step[0][2]);
                gpio_set_level(STEPPER_IN_X4, step[0][3]);
                break;
        }
        //Decrement the x position
        x_current--;
    }
    else
    {
        //State Machine - Stationary 
        switch(current_xstep)
        {
            case STEPPER_IN_X1:
                current_xstep = STEPPER_IN_X1;
                gpio_set_level(STEPPER_IN_X1, step[0][0]);
                gpio_set_level(STEPPER_IN_X2, step[0][1]);
                gpio_set_level(STEPPER_IN_X3, step[0][2]);
                gpio_set_level(STEPPER_IN_X4, step[0][3]);
                break;
            case STEPPER_IN_X2:
                current_xstep = STEPPER_IN_X2;
                gpio_set_level(STEPPER_IN_X1, step[1][0]);
                gpio_set_level(STEPPER_IN_X2, step[1][1]);
                gpio_set_level(STEPPER_IN_X3, step[1][2]);
                gpio_set_level(STEPPER_IN_X4, step[1][3]);
                break;
            case STEPPER_IN_X3:
                current_xstep = STEPPER_IN_X3;
                gpio_set_level(STEPPER_IN_X1, step[2][0]);
                gpio_set_level(STEPPER_IN_X2, step[2][1]);
                gpio_set_level(STEPPER_IN_X3, step[2][2]);
                gpio_set_level(STEPPER_IN_X4, step[2][3]);
                break;
            case STEPPER_IN_X4:
                current_xstep = STEPPER_IN_X4;
                gpio_set_level(STEPPER_IN_X1, step[3][0]);
                gpio_set_level(STEPPER_IN_X2, step[3][1]);
                gpio_set_level(STEPPER_IN_X3, step[3][2]);
                gpio_set_level(STEPPER_IN_X4, step[3][3]);
                break;
        }
        //X position stays the same
        //x_current = x_current;
    }
    
    //State Machine - Y Axis
    if (y_current < y_coordinates[cnt_y])
    {
        //State Machine - Forward 1 -> 3 -> 2 -> 4 -> 1
        switch(current_ystep)
        {
            case STEPPER_IN_Y1:
                current_ystep = STEPPER_IN_Y3;
                gpio_set_level(STEPPER_IN_Y1, step[2][0]);
                gpio_set_level(STEPPER_IN_Y2, step[2][1]);
                gpio_set_level(STEPPER_IN_Y3, step[2][2]);
                gpio_set_level(STEPPER_IN_Y4, step[2][3]);
                break;
            case STEPPER_IN_Y2:
                current_ystep = STEPPER_IN_Y4;
                gpio_set_level(STEPPER_IN_Y1, step[3][0]);
                gpio_set_level(STEPPER_IN_Y2, step[3][1]);
                gpio_set_level(STEPPER_IN_Y3, step[3][2]);
                gpio_set_level(STEPPER_IN_Y4, step[3][3]);
                break;
            case STEPPER_IN_Y3:
                current_ystep = STEPPER_IN_Y2;
                gpio_set_level(STEPPER_IN_Y1, step[1][0]);
                gpio_set_level(STEPPER_IN_Y2, step[1][1]);
                gpio_set_level(STEPPER_IN_Y3, step[1][2]);
                gpio_set_level(STEPPER_IN_Y4, step[1][3]);
                break;
            case STEPPER_IN_Y4:
                current_ystep = STEPPER_IN_Y1;
                gpio_set_level(STEPPER_IN_Y1, step[0][0]);
                gpio_set_level(STEPPER_IN_Y2, step[0][1]);
                gpio_set_level(STEPPER_IN_Y3, step[0][2]);
                gpio_set_level(STEPPER_IN_Y4, step[0][3]);
                break;
        }
        //Increment the y position
        y_current++;
    }
    else if (y_current > y_coordinates[cnt_y])
    {
        //State Machine - Reverse 1 -> 4 -> 2 -> 3 -> 1
        switch(current_ystep)
        {
            case STEPPER_IN_Y1:
                current_ystep = STEPPER_IN_Y4;
                gpio_set_level(STEPPER_IN_Y1, step[3][0]);
                gpio_set_level(STEPPER_IN_Y2, step[3][1]);
                gpio_set_level(STEPPER_IN_Y3, step[3][2]);
                gpio_set_level(STEPPER_IN_Y4, step[3][3]);
                break;
            case STEPPER_IN_Y2:
                current_ystep = STEPPER_IN_Y3;
                gpio_set_level(STEPPER_IN_Y1, step[2][0]);
                gpio_set_level(STEPPER_IN_Y2, step[2][1]);
                gpio_set_level(STEPPER_IN_Y3, step[2][2]);
                gpio_set_level(STEPPER_IN_Y4, step[2][3]);
                break;
            case STEPPER_IN_Y3:
                current_ystep = STEPPER_IN_Y1;
                gpio_set_level(STEPPER_IN_Y1, step[0][0]);
                gpio_set_level(STEPPER_IN_Y2, step[0][1]);
                gpio_set_level(STEPPER_IN_Y3, step[0][2]);
                gpio_set_level(STEPPER_IN_Y4, step[0][3]);
                break;
            case STEPPER_IN_Y4:
                current_ystep = STEPPER_IN_Y2;
                gpio_set_level(STEPPER_IN_Y1, step[1][0]);
                gpio_set_level(STEPPER_IN_Y2, step[1][1]);
                gpio_set_level(STEPPER_IN_Y3, step[1][2]);
                gpio_set_level(STEPPER_IN_Y4, step[1][3]);
                break;
        }
        //Decrement the y position
        y_current--;
    }
    else
    {
        //State Machine - Stationary 
        switch(current_ystep)
        {
            case STEPPER_IN_Y1:
                current_ystep = STEPPER_IN_Y1;
                gpio_set_level(STEPPER_IN_Y1, step[0][0]);
                gpio_set_level(STEPPER_IN_Y2, step[0][1]);
                gpio_set_level(STEPPER_IN_Y3, step[0][2]);
                gpio_set_level(STEPPER_IN_Y4, step[0][3]);
                break;
            case STEPPER_IN_Y2:
                current_ystep = STEPPER_IN_Y2;
                gpio_set_level(STEPPER_IN_Y1, step[1][0]);
                gpio_set_level(STEPPER_IN_Y2, step[1][1]);
                gpio_set_level(STEPPER_IN_Y3, step[1][2]);
                gpio_set_level(STEPPER_IN_Y4, step[1][3]);
                break;
            case STEPPER_IN_Y3:
                current_ystep = STEPPER_IN_Y3;
                gpio_set_level(STEPPER_IN_Y1, step[2][0]);
                gpio_set_level(STEPPER_IN_Y2, step[2][1]);
                gpio_set_level(STEPPER_IN_Y3, step[2][2]);
                gpio_set_level(STEPPER_IN_Y4, step[2][3]);
                break;
            case STEPPER_IN_Y4:
                current_ystep = STEPPER_IN_Y4;
                gpio_set_level(STEPPER_IN_Y1, step[3][0]);
                gpio_set_level(STEPPER_IN_Y2, step[3][1]);
                gpio_set_level(STEPPER_IN_Y3, step[3][2]);
                gpio_set_level(STEPPER_IN_Y4, step[3][3]);
                break;
        }
        //Y position stays the same
        //y_current = y_current;
    }

    if ((x_current == x_coordinates[cnt_x]) && (y_current == y_coordinates[cnt_y]))
    {
        //Increase count x and count y
        cnt_x++;
        cnt_y++;
    }
    else if ((cnt_x == x_size) && (cnt_y == y_size))
    {
        //Reset to 0
        cnt_x = 0;
        cnt_y = 0;
    }
    
    else
    {
        //Count stays the same
        cnt_x = cnt_x;
        cnt_y = cnt_y;
    }


    static bool ON;
    // Switching between an on and off state
    ON = !ON;

    //Onboard LED is blinking
    gpio_set_level(13, ON);    

}


void app_main(void)
{
    //Variable Declaration

    //Configure Digital I/O for onboard LED
    esp_rom_gpio_pad_select_gpio(13);

    gpio_set_direction(13, GPIO_MODE_OUTPUT);

    //Initial setup for Stepper Motor GPIO Pins
    // X Axis Stepper setup
    stepper_setup(STEPPER_IN_X1, STEPPER_IN_X2, STEPPER_IN_X3, STEPPER_IN_X4);
    // Y Axis Stepper setup
    stepper_setup(STEPPER_IN_Y1, STEPPER_IN_Y2, STEPPER_IN_Y3, STEPPER_IN_Y4);


    const esp_timer_create_args_t my_timer_args = {
        .callback = &timer_callback,
        .name = "My Timer"
    };

    esp_timer_handle_t timer_handler;

    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
    // Set to cycle at a rate of 50000 microseconds =  50 ms
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 50000));

    while (true)
    {
        esp_timer_dump(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000000));
    }

}

//Functions
void stepper_setup(int coil_in_1, int coil_in_2, int coil_in_3, int coil_in_4)
{
    //Configure Digital I/O for Stepper Motor
    esp_rom_gpio_pad_select_gpio(coil_in_1);
    esp_rom_gpio_pad_select_gpio(coil_in_2);
    esp_rom_gpio_pad_select_gpio(coil_in_3);
    esp_rom_gpio_pad_select_gpio(coil_in_4);

    gpio_set_direction(coil_in_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(coil_in_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(coil_in_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(coil_in_4, GPIO_MODE_OUTPUT);

    //Set All Coils to Zero [initial setup]
    gpio_set_level(coil_in_1, 0);
    gpio_set_level(coil_in_2, 0);
    gpio_set_level(coil_in_3, 0);
    gpio_set_level(coil_in_4, 0);

}
