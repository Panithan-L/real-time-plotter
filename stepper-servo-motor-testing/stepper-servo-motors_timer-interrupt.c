/* File: stepper-servo-motors_timer-interrupt.c
Notes: Description
        5/2/2024 - Created File
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
#include "driver/mcpwm_prelude.h"

//Defining Pins on the ESP32
// X Axis Stepper Motor
#define STEPPER_IN_X1   33  
#define STEPPER_IN_X3   12

#define STEPPER_IN_X2   15      
#define STEPPER_IN_X4   27     
// Y Axis Stepper Motor
#define STEPPER_IN_Y1   32
#define STEPPER_IN_Y3   19

#define STEPPER_IN_Y2   14
#define STEPPER_IN_Y4   21

#define SERVO_PWM_GPIO  20

//Servo Motor Variables
#define SERVO_MIN_PULSE         500     // Minimum pulse width in microsecond
#define SERVO_MAX_PULSE         2500    // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        0       // Minimum angle
#define SERVO_MAX_DEGREE        220     // Maximum angle

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000    // 1MHz (1us per tick)
#define SERVO_TIMEBASE_PERIOD        20000      // 20,000 ticks -> 20ms


//Global Variables
volatile int current_xstep = STEPPER_IN_X1;
volatile int current_ystep = STEPPER_IN_Y1;

//Pin output step[] = {STEPPER_IN_X1, STEPPER_IN_X2, STEPPER_IN_X3, STEPPER_IN_X1}
volatile int step[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

volatile int x_current = 0;
volatile int cnt_x = 0;

volatile int y_current = 0;
volatile int cnt_y = 0;

// X and Y Coordinates to make a rectangle, also Servo Coordinates
volatile int x_coordinates[] = {10, 10, 100, 100};
volatile int y_coordinates[] = {10, 100, 100, 10};

volatile int servo_coordinates[] = {1, 0, 1, 0};

//Calculations
//Calculating the array sizes of x_coordinates, y_coordinates, and servo_coordinates
volatile int x_size = sizeof(x_coordinates) / sizeof(x_coordinates[0]);
volatile int y_size = sizeof(y_coordinates) / sizeof(y_coordinates[0]);
volatile int servo_state_size = sizeof(servo_coordinates) / sizeof(servo_coordinates[0]);

//Global Structure ...
volatile mcpwm_cmpr_handle_t comparator = NULL;

static const char *TAG = "example";



//Function Prototypes
void stepper_setup(int coil_in_1, int coil_in_2, int coil_in_3, int coil_in_4);
static inline uint32_t pulse_from_angle(int target_angle);
void servo_pwm_setup(int pwm_period, int pwm_resolution_hz, int servo_pwm_gpio_pin);

//Interrupt Service Routine [ISR] Prototypes
void stepper_servo_motor_timer_callback(void *param);



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

    //Setting up the PWM for the servo motor
    servo_pwm_setup(SERVO_TIMEBASE_PERIOD, SERVO_TIMEBASE_RESOLUTION_HZ, SERVO_PWM_GPIO);


    /*Initializing Timer Interrupt Frequencies*/
    //Stepper Motor timer interrupt frequency, in microseconds
    int stepper_servo_motor_timer = 20000;

    const esp_timer_create_args_t my_timer_args = {
        .callback = &stepper_servo_motor_timer_callback,
        .name = "Stepper Motor Timer"
    };

    //If there are multiple timer interrupts, will need multiple "timer_handler"s. So they don't conflict 
    //  Ex: timer_handler_1, timer_handler_2, etc.
    esp_timer_handle_t timer_handler;

    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
    // Set to cycle at a rate of 50000 microseconds = 50 ms
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, stepper_servo_motor_timer));

    while (true)
    {
        esp_timer_dump(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000000));
    }

}

//Functions
//Stepper Motor Setup Function
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

//Pulse Width Function - Calculates the Pulse Width corresponding to a target angle
static inline uint32_t pulse_from_angle(int target_angle)
{
    uint32_t pulse_width;
    uint32_t pulse_width_range = SERVO_MAX_PULSE - SERVO_MIN_PULSE;
    uint32_t angle_range = SERVO_MAX_DEGREE - SERVO_MIN_DEGREE;

    // Pulse Width = Mininum Pulse Width + (Pulse Width Range) * (Target Angle / Angle Range)
    pulse_width = SERVO_MIN_PULSE + (pulse_width_range * (target_angle - SERVO_MIN_DEGREE)) / angle_range;

    return pulse_width;
}

//Servo PWM Setup Function - Takes in PWM Period, Resolution, and GPIO Pin
void servo_pwm_setup(int pwm_period, int pwm_resolution_hz, int servo_pwm_gpio_pin)
{
    // Create timer and operator
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = pwm_resolution_hz,
        .period_ticks = pwm_period,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    //Checks if MCPWM timer is successful or not
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    //Connect timer and operator
    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    //Create comparator and generator from the operator
    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = servo_pwm_gpio_pin,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_from_angle(0)));

    //Set generator action on timer and compare event
    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    //Enable and start timer
    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

}


//Interrupt Service Routines [ISR]
void stepper_servo_motor_timer_callback(void *param)
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

    //Servo Motor Control
    switch (servo_coordinates[cnt_x])
    {
    case 1:
        printf("Angle: 0 \n");
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_from_angle(0)));
        break;
    
    case 0:
        printf("Angle: 90 \n");
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_from_angle(90)));
        break;
    }

    //Updating Count Values
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
