/* File: servo-motor_timer-interrupt.c
Notes: Description
        5/2/2024 - Created File
*/

//Header Files
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_timer.h"

//Global Variables
#define SERVO_MIN_PULSE         500     // Minimum pulse width in microsecond
#define SERVO_MAX_PULSE         2500    // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        0       // Minimum angle
#define SERVO_MAX_DEGREE        220     // Maximum angle

#define SERVO_PWM_GPIO             20          // GPIO connects to the PWM signal line - Pin A5
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000    // 1MHz (1us per tick)
#define SERVO_TIMEBASE_PERIOD        20000      // 20,000 ticks -> 20ms

//Servo Variables and Arrays
//servo_coordinates 0 means release -> angle is 90? and 1 means engage -> angle is 0?
volatile int servo_coordinates[] = {0, 1, 0, 1, 1, 0};
volatile int cnt_x = 0;

//Calculating the array sizes of x_coordinates and y_coordinates
volatile int servo_state_size = sizeof(servo_coordinates) / sizeof(servo_coordinates[0]);

//Setting the PWM to be a global variable
volatile mcpwm_cmpr_handle_t comparator = NULL;


static const char *TAG = "example";

//Function Prototypes
static inline uint32_t pulse_from_angle(int target_angle);
void servo_pwm_setup(int pwm_period, int pwm_resolution_hz, int servo_pwm_gpio_pin);

//Interrupt Service Routine [ISR] Prototypes
void servo_motor_timer_callback(void *param);


void app_main(void)
{
    //Variable Declaration

    //Configure Digital I/O for onboard LED
    esp_rom_gpio_pad_select_gpio(13);

    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    
    //Setting up the PWM for the servo motor
    servo_pwm_setup(SERVO_TIMEBASE_PERIOD, SERVO_TIMEBASE_RESOLUTION_HZ, SERVO_PWM_GPIO);


    /*Initializing Timer Interrupt Frequencies*/
    //Stepper Motor timer interrupt frequency
    int servo_motor_timer = 50000;

    const esp_timer_create_args_t my_timer_args = {
        .callback = &servo_motor_timer_callback,
        .name = "Servo Motor Timer"
    };

    //If there are multiple timer interrupts, will need multiple "timer_handler"s. So they don't conflict 
    //  Ex: timer_handler_1, timer_handler_2, etc.
    esp_timer_handle_t timer_handler;

    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
    // Set to cycle at a rate of 50000 microseconds = 50 ms
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, servo_motor_timer));

    while (true)
    {
        esp_timer_dump(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000000));
    }

}

//Functions
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
void servo_motor_timer_callback(void *param)
{
    //Timer callback to drive the servo motor and blink onboard LED
    
    //servo_coordinates 0 means release -> angle is 90? and 1 means engage -> angle is 0?
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

    if (cnt_x == servo_state_size)
    {
        //Reset to Zero
        cnt_x = 0;
    }
    else
    {
        //Increment the Value
        cnt_x = cnt_x + 1;
    }

}
