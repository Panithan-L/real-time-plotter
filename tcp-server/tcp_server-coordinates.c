/* File: tcp_server_ultrasonic.c
Notes: Reading ultrasonic sensor values
        5/3/2024 - Received file from Sanzhar
        5/4/2024 - Finished?
*/

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "math.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/i2c.h"

#include "esp_timer.h"
#include "driver/gpio.h"

#include "driver/mcpwm_cap.h"
#include "esp_private/esp_clk.h"

/********/
//From stepper-servo-motors_timer-interrupt.c
#include "driver/mcpwm_prelude.h"

//Math Library for rounding
#include "math.h"
/********/

TaskHandle_t tcp_server_task_handle;

#define PI 3.14159265358979323846
#define HC_SR04_TRIG_GPIO 25
#define HC_SR04_ECHO_GPIO 34

#define I2C_MASTER_SCL_IO 20        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 22        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define MPU9250_SENSOR_ADDR 0x6B       // 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR 0x0F /*!< Register addresses of the "who am I" register */

#define NO_OF_VALUES_IN_ONE_GO 6
float dist_data_buf;

#define MPU9250_PWR_MGMT_1_REG_ADDR 0x12 /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT 0

#define PORT CONFIG_EXAMPLE_PORT
#define KEEPALIVE_IDLE CONFIG_EXAMPLE_KEEPALIVE_IDLE
#define KEEPALIVE_INTERVAL CONFIG_EXAMPLE_KEEPALIVE_INTERVAL
#define KEEPALIVE_COUNT CONFIG_EXAMPLE_KEEPALIVE_COUNT

static const char *TAG = "tcp-server-i2c-simple-example";

int32_t blink_period = 2000;         // Initial blink period = 1s, each tick is 1 us
int32_t current_blink_period = 2000; // Initial blink period = 1s

int8_t Acquisition = false;

uint32_t ramp = 0;

esp_timer_handle_t timer_handler;

/**********************************************************************************/
//Code from stepper-servo-motors_timer-interrupt.c

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

#define SERVO_ENGAGE        1
#define SERVO_DISENGAGE     0

//Global Variables
volatile int current_xstep = STEPPER_IN_X1;
volatile int current_ystep = STEPPER_IN_Y1;

//Pin output step[] = {STEPPER_IN_X1, STEPPER_IN_X2, STEPPER_IN_X3, STEPPER_IN_X1}
volatile int step[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

volatile int x_current = 0;
volatile int cnt_x = 0;

volatile int y_current = 0;
volatile int cnt_y = 0;

// X and Y Coordinates Arrays, also Servo Coordinate Array
#define coordinate_array_size 50

volatile int x_coordinates[coordinate_array_size];
volatile int y_coordinates[coordinate_array_size];
volatile int servo_coordinates[coordinate_array_size];

volatile int x_cnt_write = 0;
volatile int y_cnt_write = 0;
volatile int servo_cnt_write = 0;

// //Calculations
// //Calculating the array sizes of x_coordinates, y_coordinates, and servo_coordinates
// volatile int x_size = sizeof(x_coordinates) / sizeof(x_coordinates[0]);
// volatile int y_size = sizeof(y_coordinates) / sizeof(y_coordinates[0]);
// volatile int servo_state_size = sizeof(servo_coordinates) / sizeof(servo_coordinates[0]);

//Global Structure ...
volatile mcpwm_cmpr_handle_t comparator = NULL;


//Function Prototypes
void stepper_setup(int coil_in_1, int coil_in_2, int coil_in_3, int coil_in_4);
static inline uint32_t pulse_from_angle(int target_angle);
void servo_pwm_setup(int pwm_period, int pwm_resolution_hz, int servo_pwm_gpio_pin);

void write_coordinate(int new_x_element, int new_y_element, int new_servo_position);

//Interrupt Service Routine [ISR] Prototypes
void stepper_servo_motor_timer_callback(void *param);

/************************************************************************************************/


/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
/*
static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
*/

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
/*
static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}
*/
/**
 * @brief i2c master initialization
 */
/*
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
*/
static bool ON = 1;
static uint8_t data[12];

static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t cap_val_begin_of_sample = 0;
    static uint32_t cap_val_end_of_sample = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    // calculate the interval in the ISR,
    // so that the interval will be always correct even when capture_queue is not handled in time and overflow.
    if (edata->cap_edge == MCPWM_CAP_EDGE_POS)
    {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    }
    else
    {
        cap_val_end_of_sample = edata->cap_value;
        uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;

        // notify the task to calculate the distance
        xTaskNotifyFromISR(task_to_notify, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

/**
 * @brief generate single pulse on Trig pin to start a new sample
 */
static void gen_trig_output(void)
{
    gpio_set_level(HC_SR04_TRIG_GPIO, 1); // set high
    esp_rom_delay_us(10);
    gpio_set_level(HC_SR04_TRIG_GPIO, 0); // set low
}

void ultrasonic_init(void)
{
    ESP_LOGI(TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(TAG, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = HC_SR04_ECHO_GPIO,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(TAG, "Register capture callback");
    TaskHandle_t cur_task = tcp_server_task_handle;
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Configure Trig pin");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO, 0));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));
}

static void timer_callback(void *param)
{
    gpio_set_level(13, ON);
    ON = !ON;
    // ESP_LOGI(TAG, "GPIO state %d", ON);
    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
    // ESP_ERROR_CHECK(mpu9250_register_read(0x22, data, 12));
    ramp = (ramp + 1) & 0x0FFF;
    if (ramp == 4096)
        ramp = 0;
    // gpio_set_level(13, 0);    
}

void getSensorData()
{
    uint32_t tof_ticks;
    // trigger the sensor to start a new sample
    gen_trig_output();
    // wait for echo done signal
    printf("223\n");

    if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        printf("Notification received and reading ready \n");
    }

    // while (1)
    // {
    //     if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE)
    //     {
    //         printf("notification received and reading ready\n");
    //         break;
    //     }
    //     // printf("inside loop\n");
    //     // vTaskDelay(50);
    // }
    float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());

    // convert the pulse width into measure distance
    float distance = (float)pulse_width_us / 58;
    dist_data_buf = distance;
    ESP_LOGI(TAG, "Measured distance: %.2fcm", distance);

    // vTaskDelay(pdMS_TO_TICKS(500));
}


static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[128];
    float LstartX, LstartY, LendX, LendY;
    float RstartX, RstartY, Rlength, Rwidth;
    float PcenterX, PcenterY, Pradius, Psides;
    int Line[4];
    int Rectangle[8];

    do
    {

        gpio_set_level(27, 1);
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        gpio_set_level(27, 0);
        if (len < 0)
        {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        }
        else if (len == 0)
        {
            ESP_LOGW(TAG, "Connection closed");
        }
        else
        {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

            switch (rx_buffer[0])
            {
            case 0:
            {

                break;
            }
            case 'T':
            case 't':
            {
                strcpy((char *)rx_buffer, "TToggle Command Received\r\n");

                ESP_LOGI(TAG, "Processing %s Blink period %ld us", rx_buffer, current_blink_period);
                len = strlen((char *)rx_buffer);
                break;
            }

            case 'A':
            case 'a':
            {
                strcpy((char *)rx_buffer, "AA Command Received\r\n");
                len = strlen((char *)rx_buffer);

                current_blink_period -= 500;
                if (current_blink_period <= 500)
                    current_blink_period = 500;
                esp_timer_restart(timer_handler, current_blink_period);

                ESP_LOGI(TAG, "Processing %s ", rx_buffer);
                break;
            }
            case 'B':
            case 'b':
            {
                strcpy((char *)rx_buffer, "BB Command Received\r\n");
                len = strlen((char *)rx_buffer);
                
                current_blink_period = blink_period;
                esp_timer_restart(timer_handler, current_blink_period);
                
                ESP_LOGI(TAG, "Processing %s ", rx_buffer);

                break;
            }
            case 'C':
            case 'c':
            {
                strcpy((char *)rx_buffer, "CCAcquisition started\r\n");
                len = strlen((char *)rx_buffer);
                ESP_LOGI(TAG, "Processing %s ", rx_buffer);
                
                if (Acquisition == false)
                {
                    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, blink_period));
                    Acquisition = true;
                }
                
                break;
            }
            case 'D':
            case 'd':
            {
                strcpy((char *)rx_buffer, "DDAcquisition paused\r\n");
                len = strlen((char *)rx_buffer);
                ESP_LOGI(TAG, "Processing %s ", rx_buffer);
                if (Acquisition == true)
                {
                    ESP_ERROR_CHECK(esp_timer_stop(timer_handler));
                    Acquisition = false;
                }
                break;
            }
            case 'G':
            case 'g':
            {
                getSensorData();
                sprintf((char *)rx_buffer, "GGGet Data %4lX\r\n", (uint32_t)(dist_data_buf * 1000));
                len = strlen((char *)rx_buffer);
                ESP_LOGI(TAG, "Processing %d ", len);
                ESP_LOGI(TAG, "Gyro = %X,%X,%X,%X,%X,%X, Accel = %X,%X,%X,%X,%X,%X", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11]);
                break;
            }
            case 'I':
            case 'i':
            {
                strcpy((char *)rx_buffer, "IESP32\r\n");
                ESP_LOGI(TAG, "Processing %s", rx_buffer);
                len = strlen((char *)rx_buffer);

                break;
            }
            case 'X':
            case 'x':
            {
                strcpy((char *)rx_buffer, "XXESP32 Stopped\r\n");
                ESP_LOGI(TAG, "Processing %s", rx_buffer);
                len = strlen((char *)rx_buffer);
                if (Acquisition == true)
                {
                    ESP_ERROR_CHECK(esp_timer_stop(timer_handler));
                    Acquisition = false;
                }

                break;
            }
            case 'L':
            case 'l':
            {
                if (sscanf(rx_buffer, "L,%f,%f,%f,%f", &LstartX, &LstartY, &LendX, &LendY) == 4)
                {
                    printf("Drawing line from [%f, %f] to [%f, %f] \n", LstartX, LstartY, LendX, LendY);
                    
                    Line[0] = round(LstartX * 0.625);
                    Line[1] = round(LstartY * 0.625);
                    Line[2] = round(LendX * 0.625);
                    Line[3] = round(LendY * 0.625);

                    printf("Calculated points to be drawn \n");

                    //Writing the x, y and servo values to the arrays
                    //Updating the start of the line [x1, y1, servo_position]
                    write_coordinate(Line[0], Line[1], SERVO_DISENGAGE);
                    write_coordinate(Line[0], Line[1], SERVO_ENGAGE);

                    //Updating the end of the line [x2, y2, servo_position]
                    write_coordinate(Line[2], Line[3], SERVO_ENGAGE);
                    
                    //Replace these with updating x_coordinates, y_coordinates, and servo_coordinates

                    // xRingbufferSend(ringbuf_handle_x, &Line[0], ITEM_SIZE, pdMS_TO_TICKS(1000));
                    // xRingbufferSend(ringbuf_handle_y, &Line[1], ITEM_SIZE, pdMS_TO_TICKS(1000));       
                    // xRingbufferSend(ringbuf_handle_x, &Line[2], ITEM_SIZE, pdMS_TO_TICKS(1000));
                    // xRingbufferSend(ringbuf_handle_y, &Line[3], ITEM_SIZE, pdMS_TO_TICKS(1000));

                    strcpy((char *)rx_buffer, "AA Command Received\r\n");
                    len = strlen((char *)rx_buffer);
                    current_blink_period -= 500;
                    if (current_blink_period <= 500)
                        current_blink_period = 500;
                    esp_timer_restart(timer_handler, current_blink_period);
                    ESP_LOGI(TAG, "Processing %s: Start (%f, %f) End (%f, %f)", rx_buffer, LstartX, LstartY, LendX, LendY);
                }
                else
                {
                    // Parsing error
                    ESP_LOGE(TAG, "Error parsing line coordinates from command '%s'", rx_buffer);
                }
                break;
            }

            case 'R':
            case 'r':
            {

                if (sscanf(rx_buffer, "R,%f,%f,%f,%f", &RstartX, &RstartY, &Rlength, &Rwidth) == 4)
                {
                    //Converting to Step Increments
                    int X_start = round(RstartX * 0.625);
                    int Y_start = round(RstartY * 0.625);
                    int rectangle_length = round(Rlength * 0.625);
                    int rectangle_width = round(Rwidth * 0.625);
                    
                    Rectangle[0] = X_start;                         // x1 = x_start
                    Rectangle[1] = Y_start;                         // y1 = y_start
                    Rectangle[2] = X_start + rectangle_width;       // x2 = x_start + width
                    Rectangle[3] = Y_start;                         // y2 = y_start
                    Rectangle[4] = X_start + rectangle_width;       // x3 = x_start + width
                    Rectangle[5] = Y_start + rectangle_length;      // y3 = y_start + length
                    Rectangle[6] = X_start;                         // x4 = x_start
                    Rectangle[7] = Y_start + rectangle_length;      // y4 = y_start + length

                    //Writing the x, y and servo values to the arrays
                    //Rectangle Edge 1 [x1, y1, servo_position]
                    write_coordinate(Rectangle[0], Rectangle[1], SERVO_DISENGAGE);
                    write_coordinate(Rectangle[0], Rectangle[1], SERVO_ENGAGE);


                    //Rectangle Edge 2 [x2, y2, servo_position]
                    write_coordinate(Rectangle[2], Rectangle[3], SERVO_ENGAGE);
                    //Rectangle Edge 3 [x3, y3, servo_position]
                    write_coordinate(Rectangle[4], Rectangle[5], SERVO_ENGAGE);
                    //Rectangle Edge 4 [x4, y4, servo_position]
                    write_coordinate(Rectangle[6], Rectangle[7], SERVO_ENGAGE);

                    //Loop back to complete the rectangle [x5 and y5]
                    //Rectangle Edge 1 [x5, y5, servo_position]
                    write_coordinate(Rectangle[0], Rectangle[1], SERVO_ENGAGE);


                    // Replace these with updating x_coordinates, y_coordinates, and servo_coordinates
                    
                    // xRingbufferSend(ringbuf_handle_x, &Rectangle[0], ITEM_SIZE, pdMS_TO_TICKS(1000));
                    // xRingbufferSend(ringbuf_handle_y, &Rectangle[1], ITEM_SIZE, pdMS_TO_TICKS(1000));       
                    // xRingbufferSend(ringbuf_handle_x, &Rectangle[2], ITEM_SIZE, pdMS_TO_TICKS(1000));
                    // xRingbufferSend(ringbuf_handle_y, &Rectangle[3], ITEM_SIZE, pdMS_TO_TICKS(1000));   
                    // xRingbufferSend(ringbuf_handle_x, &Rectangle[4], ITEM_SIZE, pdMS_TO_TICKS(1000));
                    // xRingbufferSend(ringbuf_handle_y, &Rectangle[5], ITEM_SIZE, pdMS_TO_TICKS(1000));       
                    // xRingbufferSend(ringbuf_handle_x, &Rectangle[6], ITEM_SIZE, pdMS_TO_TICKS(1000));
                    // xRingbufferSend(ringbuf_handle_y, &Rectangle[7], ITEM_SIZE, pdMS_TO_TICKS(1000));              

                    strcpy((char *)rx_buffer, "AA Command Received\r\n");
                    len = strlen((char *)rx_buffer);
                    current_blink_period -= 500;
                    if (current_blink_period <= 500)
                        current_blink_period = 500;
                    esp_timer_restart(timer_handler, current_blink_period);
                    ESP_LOGI(TAG, "Processing %s: Start (%f, %f) End (%f, %f)", rx_buffer, LstartX, LstartY, LendX, LendY);
                }
                else
                {
                    // Parsing error
                    ESP_LOGE(TAG, "Error parsing line coordinates from command '%s'", rx_buffer);
                }
                break;
            }

            case 'P':
            case 'p':
            {

                if (sscanf(rx_buffer, "P,%f,%f,%f,%f", &PcenterX, &PcenterY, &Pradius, &Psides) == 4)
                {
                    PcenterX = PcenterX * 0.625;
                    PcenterY = PcenterY * 0.625;
                    Pradius = Pradius * 0.625;
                    int n = (int)Psides; 
                    int PolygonX[n], PolygonY[n];  

                    for (int i = 0; i < n; i++)
                    {
                        PolygonX[i] = round(PcenterX + Pradius * cos(2 * PI * i / n));
                        PolygonY[i] = round(PcenterY + Pradius * sin(2 * PI * i / n));

                        //Writing the x, y and servo values to the arrays
                        if (i == 0)
                        {
                            //Polygon Edge 1 [x1, y1, servo_position]
                            write_coordinate(PolygonX[0], PolygonY[0], SERVO_DISENGAGE);
                            write_coordinate(PolygonX[0], PolygonY[0], SERVO_ENGAGE);
                        }
                        else
                        {
                            //Polygon Edge 2 to Edge n 
                            write_coordinate(PolygonX[i], PolygonY[i], SERVO_ENGAGE);
                        }

                        //Replace these with updating x_coordinates, y_coordinates, and servo_coordinates

                        // xRingbufferSend(ringbuf_handle_x, &PolygonX[i], ITEM_SIZE, pdMS_TO_TICKS(1000));
                        // xRingbufferSend(ringbuf_handle_y, &PolygonY[i], ITEM_SIZE, pdMS_TO_TICKS(1000));
                    }

                    //Loop back to complete the polygon [x(n + 1) and y(n + 1)]
                    //Polygon Edge 1 [x(n + 1), y(n + 1), servo_position]
                    write_coordinate(PolygonX[0], PolygonY[0], SERVO_ENGAGE);

                    strcpy((char *)rx_buffer, "AA Command Received\r\n");
                    len = strlen((char *)rx_buffer);
                    current_blink_period -= 500;
                    if (current_blink_period <= 500)
                        current_blink_period = 500;
                    esp_timer_restart(timer_handler, current_blink_period);
                    ESP_LOGI(TAG, "Processing %s: Start (%f, %f) End (%f, %f)", rx_buffer, LstartX, LstartY, LendX, LendY);
                }
                else
                {
                    // Parsing error
                    ESP_LOGE(TAG, "Error parsing line coordinates from command '%s'", rx_buffer);
                }
                break;
            }

            default:
            {

                break;
            }
            }

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.
            int to_write = len;
            while (to_write > 0)
            {
                printf("405\n");
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    // Failed to retransmit, giving up
                    return;
                }
                to_write -= written;
            }
        }

    } while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

#ifdef CONFIG_EXAMPLE_IPV4
    if (addr_family == AF_INET)
    {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    if (addr_family == AF_INET6)
    {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1)
    {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
#ifdef CONFIG_EXAMPLE_IPV4
        if (source_addr.ss_family == PF_INET)
        {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
#ifdef CONFIG_EXAMPLE_IPV6
        if (source_addr.ss_family == PF_INET6)
        {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_rom_gpio_pad_select_gpio(13);
    esp_rom_gpio_pad_select_gpio(27);
    gpio_set_direction(13, GPIO_MODE_OUTPUT);
    gpio_set_direction(27, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(2); // Setting up gpio pin2 to turn on power to qwiic connector
    gpio_set_direction(2, GPIO_MODE_OUTPUT);

    gpio_set_level(2, 1); // Enable power to qwiic connector

    /***********************************************************************************************/
    //Initial setup for Stepper Motor GPIO Pins
    // X Axis Stepper setup
    stepper_setup(STEPPER_IN_X1, STEPPER_IN_X2, STEPPER_IN_X3, STEPPER_IN_X4);
    // Y Axis Stepper setup
    stepper_setup(STEPPER_IN_Y1, STEPPER_IN_Y2, STEPPER_IN_Y3, STEPPER_IN_Y4);

    //Setting up the PWM for the servo motor
    servo_pwm_setup(SERVO_TIMEBASE_PERIOD, SERVO_TIMEBASE_RESOLUTION_HZ, SERVO_PWM_GPIO);

    /**************************************************************************************************/


    //   ESP_ERROR_CHECK(i2c_master_init());
    //   ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
    //   ESP_ERROR_CHECK(mpu9250_register_read(MPU9250_WHO_AM_I_REG_ADDR, data, 1));
    //   ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Enable the accelerometers */
    //    ESP_ERROR_CHECK(mpu9250_register_write_byte(0x10, 0xA0));
    //    ESP_ERROR_CHECK(mpu9250_register_read(0x10, data, 1));
    //    ESP_LOGI(TAG, "REG 0x10 = %X", data[0]);

    /* Enable the gyrometers */
    //    ESP_ERROR_CHECK(mpu9250_register_write_byte(0x11, 0xA0));
    //    ESP_ERROR_CHECK(mpu9250_register_read(0x11, data, 1));
    //    ESP_LOGI(TAG, "REG 0x11 = %X", data[0]);

    /* Demonstrate writing by reseting the MPU9250 */
    // ESP_ERROR_CHECK(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

    const esp_timer_create_args_t my_timer_args = {
        .callback = &timer_callback,
        .name = "My Timer"};

    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /**********************************************************************************************/
    /*Initializing Timer Interrupt Frequencies*/
    //Stepper Motor timer interrupt frequency, in microseconds
    int stepper_servo_motor_timer = 10000;

    const esp_timer_create_args_t my_motor_timer_args = {
        .callback = &stepper_servo_motor_timer_callback,
        .name = "Stepper Motor Timer"
    };

    //If there are multiple timer interrupts, will need multiple "timer_handler"s. So they don't conflict 
    //  Ex: timer_handler_1, timer_handler_2, etc.
    esp_timer_handle_t timer_handler;

    ESP_ERROR_CHECK(esp_timer_create(&my_motor_timer_args, &timer_handler));
    // Set to cycle at a rate of 50000 microseconds = 50 ms
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, stepper_servo_motor_timer));
    /**********************************************************************************************/



    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */

    ESP_ERROR_CHECK(example_connect());

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void *)AF_INET, 2, &tcp_server_task_handle);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void *)AF_INET6, 5, &tcp_server_task_handle);
#endif
    ultrasonic_init();

    // Example to setting priority
    // xTaskCreatePinnedToCore(ultrasonic_data_lid, "lid_distance", 4096, NULL, 5, NULL, 0);
    // xTaskCreatePinnedToCore(getSensorData, "Ultrasonic", 4096, NULL, 3, NULL, 0);

    while (1)
        ;

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}


/**********************************************************************************/
//Code from stepper-servo-motors_timer-interrupt.c

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

//Function to write values to the coordinate arrays [servo position: 1 is pen down, 0 is pen up]
void write_coordinate(int new_x_element, int new_y_element, int new_servo_position)
{
    //Writing the new values to the arrays of x coordinates, y coordinates, and servo positions
    x_coordinates[x_cnt_write] = new_x_element;
    y_coordinates[y_cnt_write] = new_y_element;
    servo_coordinates[servo_cnt_write] = new_servo_position;

    //Updating the write counters
    if((x_cnt_write == coordinate_array_size) && (y_cnt_write == coordinate_array_size) && (servo_cnt_write == coordinate_array_size))
    {
        //Reset the write counters to 0, if they have reached the end
        x_cnt_write = 0;
        y_cnt_write = 0;
        servo_cnt_write = 0;
    }
    else
    {
        x_cnt_write++;
        y_cnt_write++;
        servo_cnt_write++;
    }

    printf("Added the following x and y coordinates: %d, %d \n", new_x_element, new_y_element);
    
}


//Interrupt Service Routines [ISR]
void stepper_servo_motor_timer_callback(void *param)
{
    //Timer callback to drive the stepper motor and blink onboard LED
    printf("X and Y Coordinates: %d, %d \n", x_coordinates[cnt_x], y_coordinates[cnt_y]);

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
        // printf("Angle: 0 \n");
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_from_angle(0)));
        break;
    
    case 0:
        // printf("Angle: 90 \n");
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_from_angle(90)));
        break;
    }
    

    //Updating Count Values
    if ((x_current == x_coordinates[cnt_x]) && (y_current == y_coordinates[cnt_y]))
    {
        //Clearing values to coordinate array
        x_coordinates[cnt_x] = 0;
        y_coordinates[cnt_y] = 0;
        servo_coordinates[cnt_x] = SERVO_DISENGAGE;
        
        //Increase count x and count y
        cnt_x++;
        cnt_y++;
    }
    else if ((cnt_x == coordinate_array_size) && (cnt_y == coordinate_array_size))
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
