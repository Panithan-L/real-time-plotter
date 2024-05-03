/* File: tcp_server_ultrasonic.c
Notes: Reading ultrasonic sensor values
        5/2/2024 - Received file from Sanzhar
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
    while (1)
    {
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            printf("notification received and reading ready\n");
            break;
        }
        // printf("inside loop\n");
        // vTaskDelay(50);
    }
    float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());

    // convert the pulse width into measure distance
    float distance = (float)pulse_width_us / 58;
    dist_data_buf = distance;
    ESP_LOGI(TAG, "Measured distance: %.2fcm", distance);

    // vTaskDelay(pdMS_TO_TICKS(500));
}

// void getSensorData()
// {
//     uint32_t tof_ticks;
//     // trigger the sensor to start a new sample
//     gen_trig_output();
//     // wait for echo done signal
//     printf("223\n");
//     xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000));

//     printf("226\n");
//     float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());

//     if (pulse_width_us <= 35000)
//     {
//         // convert the pulse width into measure distance
//         float distance = (float)pulse_width_us / 58;
//         dist_data_buf = distance;
//         ESP_LOGI(TAG, "Measured distance: %.2fcm", distance);
//     }

//     vTaskDelay(pdMS_TO_TICKS(500));
// }

static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[128];
    float LstartX, LstartY, LendX, LendY;
    float RstartX, RstartY, Rlength, Rwidth;
    float PcenterX, PcenterY, Pradius, Psides;
    float Line[4];
    float Rectangle[8];

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

                // current_blink_period -= 500;
                // if (current_blink_period <= 500)
                //     current_blink_period = 500;
                // esp_timer_restart(timer_handler, current_blink_period);

                ESP_LOGI(TAG, "Processing %s ", rx_buffer);
                break;
            }
            case 'B':
            case 'b':
            {
                strcpy((char *)rx_buffer, "BB Command Received\r\n");
                len = strlen((char *)rx_buffer);
                
                // current_blink_period = blink_period;
                // esp_timer_restart(timer_handler, current_blink_period);
                
                ESP_LOGI(TAG, "Processing %s ", rx_buffer);

                break;
            }
            case 'C':
            case 'c':
            {
                strcpy((char *)rx_buffer, "CCAcquisition started\r\n");
                len = strlen((char *)rx_buffer);
                ESP_LOGI(TAG, "Processing %s ", rx_buffer);
                
                // if (Acquisition == false)
                // {
                //     ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, blink_period));
                //     Acquisition = true;
                // }
                
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
                    Line[0] = LstartX * 0.625;
                    Line[1] = LstartY * 0.625;
                    Line[2] = LendX * 0.625;
                    Line[3] = LendY * 0.625;

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
                    Rlength = Rlength * 0.625;
                    Rwidth = Rwidth * 0.625;
                    Rectangle[0] = RstartX * 0.625;        // x1
                    Rectangle[1] = RstartY * 0.625;        // y1
                    Rectangle[2] = Rectangle[0];           // x2 = x1
                    Rectangle[3] = Rectangle[1] + Rlength; // y2 = y1+len
                    Rectangle[4] = Rectangle[0] + Rwidth;  // x3 = x1+w
                    Rectangle[5] = Rectangle[3];           // y3 = y2
                    Rectangle[4] = Rectangle[4] + Rwidth;  // x4 =x3
                    Rectangle[5] = Rectangle[1];           // y4 = y1

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
                    float PolygonX[n], PolygonY[n];  

                    for (int i = 0; i < n; i++)
                    {
                        PolygonX[i] = PcenterX + Pradius * cos(2 * PI * i / n);
                        PolygonY[i] = PcenterY + Pradius * sin(2 * PI * i / n);
                    }

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
    while (1)
        ;

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
