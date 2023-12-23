/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "driver/mcpwm_timer.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"

#include "esp_attr.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/i2c.h"
#include <string.h>

// includes for ultrasonic sensor
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

char buff[128];

#define HC_SR04_TRIG_GPIORIGHT  25
#define HC_SR04_ECHO_GPIORIGHT  26
#define HC_SR04_TRIG_GPIOLEFT  4
#define HC_SR04_ECHO_GPIOLEFT  36

// includes for the alpha display
#include <stdio.h>
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

static const uint16_t alphafonttable[] = {
    0b0000000000000001, 0b0000000000000010, 0b0000000000000100,
    0b0000000000001000, 0b0000000000010000, 0b0000000000100000,
    0b0000000001000000, 0b0000000010000000, 0b0000000100000000,
    0b0000001000000000, 0b0000010000000000, 0b0000100000000000,
    0b0001000000000000, 0b0010000000000000, 0b0100000000000000,
    0b1000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0001001011001001, 0b0001010111000000, 0b0001001011111001,
    0b0000000011100011, 0b0000010100110000, 0b0001001011001000,
    0b0011101000000000, 0b0001011100000000,
    0b0000000000000000, //
    0b0000000000000110, // !
    0b0000001000100000, // "
    0b0001001011001110, // #
    0b0001001011101101, // $
    0b0000110000100100, // %
    0b0010001101011101, // &
    0b0000010000000000, // '
    0b0010010000000000, // (
    0b0000100100000000, // )
    0b0011111111000000, // *
    0b0001001011000000, // +
    0b0000100000000000, // ,
    0b0000000011000000, // -
    0b0100000000000000, // .
    0b0000110000000000, // /
    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
    0b0001001000000000, // :
    0b0000101000000000, // ;
    0b0010010000000000, // <
    0b0000000011001000, // =
    0b0000100100000000, // >
    0b0001000010000011, // ?
    0b0000001010111011, // @
    0b0000000011110111, // A
    0b0001001010001111, // B
    0b0000000000111001, // C
    0b0001001000001111, // D
    0b0000000011111001, // E
    0b0000000001110001, // F
    0b0000000010111101, // G
    0b0000000011110110, // H
    0b0001001000001001, // I
    0b0000000000011110, // J
    0b0010010001110000, // K
    0b0000000000111000, // L
    0b0000010100110110, // M
    0b0010000100110110, // N
    0b0000000000111111, // O
    0b0000000011110011, // P
    0b0010000000111111, // Q
    0b0010000011110011, // R
    0b0000000011101101, // S
    0b0001001000000001, // T
    0b0000000000111110, // U
    0b0000110000110000, // V
    0b0010100000110110, // W
    0b0010110100000000, // X
    0b0001010100000000, // Y
    0b0000110000001001, // Z
    0b0000000000111001, // [
    0b0010000100000000, //
    0b0000000000001111, // ]
    0b0000110000000011, // ^
    0b0000000000001000, // _
    0b0000000100000000, // `
    0b0001000001011000, // a
    0b0010000001111000, // b
    0b0000000011011000, // c
    0b0000100010001110, // d
    0b0000100001011000, // e
    0b0000000001110001, // f
    0b0000010010001110, // g
    0b0001000001110000, // h
    0b0001000000000000, // i
    0b0000000000001110, // j
    0b0011011000000000, // k
    0b0000000000110000, // l
    0b0001000011010100, // m
    0b0001000001010000, // n
    0b0000000011011100, // o
    0b0000000101110000, // p
    0b0000010010000110, // q
    0b0000000001010000, // r
    0b0010000010001000, // s
    0b0000000001111000, // t
    0b0000000000011100, // u
    0b0010000000000100, // v
    0b0010100000010100, // w
    0b0010100011000000, // x
    0b0010000000001100, // y
    0b0000100001001000, // z
    0b0000100101001001, // {
    0b0001001000000000, // |
    0b0010010010001001, // }
    0b0000010100100000, // ~
    0b0011111111111111,
};

static const char *TAG = "example";

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_PULSE_GPIO             13        // GPIO connects to the PWM signal line
#define ESC_PULSE_GPIO             15        // GPIO connects to the PWM signal line

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define ESC_MIN 900
#define ESC_MAX 2100
#define ESC_NEUTRAL 1500

// For ADC
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

// For ADC
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1 (A2)
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

bool pulsed = false;

int count;

float speed_cnt = 0;

float speed = 0;

volatile int wall = 0;

// PID variables
float dist_read_right = 0;
float dist_read_left = 0;

enum turns
  {
    TURN_RIGHT,
    TURN_LEFT,
    STRAIGHT,
  };

enum turns pid_turn = STRAIGHT;

// Alpha variables
char alpha_str[20];

// 360 turn
int rev_direction = 0;

// Force stop
int e_brake = 0;

//////////////////////////////////////////WIFI//////////////////////////////////////////
#define WIFI_SSID      "Group_7"
#define WIFI_PASSWORD  "smartsys"
#define UDP_SERVER_IP  "192.168.1.23"
#define UDP_PORT       3333
#define PORT       3333
#define ESP32_HOSTNAME "ESP32"
// static const char *TAG = "UDP_CLIENT";

static void wifi_init(void);
static void udp_client_task(void *pvParameters);

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi started, trying to connect...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "Connected to WiFi successfully!");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();  // Reconnect upon disconnection
        ESP_LOGI(TAG, "Disconnected from WiFi. Trying to reconnect...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: %s", ip4addr_ntoa(&event->ip_info.ip));
    }
}
static void wifi_init(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Create a default WiFi STA network interface instance
    esp_netif_create_default_wifi_sta();
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    
    // Set hostname for the STA interface
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif) {
        esp_netif_set_hostname(sta_netif, ESP32_HOSTNAME);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init finished.");
}

//////////////////////////////////////////////LIDAR//////////////////////////////////////////////
// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// LIDARLITE
#define LIDARLITE_ADDR_DEFAULT 0x62

static void i2c_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    conf.clk_flags = 0;
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                        I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                        I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK) {printf("- initialized: yes\n");}

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
        printf( "- Device found at address: 0x%X%s", i, "\n");
        count++;
        }
    }
    if (count == 0) {printf("- No I2C devices found!" "\n");}
}


// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
    //adapted from esp idf i2c_simple_main program
    uint8_t write_buf[2] = {reg, data};
    int ret;
    ret = i2c_master_write_to_device(I2C_EXAMPLE_MASTER_NUM, LIDARLITE_ADDR_DEFAULT, write_buf, sizeof(write_buf), 1000/ portTICK_PERIOD_MS);
    return ret;
}

// Read register
uint8_t readRegister(uint8_t reg) {
    //adapted from esp idf i2c_simple_main program
    uint8_t data;
    i2c_master_write_read_device(I2C_EXAMPLE_MASTER_NUM, LIDARLITE_ADDR_DEFAULT, &reg, 1, &data, 1, 1000 / portTICK_PERIOD_MS);
    return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
    // YOUR CODE HERE
    //adapted from esp idf i2c_simple_main program

    int16_t data, MSB, LSB;
    
    LSB = readRegister(reg);
    MSB = readRegister(reg + 1);
    data = MSB;
    data = LSB | (MSB << 8);
    return data;
}
//   configuration:  Default 0.
//     0: Maximum range. Uses maximum acquisition count.
//     1: Balanced performance.
//     2: Short range, high speed. Reduces maximum acquisition count.
//     3: Mid range, higher speed. Turns on quick termination
//          detection for faster measurements at short range (with decreased
//          accuracy)
//     4: Maximum range, higher speed on short range targets. Turns on quick
//          termination detection for faster measurements at short range (with
//          decreased accuracy)1
//     5: Very short range, higher speed, high error. Reduces maximum
//          acquisition count to a minimum for faster rep rates on very
//          close targets with high error.
void configure(uint8_t configuration, uint8_t lidarliteAddress)
{
    uint8_t sigCountMax = 0;
    uint8_t acqConfigReg = 0;

    switch (configuration)
    {
        case 0: // Default mode - Maximum range
            sigCountMax     = 0xff;
            acqConfigReg    = 0x08;
            break;

        case 1: // Balanced performance
            sigCountMax     = 0x80;
            acqConfigReg    = 0x08;
            break;

        case 2: // Short range, high speed
            sigCountMax     = 0x18;
            acqConfigReg    = 0x00;
            break;

        case 3: // Mid range, higher speed on short range targets
            sigCountMax     = 0x80;
            acqConfigReg    = 0x00;
            break;

        case 4: // Maximum range, higher speed on short range targets
            sigCountMax     = 0xff;
            acqConfigReg    = 0x00;
            break;

        case 5: // Very short range, higher speed, high error
            sigCountMax     = 0x04;
            acqConfigReg    = 0x00;
            break;
    }

    writeRegister(0x05, sigCountMax);
    writeRegister(0xE5, acqConfigReg);
}

uint8_t getBusyFlag(uint8_t lidarliteAddress)
{
    uint8_t  statusByte = 0;
    uint8_t  busyFlag; // busyFlag monitors when the device is done with a measurement

    // Read status register to check busy flag
    statusByte = readRegister(0x01);

    // STATUS bit 0 is busyFlag
    busyFlag = statusByte & 0x01;
    return busyFlag;
}

void waitForBusy(uint8_t lidarliteAddress)
{
    uint8_t  busyFlag;

    do
    {
        busyFlag = getBusyFlag(lidarliteAddress);
    } while (busyFlag);

}

void takeRange(uint8_t lidarliteAddress)
{
    uint8_t dataByte = 0x04;
    writeRegister(0x00, dataByte);
}


uint16_t readDistance(uint8_t lidarliteAddress)
{
    uint16_t distance;
    // Read two bytes from registers 0x10 and 0x11
    distance = read16(0x10);
    // printf("Reading %d: %d\n",i, distance);
    return distance;
}

/////////////////////////////////////////////Wheel Speed/////////////////////////////////////////////

// For ADC
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void timer() {
    while(1) {
        count = 0;
        vTaskDelay(100);
        printf("Count: %d\n", count);
        speed = (count * (2*3.14159*7/6))/100; //0.005; //(count / 6) * 0.62;
        printf("Speed is %.1fm/s\n", speed);
    }
}


static inline uint32_t example_angle_to_compare(int angle) {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

static inline uint32_t speed_to_compare(float speed) {
    uint32_t calc_speed;
    if(speed >= 0) {
        calc_speed = ((ESC_MAX - ESC_NEUTRAL) * speed) + ESC_NEUTRAL;
    } else {
        calc_speed = ((ESC_NEUTRAL - ESC_MIN) * speed) + ESC_NEUTRAL;
    }
    
    return calc_speed;
}

void speed_task(void *param) {
    // /* Install UART driver for interrupt-driven reads and writes */
    // ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );

    // /* Tell VFS to use UART driver */
    // esp_vfs_dev_uart_use_driver(UART_NUM_0);
    
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer2 = NULL;
    mcpwm_timer_config_t timer_config2 = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config2, &timer2));

    mcpwm_oper_handle_t oper2 = NULL;
    mcpwm_operator_config_t operator_config2 = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config2, &oper2));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer2));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator2 = NULL;
    mcpwm_comparator_config_t comparator_config2 = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &comparator_config2, &comparator2));

    mcpwm_gen_handle_t generator2 = NULL;
    mcpwm_generator_config_t generator_config2 = {
        .gen_gpio_num = ESC_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &generator_config2, &generator2));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator2, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer2));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer2, MCPWM_TIMER_START_NO_STOP));
        
    vTaskDelay(3500 / portTICK_PERIOD_MS); // Do for at least 3s, and leave in neutral state
    
    uint8_t *data = (uint8_t *) malloc(1024);
    char speed_buf[20] = "0";

    float previous_error = 0;
    float integral = 0;
    float derivative = 0;
    float dt = 100;
    float Kp = 1;
    float Ki = 0;
    float Kd = 1;
    float output;
    float error;

    float target = 1.0;

    while (1) {
        if(e_brake == 0) {
            if(rev_direction == 0) {
                if (wall == 1) {
                    speed_cnt = 0.0;
                    //ESP_LOGI(TAG, "Speed of buggy STOPPED: %f", speed_cnt);
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
                    vTaskDelay(pdMS_TO_TICKS(1000));
                } else {
                    error = target - speed;
                    integral = integral + error * dt;
                    derivative = (error - previous_error) / dt;
                    output = Kp * error + Ki * integral + Kd * derivative;
                    previous_error = error;
                    if(error < -0.2)
                    {
                        speed_cnt = 0.146;
                        //ESP_LOGI(TAG, "SPEED UP");
                    }
                    else if(error > 0.2)
                    {
                        speed_cnt = 0.158;
                        //ESP_LOGI(TAG, "SLOW DOWN");
                    }
                    else
                    {
                        continue;
                    }
                    ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
                    
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
                    vTaskDelay(pdMS_TO_TICKS(800));
                }
            } else {
                speed_cnt = 0;
                ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
                vTaskDelay(pdMS_TO_TICKS(2000));
                speed_cnt = -0.27;
                ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
                vTaskDelay(pdMS_TO_TICKS(100));
                speed_cnt = 0;
                ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
                vTaskDelay(pdMS_TO_TICKS(100));
                speed_cnt = -0.27;
                ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
                vTaskDelay(pdMS_TO_TICKS(750));
                speed_cnt = 0;
                ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
                vTaskDelay(pdMS_TO_TICKS(500));
                speed_cnt = 0.15;
                ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
                vTaskDelay(pdMS_TO_TICKS(750));
                if (wall == 1) {
                    rev_direction = 0;
                    continue;
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                speed_cnt = 0;
                ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
                vTaskDelay(pdMS_TO_TICKS(500));
                rev_direction = 0;
            }
        } else {
            speed_cnt = 0.0;
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
        }
    }
}

void encoder_task(void *arg)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    // Initialize counter
    count = 0;

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;

        if (adc_reading < 4095 && pulsed == false)
        {
            count++;
            pulsed = true;
        } else if (adc_reading == 4095) {
            pulsed = false;
        }

        // printf("Raw: %d\n", adc_reading);
        // printf("Count: %d\n", count);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void servo_task(void *param) {
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    int angle = 0;
    int step = 2;
    while (1) {
        if(e_brake == 0) {
            if(rev_direction == 0) {
                if(pid_turn == STRAIGHT) {
                    angle = -17;
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
                } else if(pid_turn == TURN_LEFT) {
                    angle = -11;
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
                } else if(pid_turn == TURN_RIGHT) {
                    angle = -23;
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
                }
            } else {
                angle = -17;
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
                vTaskDelay(pdMS_TO_TICKS(2000));
                angle = -17;
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
                vTaskDelay(pdMS_TO_TICKS(700));
                angle = -60; // reverse right
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
                vTaskDelay(pdMS_TO_TICKS(750));
                
                angle = -17;
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
                vTaskDelay(pdMS_TO_TICKS(300));
                
                angle = 40; // turn left forward
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
                vTaskDelay(pdMS_TO_TICKS(1750));
                angle = -17; // reset
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
                vTaskDelay(pdMS_TO_TICKS(500));
                rev_direction = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(25));
        } else {
            continue;
        }
    }
}

void lidar_task()
{
    printf("\n>> Polling LIDAR\n");
    int numTrials = 5;
    while (1) {
        int cumDistance = 0;
        for (int i = 0; i < numTrials; i++) {
            waitForBusy(LIDARLITE_ADDR_DEFAULT);
            takeRange(LIDARLITE_ADDR_DEFAULT);
            cumDistance += readDistance(LIDARLITE_ADDR_DEFAULT);
        }
        if(cumDistance / numTrials < 80)
        {
            speed_cnt = 0.0;
            ESP_LOGI(TAG, "WALL DETECTED");
            wall = 1;
            //ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
        } else {
            wall = 0;
        }
        printf("Distance: %d\n", cumDistance / numTrials);
        vTaskDelay(30/portTICK_PERIOD_MS);
    }
}

//////////////////////////////////////// Ultrasonic Sensor /////////////////////////////////////////////////////
static bool hc_sr04_echo_callback_left(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t cap_val_begin_of_sample_left = 0;
    static uint32_t cap_val_end_of_sample_left = 0;
    TaskHandle_t task_to_notify_left = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup_left = pdFALSE;

    //calculate the interval in the ISR,
    //so that the interval will be always correct even when capture_queue is not handled in time and overflow.
    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample_left = edata->cap_value;
        cap_val_end_of_sample_left = cap_val_begin_of_sample_left;
    } else {
        cap_val_end_of_sample_left = edata->cap_value;
        uint32_t tof_ticks_left = cap_val_end_of_sample_left - cap_val_begin_of_sample_left;

        // notify the task to calculate the distance
        xTaskNotifyFromISR(task_to_notify_left, tof_ticks_left, eSetValueWithOverwrite, &high_task_wakeup_left);
    }

    return high_task_wakeup_left == pdTRUE;
}

static bool hc_sr04_echo_callback_right(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t cap_val_begin_of_sample_right = 0;
    static uint32_t cap_val_end_of_sample_right = 0;
    TaskHandle_t task_to_notify_right = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup_right = pdFALSE;

    //calculate the interval in the ISR,
    //so that the interval will be always correct even when capture_queue is not handled in time and overflow.
    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample_right = edata->cap_value;
        cap_val_end_of_sample_right = cap_val_begin_of_sample_right;
    } else {
        cap_val_end_of_sample_right = edata->cap_value;
        uint32_t tof_ticks = cap_val_end_of_sample_right - cap_val_begin_of_sample_right;

        // notify the task to calculate the distance
        xTaskNotifyFromISR(task_to_notify_right, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup_right);
    }

    return high_task_wakeup_right == pdTRUE;
}

/**
 * @brief generate single pulse on Trig pin to start a new sample
 */
static void gen_trig_output(void)
{
    gpio_set_level(HC_SR04_TRIG_GPIORIGHT, 1); // set high
    esp_rom_delay_us(10);
    gpio_set_level(HC_SR04_TRIG_GPIORIGHT, 0); // set low
}

static void gen_trig_outputLeft(void)
{
    gpio_set_level(HC_SR04_TRIG_GPIOLEFT, 1); // set high
    esp_rom_delay_us(10);
    gpio_set_level(HC_SR04_TRIG_GPIOLEFT, 0); // set low
}

/////////////////////////////////// Ultra task //////////////////////////////////////

static void ultra_sensor_right() {
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
        .gpio_num = HC_SR04_ECHO_GPIORIGHT,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(TAG, "Register capture callback");
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback_right,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Configure Trig pin");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIORIGHT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIORIGHT, 0));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    uint32_t tof_ticks;
    while (1) {
        // trigger the sensor to start a new sample
        gen_trig_output();
        // wait for echo done signal
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us > 35000) {
                dist_read_right = 9999;
                continue;
            }
            // convert the pulse width into measure distance
            float distance = (float) pulse_width_us / 58;
            //ESP_LOGI(TAG, "Measured distance RIGHT: %.2fcm", distance);
            dist_read_right = distance;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
static void ultra_sensor_left() {
    ESP_LOGI(TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 1,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(TAG, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = HC_SR04_ECHO_GPIOLEFT,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(TAG, "Register capture callback");
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback_left,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Configure Trig pin");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIOLEFT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIOLEFT, 0));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    uint32_t tof_ticks;
    while (1) {
        // trigger the sensor to start a new sample
        gen_trig_outputLeft();
        // wait for echo done signal
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us > 35000) {
                dist_read_left = 9999;
                continue;
            }
            // convert the pulse width into measure distance
            float distance = (float) pulse_width_us / 58;
            //ESP_LOGI(TAG, "Measured distance LEFT: %.2fcm", distance);
            dist_read_left = distance;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//////////////////////////////////////// STEERING task ////////////////////////////////////////
static void steer_task() {
    float previous_error = 0;
    float integral = 0;
    float derivative = 0;
    float dt = 100;
    float Kp = 1;
    float Ki = 0;
    float Kd = 1;
    float output;
    float error;

    float target = 20 ;
    float dist_read;

    while(1)
    {
        if(dist_read_left < dist_read_right) {
            dist_read = dist_read_left;

            error = target - dist_read;
            integral = integral + error * dt;
            derivative = (error - previous_error) / dt;
            output = Kp * error + Ki * integral + Kd * derivative;
            previous_error = error;

            //ESP_LOGI(TAG, "LEFT IS CLOSER TO WALL");

            if(error < -0.65) {
                pid_turn = TURN_LEFT;
            } else if(error > 0.25) {
                pid_turn = TURN_RIGHT;
            } else {
                pid_turn = STRAIGHT;
            }
        } else {
            dist_read = dist_read_right;

            error = target - dist_read;
            integral = integral + error * dt;
            derivative = (error - previous_error) / dt;
            output = Kp * error + Ki * integral + Kd * derivative;
            previous_error = error;

            //ESP_LOGI(TAG, "RIGHT IS CLOSER TO WALL");
            if(error < -0.65) {
                pid_turn = TURN_RIGHT;
            } else if(error > 0.25) {
                pid_turn = TURN_LEFT;
            } else {
                pid_turn = STRAIGHT;
            }
        }
                
        vTaskDelay(pdMS_TO_TICKS(dt));
    }
}


//////////////////////////////////////// CRUISE CONTROL task ////////////////////////////////////////


/////////////////////////////////////// Alphanumeric Functions //////////////////////////////////////////////////////
// Turn on oscillator for alpha display
int alpha_oscillator() {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    return ret;
}

// Set blink rate to off
int no_blink() {
    int ret;
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd2);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
    int ret;
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
    i2c_master_stop(cmd3);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd3);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    return ret;
}

static void show_display() {
    int i;
        // Debug
    int ret;
    int sec_elapse = 0;
    int min_elapse = 0;
    printf(">> Test Alphanumeric Display: \n");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}

    // Write to characters to buffer
    uint16_t displaybuffer[8];
    while(1) {
        char sec[50];
        char min[50];
        sec_elapse++;
        if(sec_elapse == 60) {
            sec_elapse = 0;
            min_elapse++;
        }

        if(sec_elapse < 10) {
            sprintf(sec, "0%d", sec_elapse);
        } else {
            sprintf(sec, "%d", sec_elapse);
        }

        if(min_elapse < 10) {
            sprintf(min, "0%d", min_elapse);
        } else {
            sprintf(min, "%d", min_elapse);
        }

        strcat(min, sec);
        strcpy(alpha_str, min);
        for(i = 0; i < 8; i++) {
            displaybuffer[i] = 0b0000000000000000;
        }
        for(i = 0; i < strlen(alpha_str); i++) {
            displaybuffer[i] = alphafonttable[(int) alpha_str[i]];
        }
        // Send commands characters to display over I2C
        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
        for (uint8_t i=0; i<8; i++) {
            i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd4);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void udp_server_task(void *pvParameters) {
    char rx_buffer[128];
    struct sockaddr_in server_addr, client_addr;
    int addr_family, ip_protocol, server_sock;
    
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    server_sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));


    socklen_t socklen = sizeof(client_addr); // Define socklen here

    while (1) {
        int len = recvfrom(server_sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&client_addr, &socklen);
        
        // Error occurred during receiving
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        }
        // Data received
        else {
            rx_buffer[len] = 0;  // Null-terminate whatever we received and treat like a string
            ESP_LOGI(TAG, "Received %d bytes from %s:", len, inet_ntoa(client_addr.sin_addr));
            ESP_LOGI(TAG, "%s", rx_buffer);
            if(strcmp(rx_buffer, "stop") == 0) {
                e_brake = 1;
            } else if(strcmp(rx_buffer, "start") == 0) {
                e_brake = 0;
            } else if(strcmp(rx_buffer, "turn") == 0) {
                rev_direction = 1;
            }
            // if (strcmp(rx_buffer, "TURN_ON") == 0) {
            //     gpio_set_level(LED_GPIO_PIN, 1); // Turn on the LED
            //     ESP_LOGI(TAG, "%s", "on");
            //     const char* ack_msg = "LED_ON_ACK";
            //     sendto(server_sock, ack_msg, strlen(ack_msg), 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
            // } else {
            //     gpio_set_level(LED_GPIO_PIN, 0); // Turn on the LED
            //     ESP_LOGI("LED", "%s", "off");
            //     const char* ack_msg = "LED_OFF_ACK";
            //     sendto(server_sock, ack_msg, strlen(ack_msg), 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
            // }
        }
    }
    if (server_sock != -1) {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(server_sock, 0);
        close(server_sock);
    }
    vTaskDelete(NULL);
}


void app_main(void) {
    i2c_master_init();
    i2c_scanner();
    configure(2, LIDARLITE_ADDR_DEFAULT);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    //ESP_ERROR_CHECK(example_connect());

    wifi_init();

    // Create the servo task
    xTaskCreate(lidar_task, "LidarTask", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(ultra_sensor_right, "ultra_sensor_right", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(ultra_sensor_left, "ultra_sensor_left", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

    xTaskCreate(steer_task, "SteerTask", 4096, NULL, configMAX_PRIORITIES-3, NULL);
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, configMAX_PRIORITIES-3, NULL);

    xTaskCreate(servo_task, "ServoTask", 4096, NULL, configMAX_PRIORITIES-4, NULL);
    xTaskCreate(speed_task, "SpeedTask", 4096, NULL, configMAX_PRIORITIES-4, NULL);

    xTaskCreate(encoder_task, "EncoderTask", 4096, NULL, configMAX_PRIORITIES-6, NULL);

    xTaskCreate(timer, "timer", 4096, NULL, configMAX_PRIORITIES-7, NULL);
    xTaskCreate(show_display,"show_display", 4096, NULL, configMAX_PRIORITIES-8, NULL);
    // xTaskCreate(udp_client_task, "UDPTask", 4096, NULL, configMAX_PRIORITIES-9, NULL);
}
