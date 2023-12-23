/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"

// vote button A5
// select button A4

#define GPIO_INPUT_IO_1       4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1

#define GPIO_INPUT_IO_2       5
#define GPIO_INPUT_PIN_SEL_2  (1ULL<<GPIO_INPUT_IO_2)

int incomingID = -1;

char state[5] = "CT";
char stateString[20] = "Connecticut";  

enum State {
    CT = 0,
    MA = 1,
    ME = 2,
    NH = 3,
    NY = 4,
    RI = 5,
    VT = 6,
    NUM_STATES
};


enum State currentState = CT;

// Global Variables
int restricted[100];
int rest_cnt = 0;
int i;

int vote_bool = 0;

int msg_id;
esp_mqtt_client_handle_t client;


static const char *TAG = "mqtt5_example";

/////////////////////////RECEIVER CODE/////////////////////////
#include <stdlib.h>             // Added in 2023..
#include <inttypes.h>           // Added in 2023
#include "freertos/queue.h"     // Added in 2023
#include "driver/rmt_tx.h"      // Modified in 2023
#include "soc/rmt_reg.h"        // Not needed?
#include "driver/gptimer.h"     // Added in 2023
#include "driver/mcpwm_prelude.h"// Added in 2023
#include "driver/ledc.h"        // Added in 2023
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_wifi.h"

// MCPWM defintions -- 2023: modified
#define MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define MCPWM_FREQ_HZ             38000    // 38KHz PWM -- 1/38kHz = 26.3us
#define MCPWM_FREQ_PERIOD         263      // 263 ticks = 263 * 0.1us = 26.3us
#define MCPWM_GPIO_NUM            25

// LEDC definitions -- 2023: modified
// NOT USED / altnernative to MCPWM above
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          25
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           6     // Set duty resolution to 6 bits
#define LEDC_DUTY               32    // Set duty to 50%. ((2^6) - 1) * 50% = 32
#define LEDC_FREQUENCY          38000 // Frequency in Hertz. 38kHz

// UART definitions -- 2023: no changes
#define UART_TX_GPIO_NUM 25 // A1
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)
#define BUF_SIZE2 (32)

// LED Output pins definitions -- 2023: minor changes
#define BLUEPIN   14
#define GREENPIN  32
#define REDPIN    15
#define ONBOARD   13
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<BLUEPIN) | (1ULL<<GREENPIN) | (1ULL<<REDPIN) | (1ULL<<ONBOARD) )

// Default ID/color
#define ID 3
#define COLOR 'R'

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;              // START BYTE that UART looks for
char myID = (char) ID;
char myColor = (char) COLOR;
int len_out = 3;

bool signalReceived = false;
int timer_count = 0;

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL; // 2023: no changes
// static xQueueHandle_t timer_queue; -- 2023: removed

// A simple structure to pass "events" to main task -- 2023: modified
typedef struct {
    uint64_t event_count;
} example_queue_element_t;

// Create a FIFO queue for timer-based events -- Modified
example_queue_element_t ele;
QueueHandle_t timer_queue;

// System tags for diagnostics -- 2023: modified
//static const char *TAG_SYSTEM = "ec444: system";       // For debug logs
static const char *TAG_UART = "ec444: uart";           // For UART logs

// Utilities ///////////////////////////////////////////////////////////////////

// Checksum -- 2023: no changes
char genCheckSum(char *p, int len) {
    char temp = 0;
    for (int i = 0; i < len; i++){
        temp = temp^p[i];
    }
    // printf("%X\n",temp);  // Diagnostic

    return temp;
}
bool checkCheckSum(uint8_t *p, int len) {
    char temp = (char) 0;
    bool isValid;
    for (int i = 0; i < len-1; i++){
        temp = temp^p[i];
    }
    // printf("Check: %02X ", temp); // Diagnostic
    if (temp == p[len-1]) {
        isValid = true; }
    else {
        isValid = false; }
    return isValid;
}

// MCPWM Initialize -- 2023: this is to create 38kHz carrier
static void pwm_init() {

    // Create timer
    mcpwm_timer_handle_t pwm_timer = NULL;
    mcpwm_timer_config_t pwm_timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .period_ticks = MCPWM_FREQ_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&pwm_timer_config, &pwm_timer));

    // Create operator
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    // Connect timer and operator
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, pwm_timer));

    // Create comparator from the operator
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    // Create generator from the operator
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = MCPWM_GPIO_NUM,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the duty cycle is 50%
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator,132));
    // CANNOT FIGURE OUT HOW MANY TICKS TO COMPARE TO TO GET 50%

    // Set generator action on timer and compare event
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    // Enable and start timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(pwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP));

}

// Configure UART -- 2023: minor changes
static void uart_init() {
    // Basic configs
    const uart_config_t uart_config = {
        .baud_rate = 1200, // Slow BAUD rate
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_param_config(UART_NUM_1, &uart_config);

    // Set UART pins using UART0 default pins
    uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Reverse receive logic line
    uart_set_line_inverse(UART_NUM_1,UART_SIGNAL_RXD_INV);

    // Install UART driver
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// GPIO init for LEDs -- 2023: modified
static void led_init() {
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

/////////////////////////ALPHA NUMERIC DISPLAY/////////////////////////
volatile bool updateDisplay = false;

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

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


uint16_t displaybuffer[8];


// Bit map for alphanumeric display (14-seg)
// Taken from: https://github.com/adafruit/Adafruit_LED_Backpack
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
// Function to initiate i2c -- note the MSB declaration!
static void i2c_example_master_init(){
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
    conf.clk_flags = 0;                                     // <-- UNCOMMENT IF YOU GET ERRORS (see readme.md)
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                        I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                        I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {printf("- initialized: yes\n\n");}

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

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
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}

////////////////////////////////////////////////////////////////////////////////

// Alphanumeric Functions //////////////////////////////////////////////////////

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


///////////////////////////////Button Interrupt////////////////////////////////////

static QueueHandle_t gpio_evt_queue1 = NULL;
static QueueHandle_t gpio_evt_queue2 = NULL;

static void IRAM_ATTR gpio_isr_handler1(void* arg){
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue1, &gpio_num, NULL);
}
static void IRAM_ATTR gpio_isr_handler2(void* arg){
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue2, &gpio_num, NULL);
}


// Button interrupt init -- 2023: minor changes
static void button1_init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Interrupt on rising edge
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; // Bit mask for Button 1
    io_conf.mode = GPIO_MODE_INPUT; // Set as input mode
    io_conf.pull_up_en = 1; // Enable pull-up mode
    gpio_config(&io_conf);

    gpio_intr_enable(GPIO_INPUT_IO_1); // Enable interrupt for Button 1
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler1, (void*) GPIO_INPUT_IO_1); // Add ISR handler for Button 1

    gpio_evt_queue1 = xQueueCreate(10, sizeof(uint32_t)); // Create a queue for Button 1 events
}

static void button2_init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Interrupt on rising edge
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL_2; // Bit mask for Button 2
    io_conf.mode = GPIO_MODE_INPUT; // Set as input mode
    io_conf.pull_up_en = 1; // Enable pull-up mode
    gpio_config(&io_conf);

    gpio_intr_enable(GPIO_INPUT_IO_2); // Enable interrupt for Button 2
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler2, (void*) GPIO_INPUT_IO_2); // Add ISR handler for Button 2

    gpio_evt_queue2 = xQueueCreate(10, sizeof(uint32_t)); // Create a queue for Button 2 events
}


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}
esp_mqtt_client_handle_t mqtt_client = NULL;

static esp_mqtt5_user_property_item_t user_property_arr[] = {
        {"board", "esp32"},
        {"u", "user"},
        {"p", "password"}
    };

#define USE_PROPERTY_ARR_SIZE   sizeof(user_property_arr)/sizeof(esp_mqtt5_user_property_item_t)

static esp_mqtt5_publish_property_config_t publish_property = {
    .payload_format_indicator = 1,
    .message_expiry_interval = 1000,
    .topic_alias = 0,
    .response_topic = "/topic/test/response",
    .correlation_data = "123456",
    .correlation_data_len = 6,
};

static esp_mqtt5_subscribe_property_config_t subscribe_property = {
    .subscribe_id = 25555,
    .no_local_flag = false,
    .retain_as_published_flag = false,
    .retain_handle = 0,
    .is_share_subscribe = true,
    .share_name = "group1",
};

static esp_mqtt5_subscribe_property_config_t subscribe1_property = {
    .subscribe_id = 25555,
    .no_local_flag = true,
    .retain_as_published_flag = false,
    .retain_handle = 0,
};

static esp_mqtt5_unsubscribe_property_config_t unsubscribe_property = {
    .is_share_subscribe = true,
    .share_name = "group1",
};

static esp_mqtt5_disconnect_property_config_t disconnect_property = {
    .session_expiry_interval = 60,
    .disconnect_reason = 0,
};

static void print_user_property(mqtt5_user_property_handle_t user_property)
{
    if (user_property) {
        uint8_t count = esp_mqtt5_client_get_user_property_count(user_property);
        if (count) {
            esp_mqtt5_user_property_item_t *item = malloc(count * sizeof(esp_mqtt5_user_property_item_t));
            if (esp_mqtt5_client_get_user_property(user_property, item, &count) == ESP_OK) {
                for (int i = 0; i < count; i ++) {
                    esp_mqtt5_user_property_item_t *t = &item[i];
                    ESP_LOGI(TAG, "key is %s, value is %s", t->key, t->value);
                    free((char *)t->key);
                    free((char *)t->value);
                }
            }
            free(item);
        }
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt5_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    int msg_id;

    ESP_LOGD(TAG, "free heap size is %" PRIu32 ", minimum %" PRIu32, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        print_user_property(event->property->user_property);
        esp_mqtt5_client_set_user_property(&publish_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_publish_property(client, &publish_property);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "NY", 0, 1, 1);
        esp_mqtt5_client_delete_user_property(publish_property.user_property);
        publish_property.user_property = NULL;
        ESP_LOGI(TAG, "sent vote, msg_id=%d", msg_id);

        esp_mqtt5_client_set_user_property(&subscribe_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        esp_mqtt5_client_set_subscribe_property(client, &subscribe_property);
        msg_id = esp_mqtt_client_subscribe(client, "/topic/restrict", 0);
        esp_mqtt5_client_delete_user_property(subscribe_property.user_property);
        subscribe_property.user_property = NULL;
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // esp_mqtt5_client_set_user_property(&subscribe1_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        // esp_mqtt5_client_set_subscribe_property(client, &subscribe1_property);
        // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 2);
        // esp_mqtt5_client_delete_user_property(subscribe1_property.user_property);
        // subscribe1_property.user_property = NULL;
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // esp_mqtt5_client_set_user_property(&unsubscribe_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
        // esp_mqtt5_client_set_unsubscribe_property(client, &unsubscribe_property);
        // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos0");
        // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        // esp_mqtt5_client_delete_user_property(unsubscribe_property.user_property);
        // unsubscribe_property.user_property = NULL;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        esp_mqtt5_client_set_publish_property(client, &publish_property);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    // case MQTT_EVENT_UNSUBSCRIBED:
    //     ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    //     print_user_property(event->property->user_property);
    //     esp_mqtt5_client_set_user_property(&disconnect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    //     esp_mqtt5_client_set_disconnect_property(client, &disconnect_property);
    //     esp_mqtt5_client_delete_user_property(disconnect_property.user_property);
    //     disconnect_property.user_property = NULL;
    //     esp_mqtt_client_disconnect(client);
    //     break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        print_user_property(event->property->user_property);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        print_user_property(event->property->user_property);
        ESP_LOGI(TAG, "payload_format_indicator is %d", event->property->payload_format_indicator);
        ESP_LOGI(TAG, "response_topic is %.*s", event->property->response_topic_len, event->property->response_topic);
        ESP_LOGI(TAG, "correlation_data is %.*s", event->property->correlation_data_len, event->property->correlation_data);
        ESP_LOGI(TAG, "content_type is %.*s", event->property->content_type_len, event->property->content_type);
        ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
        char *tempDataBuffer = (char *)malloc(event->data_len + 1);
        strncpy(tempDataBuffer, event->data, event->data_len);
        tempDataBuffer[event->data_len] = '\0';  // Null-terminate the string
        restricted[rest_cnt] = atoi(tempDataBuffer);
        rest_cnt++;
        free(tempDataBuffer);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        print_user_property(event->property->user_property);
        ESP_LOGI(TAG, "MQTT5 return code is %d", event->error_handle->connect_return_code);
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt5_app_start(void)
{
    esp_mqtt5_connection_property_config_t connect_property = {
        .session_expiry_interval = 10,
        .maximum_packet_size = 1024,
        .receive_maximum = 65535,
        .topic_alias_maximum = 2,
        .request_resp_info = true,
        .request_problem_info = true,
        .will_delay_interval = 10,
        .payload_format_indicator = true,
        .message_expiry_interval = 10,
        .response_topic = "/test/response",
        .correlation_data = "123456",
        .correlation_data_len = 6,
    };

    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .network.disable_auto_reconnect = true,
        .credentials.username = "123",
        .credentials.authentication.password = "456",
        .session.last_will.topic = "/topic/will",
        .session.last_will.msg = "i will leave",
        .session.last_will.msg_len = 12,
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt5_cfg);

#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt5_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt5_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt5_cfg);

    /* Set connection properties and user properties */
    esp_mqtt5_client_set_user_property(&connect_property.user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_user_property(&connect_property.will_user_property, user_property_arr, USE_PROPERTY_ARR_SIZE);
    esp_mqtt5_client_set_connect_property(client, &connect_property);

    /* If you call esp_mqtt5_client_set_user_property to set user properties, DO NOT forget to delete them.
     * esp_mqtt5_client_set_connect_property will malloc buffer to store the user_property and you can delete it after
     */
    esp_mqtt5_client_delete_user_property(connect_property.user_property);
    esp_mqtt5_client_delete_user_property(connect_property.will_user_property);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt5_event_handler, NULL);
    esp_mqtt_client_start(client);
}

////////////////////////////////////////TASKS////////////////////////////////////////


void publish_button_task(void *pvParameter) {
    uint32_t io_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue1, &io_num, portMAX_DELAY)) {
            if(vote_bool == 1) {
                // Logic for Button 1
                ESP_LOGI(TAG, "Button 1 pressed");
                esp_mqtt5_publish_property_config_t publish_property = {
                    .payload_format_indicator = 1,                     // Set to 1 if payload format is UTF-8, 0 if binary
                    .message_expiry_interval = 1000,                   // Message expiration interval in milliseconds
                    .topic_alias = 0,                                  // Topic alias, set to 0 if not used
                    .response_topic = "/topic/test/response",          // Response topic for the published message
                    .correlation_data = "123456",                      // Correlation data associated with the message
                    .correlation_data_len = 6,                         // Length of the correlation data
                    .user_property = NULL,                             // User properties, set to NULL for none
                };
                esp_mqtt5_client_set_publish_property(client, &publish_property);
                char payload[50];
                sprintf(payload, "%s,%d", stateString, incomingID);
                int msg_id = esp_mqtt_client_publish(client, "/topic/votes", payload, 0, 1, 1);
                esp_mqtt5_client_delete_user_property(publish_property.user_property);
                publish_property.user_property = NULL;
                ESP_LOGI(TAG, "sent data for Button 1, msg_id=%d", msg_id);
                vote_bool = 0;
                vTaskDelay(200 / portTICK_PERIOD_MS);

            } else {
                printf("Can't vote sorry");
            }
        }
    }
}


void state_button_task(void *pvParameter) {
    uint32_t io_num;
    while (1) {
        // Check if Button 1 is pressed
            
        if (xQueueReceive(gpio_evt_queue2, &io_num, portMAX_DELAY)) 
        {
            if(vote_bool == 1) {
                ESP_LOGI(TAG, "Button 2 pressed");
                currentState = (currentState + 1) % NUM_STATES;
                // Replace state with next northeastern state
                switch (currentState) {
                    case CT:
                        strcpy(state, "CT");
                        strcpy(stateString, "Connecticut");
                        break;
                    case MA:
                        strcpy(state, "MA");
                        strcpy(stateString, "Massachusetts");
                        break;
                    case ME:
                        strcpy(state, "ME");
                        strcpy(stateString, "Maine");
                        break;
                    case NH:
                        strcpy(state, "NH");
                        strcpy(stateString, "New Hampshire");
                        break;
                    case NY:
                        strcpy(state, "NY");
                        strcpy(stateString, "New York");
                        break;
                    case RI:
                        strcpy(state, "RI");
                        strcpy(stateString, "Rhode Island");
                        break;
                    case VT:
                        strcpy(state, "VT");
                        strcpy(stateString, "Vermont");
                        break;
                    default:
                        strcpy(state, "CT"); // Default case to handle unexpected values
                        strcpy(stateString, "Connecticut");
                        break;
                }
            }
            updateDisplay = true;
        }

            vTaskDelay(200 / portTICK_PERIOD_MS);
    }

        // Adjust the delay according to your needs
        vTaskDelay(500 / portTICK_PERIOD_MS);
}

static void display_task() {
    int ret;
    printf(">> Test Alphanumeric Display: \n");

    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if (ret == ESP_OK) {
        printf("- oscillator: ok \n");
    }

    // Set display blink off
    ret = no_blink();
    if (ret == ESP_OK) {
        printf("- blink: off \n");
    }

    // Set brightness to maximum
    ret = set_brightness_max(0xF);
    if (ret == ESP_OK) {
        printf("- brightness: max \n");
    }

    // Initialize the display buffer
    uint16_t displaybuffer[8];
    displaybuffer[0] = alphafonttable[(int)state[0]];
    displaybuffer[1] = alphafonttable[(int)state[1]];

    // Continually updates the display
    while (1) {
        if (updateDisplay) {
            // Update the display buffer
            displaybuffer[0] = alphafonttable[(int)state[0]];
            displaybuffer[1] = alphafonttable[(int)state[1]];

            // Send commands and characters to display over I2C
            i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
            i2c_master_start(cmd4);
            i2c_master_write_byte(cmd4, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);

            for (uint8_t i = 0; i < 8; i++) {
                i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
                i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
            }

            i2c_master_stop(cmd4);
            ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd4);

            // Reset update flag
            updateDisplay = false;
        }

        // Yield to the scheduler regularly
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 ms
    }
}

//////// RECIEVER TASKS ///////////
void recv_task(){
    // Buffer for input data
    // Receiver expects message to be sent multiple times
    uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE2);
    while (1) {
        int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE2, 100 / portTICK_PERIOD_MS);
        //ESP_LOGE(TAG_UART, "Length: %d", len_in);
        if (len_in > 10) {
        int nn = 0;
        //ESP_LOGE(TAG_UART, "Length greater than 10");
        while (data_in[nn] != start) {
            nn++;
        }
        uint8_t copied[len_out];
        memcpy(copied, data_in + nn, len_out * sizeof(uint8_t));
        //printf("before checksum");
        //ESP_LOG_BUFFER_HEXDUMP(TAG_UART, copied, len_out, ESP_LOG_INFO);
        if (checkCheckSum(copied,len_out)) {
            printf("after checksum");
            ESP_LOG_BUFFER_HEXDUMP(TAG_UART, copied, len_out, ESP_LOG_INFO);
            printf("ID: %d\n", (int)copied[1]);
            incomingID = (int)copied[1];
            uart_flush(UART_NUM_1);
            signalReceived = true;
            myColor = 'G';
            vote_bool = 1;
            for(i=0; i<rest_cnt; i++) {
                if((int)copied[1] == restricted[i]) {
                    vote_bool = 0;
                    myColor = 'R';
                    break;
                } else {
                    vote_bool = 1;
                    myColor = 'G';
                }
            }
        }
        }
        else{
        // printf("Nothing received.\n");
        }
        //vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    free(data_in);
}

// LED task to light LED based on traffic state -- 2023: no changes
void led_task(){
    while(1) {
        switch((int)myColor){
        case 'R' : // Red
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 1);
            gpio_set_level(BLUEPIN, 0);
            // printf("Current state: %c\n",status);
            break;
        case 'B' : // Yellow
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(REDPIN, 0);
            gpio_set_level(BLUEPIN, 1);
            // printf("Current state: %c\n",status);
            break;
        case 'G' : // Green
            gpio_set_level(GREENPIN, 1);
            gpio_set_level(REDPIN, 0);
            gpio_set_level(BLUEPIN, 0);
            // printf("Current state: %c\n",status);
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // initialize global variable (array of restricted IDs)
    for(i=0; i<100; i++) {
        restricted[i] = -1; // init
    }

    // Reciever inits
    mux = xSemaphoreCreateMutex();
    uart_init();
    led_init();
    pwm_init();

    // end of reciver inits

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // Initialize the GPIO
    button1_init();
    button2_init();
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    i2c_example_master_init();
    i2c_scanner();

    currentState = CT; // Reset current state to CT
    strcpy(state, "CT"); // Reset state string to "CT"

    // Update the display initially
    updateDisplay = true; // Flag to indicate the display needs updating

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(publish_button_task, "publish_button_task", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(state_button_task, "state_button_task", 4096, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, configMAX_PRIORITIES-3, NULL);
    xTaskCreate(recv_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES-4, NULL);
    xTaskCreate(led_task, "set_traffic_task", 1024*2, NULL, configMAX_PRIORITIES-4, NULL);
    mqtt5_app_start();

}