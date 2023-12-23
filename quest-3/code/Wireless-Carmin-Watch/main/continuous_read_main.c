#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include "./ADXL343.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include<time.h>
#include <string.h>           
#include <inttypes.h>           
#include "freertos/queue.h"     
#include "esp_log.h"            // for error logging
#include "esp_system.h"         
#include "driver/uart.h"
#include "driver/gptimer.h"        
#include "driver/ledc.h"       
#include "esp_vfs_dev.h" 
#include "sdkconfig.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

char buff[128];
char payload[128];
uint16_t displaybuffer[8];

////////////////////////// Temperature Sensor //////////////////////////

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

// Global Variables
double tempC = 0;

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

////////////////////////// Accelerometer //////////////////////////

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

// ADXL343
#define SLAVE_ADDR1                         ADXL343_ADDRESS // 0x53

//Global Variables
float prevXAccel = 0.0;
float prevYAccel = 0.0;
float prevZAccel = 0.0;

int stepCount = 0;

// Function to initiate i2c -- note the MSB declaration!
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
    conf.clk_flags = 0;                                     // <-- UNCOMMENT IF YOU GET ERRORS (see readme.md)
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
    if (count == 0) {printf("- No I2C devices found!" "\n");}
}

////////////////////////////////////////////////////////////////////////////////

// Alphanumeric Functions //////////////////////////////////////////////////////
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

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

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID(uint8_t *data) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR1 << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) 
{
    // YOUR CODE HERE
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Create an I2C command

    // Start the I2C communication, write the register address, and data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR1 << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    // Execute the I2C command
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return err; // Return success code or an error code
}

// Read register
uint8_t readRegister(uint8_t reg) 
{
    // YOUR CODE HERE
    uint8_t data;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Create an I2C command

    // Start the I2C communication, write the register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR1 << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

    // Restart and read data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR1 << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_ACK); // Read the data with ACK
    i2c_master_stop(cmd);

    // Execute the I2C command
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) 
    {
        return data; // Return the read data
    }    
    else 
    {
        return 0; // Return an error value or handle the error as needed
    }
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) 
{
    // YOUR CODE HERE
    uint8_t data1, data2;
    data1 = readRegister(reg);
    if (reg == 41)
    {
        data2 = 0;
    }
    else
    {
        data2 = readRegister(reg+1);
    }
    return (((int16_t)data2 << 8) | data1);
}

void setRange(range_t range) {
    /* Red the data format register to preserve bits */
    uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

    /* Update the data rate */
    format &= ~0x0F;
    format |= range;

    /* Make sure that the FULL-RES bit is enabled for range scaling */
    format |= 0x08;

    /* Write the register back to the IC */
    writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
    /* Red the data format register to preserve bits */
    return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
    return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

////////////////////////////////////////////////////////////////////////////////

// function to get acceleration
void getAccel(float * xp, float *yp, float *zp) {
    *xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    //printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
}

// function to print roll and pitch
// roll and pitch calculation referenced from: https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
void calcRP(float x, float y, float z){
    // YOUR CODE HERE
    // double x_Buff = float(x);
    // double y_Buff = float(y);
    // double z_Buff = float(z);
    
    // roll: rotation about the X-axis
    double roll = atan2(y , z) * 57.3;
    // pitch: rotation about the Y-axis
    double pitch = atan2((- x) , sqrt(y * y + z * z)) * 57.3;
    
    printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);
}

// Task to continuously poll acceleration and calculate roll and pitch
static void test_adxl343() {
    printf("\n>> Polling ADAXL343\n");
    while (1) {
        float xVal, yVal, zVal;
        getAccel(&xVal, &yVal, &zVal);
        calcRP(xVal, yVal, zVal);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

////////////////////////// TIMER //////////////////////////
#define ONBOARD   13  // onboard LED
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<ONBOARD)

// Global flag
bool flag = false;

// Initialize GPIO for LED signals
static void led_init() {
    gpio_config_t io_conf = {};        // zero-initialize the config structure
    io_conf.mode = GPIO_MODE_OUTPUT;   // set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; // bit mask of the pins that you want to set
    io_conf.pull_down_en = 0;          // disable pull-down mode
    io_conf.pull_up_en = 0;            // disable pull-up mode
    gpio_config(&io_conf);             // configure GPIO with the given settings
}

int timerCount = 0; // Counter for timer

// A simple structure for queue elements
typedef struct {
    uint64_t event_count;
} example_queue_element_t;

// Create a FIFO queue for timer-based events
example_queue_element_t ele;
QueueHandle_t timer_queue;

// System log tags -- get logged when things happen, for debugging 
static const char *TAG_TIMER = "ec444: timer";       

// Timer interrupt handler -- callback timer function -- from GPTimer guide example
static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t timer_queue1 = (QueueHandle_t)user_data;    // represents state info passed to callback, if needed
    example_queue_element_t ele = {
          .event_count = edata->count_value                   // Retrieve count value and send to queue
    };
    xQueueSendFromISR(timer_queue1, &ele, &high_task_awoken); // Puts data into queue and alerts other recipients
    return (high_task_awoken == pdTRUE);  		      // pdTRUE indicates data posted successfully
}

// Timer configuration -- from GPTimer guide example
static void alarm_init() {
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer)); // instantiates timer

    gptimer_event_callbacks_t cbs = { // Set alarm callback
      .on_alarm = timer_on_alarm_cb,  // This is a specific supported callback from callbacks list
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, timer_queue)); // This registers the callback
    ESP_ERROR_CHECK(gptimer_enable(gptimer));                                      // Enables timer interrupt ISR

    ESP_LOGI(TAG_TIMER, "Start timer, update alarm value dynamically and auto reload"); 
    gptimer_alarm_config_t alarm_config = { // Configure the alarm 
      .reload_count = 0,                    // counter will reload with 0 on alarm event
      .alarm_count = 1*1000000,            // period = 1*1s = 1s
      .flags.auto_reload_on_alarm = true,   // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));  // this enacts the alarm config
    ESP_ERROR_CHECK(gptimer_start(gptimer));                            // this starts the timer
}


////////////////////////// BUTTON //////////////////////////

bool sample = false;
int button_state = 1; // 0 = stopped 1 = reset 2 = start

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1       12
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1 // casting GPIO input to bitmap

int btn_flag = 0;     // Global flag for signaling from ISR

// Define button interrupt handler -- just sets the flag on interrupt
static void IRAM_ATTR gpio_isr_handler(void* arg){
    btn_flag = 1;
}

// Intialize the GPIO to detect button press as interrupt
static void button_init() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;     // interrupt of rising edge
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; // bit mask of the pins, use GPIO4 here
    io_conf.mode = GPIO_MODE_INPUT;            // set as input mode
    io_conf.pull_up_en = 1;                    // enable resistor pull-up mode on pin
  gpio_config(&io_conf);                       // apply parameters
  gpio_intr_enable(GPIO_INPUT_IO_1 );          // enable interrupts on pin
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);   //install gpio isr service
  gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1); //hook isr handler for specific gpio pin
}
////////////////////////// BUZZER //////////////////////////

#define BUZZER_PIN 26

// Initialize the buzzer
static void buzzer_init(void) 
{
    gpio_reset_pin(BUZZER_PIN);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
}

////////////////////////// WIFI //////////////////////////
#define WIFI_SSID      "Group_7"
#define WIFI_PASSWORD  "smartsys"
#define UDP_SERVER_IP  "192.168.1.36"
#define UDP_PORT       3333
#define ESP32_HOSTNAME "ESP32"
static const char *TAG = "UDP_CLIENT";

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

////////////////////////// TASKS //////////////////////////
// Timer task - report step and temp data to console
static void timer_task(void *arg) 
{
    while (1) {
        // Transfer from queue and do something if triggered
        if (xQueueReceive(timer_queue, &ele, pdMS_TO_TICKS(2000))) 
        {
            //printf("timerCount: %d\n", timerCount);
            timerCount++;
            //print data (step, temp)
            if(timerCount == 10 && button_state == 2) {
                sprintf(payload, "%d,%f\n", stepCount, tempC);
                printf("%s\n", payload);
                // printf("%s", buff);
                stepCount = 0;
                timerCount = 0;
            }
        }
    }
}

void activity_task()
{
    while(1)
    {                               // loop forever in this task
        if(btn_flag)
        {
            button_state++;
            if(button_state > 2)
            {
                button_state = 0;
            }
            timerCount = 0;
            if(button_state == 1)
            {
                printf("CLEAR\n");
            }
            //printf("Button pressed.\n");
            vTaskDelay(400 / portTICK_PERIOD_MS);  // wait a bit
            btn_flag = 0;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);  // wait a bit
    }
}
// scrolling display task
// static void display_task()
// {
//     int ret;
//     int display_offset = 0;
//     int i;
//     int shift = 0;
//     while (1) 
//     {
//         for(i = 0; i < 14; i++) {
//             displaybuffer[i] = 0b0000000000000000; // change to be zeros
//         }
//         if(strlen(buff) <= 4) // Characters fit without scrolling
//         {
//             if(strlen(buff) < 4) // Fill the rest of the display buffer with spaces
//                 {
//                     for(int i = strlen(buff); i < 4; i++) // fill the rest of the display buffer with spaces
//                     {
//                         displaybuffer[i] = alphafonttable[0];
//                     }
//                 }
//                 //Send commands characters to display over I2C
//                 i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
//                 i2c_master_start(cmd4);
//                 i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
//                 i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
//                 for (uint8_t i=0; i<8; i++) 
//                 {
//                     i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
//                     i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
//                 }
//                 i2c_master_stop(cmd4);
//                 ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
//                 i2c_cmd_link_delete(cmd4);
//         }
//         else
//         {
//             shift++;
//             if(shift > (strlen(buff) - 4)) {
//                 shift = 0;
//             }
//             for(i = 0; i < strlen(buff); i++) {
//                 displaybuffer[i] = alphafonttable[(int) buff[i]];
//             }
//             // Send commands characters to display over I2C
//             i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
//             i2c_master_start(cmd4);
//             i2c_master_write_byte(cmd4, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
//             i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);   
//             for(i = 0; i < 14; i++) {
//                 i2c_master_write_byte(cmd4, displaybuffer[i + shift] & 0xFF, ACK_CHECK_EN);
//                 i2c_master_write_byte(cmd4, displaybuffer[i + shift] >> 14, ACK_CHECK_EN);
//             }
//             i2c_master_stop(cmd4);
//             ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
//             i2c_cmd_link_delete(cmd4);

//             // Delay to control scrolling speed
//             vTaskDelay(140 / portTICK_PERIOD_MS);
//         }
//     }
// }

// void display_task()
// {
//     char str[20];
//     int num;
//     int i;
//     // Debug
//     int ret;
//     printf(">> Test Alphanumeric Display: \n");

//     // Set up routines
//     // Turn on alpha oscillator
//     ret = alpha_oscillator();
//     if(ret == ESP_OK) {printf("- oscillator: ok \n");}
//     // Set display blink off
//     ret = no_blink();
//     if(ret == ESP_OK) {printf("- blink: off \n");}
//     ret = set_brightness_max(0xF);
//     if(ret == ESP_OK) {printf("- brightness: max \n");}

//     // Write to characters to buffer
//     uint16_t displaybuffer[8];
    
//     while(1) {
//         strcpy(str, buff);
//         for(i = 0; i < 8; i++) {
//             displaybuffer[i] = 0b0000110000111111; // change to be zeros
//         }

//         for(i = 0; i < strlen(str); i++) {
//             displaybuffer[i] = alphafonttable[(int) str[i]];
//         }
    
//         // Send commands characters to display over I2C
//         i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
//         i2c_master_start(cmd4);
//         i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
//         i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
//         for (uint8_t i=0; i<8; i++) {
//             i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
//             i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
//         }
//         i2c_master_stop(cmd4);
//         ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
//         i2c_cmd_link_delete(cmd4);
//     }
// }

void display_task()
{
    char str[20];
    int num;
    int i;
    // Debug
    int ret;
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
    uint16_t displaybuffer[20];
    int displayOffset = 0; // Initialize the display offset to 0

    while(1) {
        strcpy(str, buff);

        for(i = 0; i < 20; i++) {
            displaybuffer[i] = 0b0000000000000000; // change to be zeros
        }

        // Update the display buffer with characters from str, starting from displayOffset
        for(i = 0; i < 20; i++) {
            if (i + displayOffset < strlen(str)) {
                displaybuffer[i] = alphafonttable[(int) str[i + displayOffset]];
            }
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

        // Scroll the display
        displayOffset++;
        if (displayOffset > strlen(str)) {
            displayOffset = 0; // Reset the display offset if we've reached the end of the string
        }

        vTaskDelay(400 / portTICK_PERIOD_MS); // Adjust the delay to control scrolling speed
    }
}


void temperature_task()
{

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
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

        // Convert voltage to temperature
        // Formula code Referenced from: https://www.e-tinkers.com/2019/10/using-a-thermistor-with-arduino-and-unexpected-esp32-adc-non-linearity/
        double V = (double) voltage / 1000;
        double Rt = 10000 * V / (3 - V);
        double temp = 1 / (1 / (273.15 + 25) + log(Rt / 10000) / 3950.0);
        tempC = temp - 273.15;

        if (tempC > 25.0)
        {
            gpio_set_level(BUZZER_PIN, 1);
            // printf("BUZZZZZ\n");
        }
        else
        {
            gpio_set_level(BUZZER_PIN, 0);
        }

        // printf("Body Temperature: %fC\n", tempC);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void steps_task()
{
    while(1)
    {
        float xVal, yVal, zVal;
        getAccel(&xVal, &yVal, &zVal);

        float xDiff = fabs(xVal - prevXAccel);
        float yDiff = fabs(yVal - prevYAccel);
        float zDiff = fabs(zVal - prevZAccel);

        if(xDiff > 5.0 || yDiff > 5.0 || zDiff > 5.0)
        {
            stepCount++;
        }

        prevXAccel = xVal;
        prevYAccel = yVal;
        prevZAccel = zVal;

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
}


void get_time_task()
{   
    const uart_port_t uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_SCLK_APB,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));


    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE,  UART_PIN_NO_CHANGE));

    const int uart_buffer_size = (1024*2);
    // Configure a temporary buffer for the incoming data
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    uint8_t data[128];
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(uart_num, data, 128-1, 10);
        // Write data back to the UART
        // uart_write_bytes(uart_num, (const char*) data, len);
        if (len) {
            data[len] = '\0';
            strcpy(buff,(char*) data);
            // ESP_LOGI("UART TEST", "Recv str: %s", (char *) data);

        }
    }
}

static void udp_client_task(void *pvParameters) {
    char rx_buffer[128];
    char host_ip[] = UDP_SERVER_IP;
    int addr_family;
    int ip_protocol;

    while (1) {
        struct sockaddr_in dest_addr;
        struct sockaddr_in source_addr;  // For the source address in recvfrom
        socklen_t socklen = sizeof(source_addr);
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(UDP_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        while (1) {
            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            ESP_LOGI(TAG, "%s Message sent", payload);
            // Listen for incoming data after sending.
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0;  // Null-terminate whatever was received to make it a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, inet_ntoa(source_addr.sin_addr));
                ESP_LOGI(TAG, "%s", rx_buffer);
                strcpy(buff,(char*) rx_buffer);

                // int received_blink_duration = atoi(rx_buffer);
                // ESP_LOGI("BLINK", "%d", received_blink_duration);

                // if(received_blink_duration > 0) {
                //     blink_duration = received_blink_duration;
                // }
            }

            vTaskDelay(10000/portTICK_PERIOD_MS);  // Send every 2 seconds and then check for incoming data
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


void app_main(void)
{
    // Timer queue initialize 
    timer_queue = xQueueCreate(10, sizeof(example_queue_element_t));
    if (!timer_queue) {
    ESP_LOGE(TAG_TIMER, "Creating queue failed");
    return;
    }

    // ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
    //         256, 0, 0, NULL, 0) );

    button_init();     // Initialize button config
    buzzer_init();

    // /* Tell VFS to use UART driver */
    // esp_vfs_dev_uart_use_driver(UART_NUM_0);

    // //Check if Two Point or Vref are burned into eFuse
    // check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    // Routine
    i2c_master_init();
    i2c_scanner();
    alarm_init();

    // Check for ADXL343
    uint8_t deviceID;
    getDeviceID(&deviceID);
    if (deviceID == 0xE5) {
        printf("\n>> Found ADAXL343\n");
    }

    // Disable interrupts
    writeRegister(ADXL343_REG_INT_ENABLE, 0);
    // Set range
    setRange(ADXL343_RANGE_16_G);
    // Enable measurements
    writeRegister(ADXL343_REG_POWER_CTL, 0x08);

    gpio_set_level(BUZZER_PIN, 1);

    // Wifi and UDP
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init();
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(get_time_task, "get_time_task", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(activity_task, "activity_task", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(temperature_task, "temperature_task", 4096, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(steps_task, "steps_task", 4096, NULL, configMAX_PRIORITIES-3, NULL);
    xTaskCreate(timer_task, "timer_task", 4096, NULL, configMAX_PRIORITIES-4, NULL);
    xTaskCreate(display_task, "display_task", 4096, NULL, configMAX_PRIORITIES-1, NULL);
}
