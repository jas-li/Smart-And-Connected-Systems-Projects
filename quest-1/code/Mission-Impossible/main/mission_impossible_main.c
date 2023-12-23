// Authors: Jason Li, Jake Lee, Maxim Slobodchikov, Jialin Sui
// Date: 09/22/2023
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_vfs_dev.h"	// This is associated with VFS -- virtual file system interface and abstraction -- see the docs
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "sdkconfig.h"

#define BLINK_GPIO CONFIG_BLINK_GPIO

#define ALL_CLEAR 12 // green LED
#define TEMPERATURE_CHANGE 27 // red LED
#define LIGHT_CHANGE 33 // yellow LED
#define FLOOR_TOUCH 15 // blue LED

#define BUTTON 32
#define PHOTOCELL 39

#define GPIO_INPUT_IO_1       32
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;
static const adc_channel_t channel2 = ADC_CHANNEL_3;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static const char *TAG = "RTOS Task";

static uint8_t red_temp_state = 0;
static uint8_t yellow_light_state = 0;
static uint8_t blue_floor_state = 0;


int flag = 0; // Flag to check if button is pressed

static void IRAM_ATTR gpio_isr_handler(void* arg)  // Interrupt handler for your GPIO
{
    flag = 1;
}

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

static void init(void) // Iniialize pins and directions
{
    ESP_LOGI(TAG, "Configured GPIO LED!");
    gpio_reset_pin(ALL_CLEAR);
    gpio_reset_pin(TEMPERATURE_CHANGE);
    gpio_reset_pin(LIGHT_CHANGE);
    gpio_reset_pin(FLOOR_TOUCH);

    gpio_reset_pin(BUTTON);

    // Assigning the input/output pins
    gpio_set_direction(ALL_CLEAR, GPIO_MODE_OUTPUT);
    gpio_set_direction(TEMPERATURE_CHANGE, GPIO_MODE_OUTPUT);
    gpio_set_direction(LIGHT_CHANGE, GPIO_MODE_OUTPUT);
    gpio_set_direction(FLOOR_TOUCH, GPIO_MODE_OUTPUT);

    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);

}

// Button interrupt init
// Configuration code from: https://github.com/BU-EC444/04-Code-Examples/blob/main/button-timer/main/main_1.c
static void button_init()
{
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_intr_enable(GPIO_INPUT_IO_1 );
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
}

static void temperature_change_task(void *arg)
{
    float time = 0;
     //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

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
        double tempC = temp - 273.15;

        time += 0.5;
        printf("Time: %.2f\n", time);
        printf("Temperature Reading: %.2fC\n", tempC);
        if(tempC > 21) {
            red_temp_state= 1;
        } else {
            red_temp_state = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void light_change_task(void *arg)
{

   //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel2, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel2, atten);
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
                adc_reading += adc1_get_raw((adc1_channel_t)channel2);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel2, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        printf("Light Voltage Reading: %ldmV\n\n", voltage);
        if(voltage < 1480) {
            yellow_light_state = 1;
        } else {
            yellow_light_state = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void floor_touch_task(void *arg)
{
    while(1)
    {                               // loop forever in this task
        if(flag)
        {
            //printf("Button pressed.\n");
            blue_floor_state = 1; // toggle the state of the blue LED
            vTaskDelay(1000 / portTICK_PERIOD_MS);  // wait a bit
            blue_floor_state = 0; // toggle the state of the blue LED
            flag = 0;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);  // wait a bit
    }
}

static void led_task(void *arg)
{
    while(1)
    {
        if(red_temp_state == 1)
        {
            gpio_set_level(TEMPERATURE_CHANGE, 1);
        }
        else
        {
            gpio_set_level(TEMPERATURE_CHANGE, 0);
        }

        if(yellow_light_state == 1)
        {
            gpio_set_level(LIGHT_CHANGE, 1);
        }
        else
        {
            gpio_set_level(LIGHT_CHANGE, 0);
        }

        if(blue_floor_state == 1)
        {
            gpio_set_level(FLOOR_TOUCH, 1);
        }
        else
        {
            gpio_set_level(FLOOR_TOUCH, 0);
        }

        if(blue_floor_state == 0 && yellow_light_state == 0 && red_temp_state == 0)
        {
            gpio_set_level(ALL_CLEAR, 1);
        }
        else
        {
            gpio_set_level(ALL_CLEAR, 0);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}




void app_main()
{
    init();
    button_init();

    //Initializing all the tasks
    xTaskCreate(led_task, "led_task",1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(floor_touch_task, "floor_touch_task",1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(temperature_change_task, "temperature_change_task",1024*2, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(light_change_task, "light_change_task",1024*2, NULL, configMAX_PRIORITIES-3, NULL);

    while(1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);  // wait a bit

    }
}
