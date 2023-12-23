/*
 BU EC444
 JiaLin Sui

 Skill 09 - Photoresistor
 19 Sep. 2023

*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include <math.h>

#define RESISTOR_PIN 36 // GPIO pin connected to the photoresistor
#define TEMP_PIN 4
#define VREF 3.3
static void readLightLevel() {
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // ADC1_CHANNEL_6 corresponds to GPIO 34

    uint32_t rawValue = adc1_get_raw(ADC1_CHANNEL_0);

    // Convert the ADC reading to resistance
    double voltage = rawValue * (VREF / 4095.0); // Use the specified reference voltage

    printf("Raw ADC Value: %ld\n", rawValue);
    printf("Voltage (V): %.2f\n", voltage);
}

static void readTempLevel() {
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11); // ADC1_CHANNEL_6 corresponds to GPIO 34

    uint32_t rawValue = adc1_get_raw(ADC1_CHANNEL_3);

    // Convert the ADC reading to resistance
    double voltage = rawValue * (VREF / 4095.0); // Use the specified reference voltage

    printf("Raw ADC Value: %ld\n", rawValue);
    printf("Temp Voltage (V): %.2f\n", voltage);
}

void app_main() {
    while (1) {
        readLightLevel();
        readTempLevel();
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 second
    }
}
