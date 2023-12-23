#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#define LED_GPIO_PIN 13
#define WIFI_SSID      "Group_7"
#define WIFI_PASSWORD  "smartsys"
#define UDP_SERVER_IP  "192.168.1.18"
#define UDP_PORT       8080
#define ESP32_HOSTNAME "ESP32"

static const char *TAG = "UDP_CLIENT";

static void wifi_init(void);
static void udp_client_task(void *pvParameters);

volatile int blink_duration = 1000;

void LED_blink_task(void *pVParameter)
{   
      // Configure the GPIO as an output.
    esp_rom_gpio_pad_select_gpio(LED_GPIO_PIN);
    gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);

    while(1)
    {
        gpio_set_level(LED_GPIO_PIN, 0);  // Turn the LED on (assuming active low)
        vTaskDelay(blink_duration / portTICK_PERIOD_MS);  // Delay from global blink variable

        gpio_set_level(LED_GPIO_PIN, 1);  // Turn the LED off (assuming active low)
        vTaskDelay(blink_duration / portTICK_PERIOD_MS);  // Delay from global blink variable
    }
}

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


void app_main(void) {
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init();
    xTaskCreate(LED_blink_task, "LED_blink_task", 4096, (void *)blink_duration, 5, NULL);
    xTaskCreate(udp_client_task, "udp_client_task", 4096, NULL, 5, NULL);
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
            int err = sendto(sock, "GET_BLINK_TIME", strlen("GET_BLINK_TIME"), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

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

                int received_blink_duration = atoi(rx_buffer);
                ESP_LOGI("BLINK", "%d", received_blink_duration);

                if(received_blink_duration > 0) {
                    blink_duration = received_blink_duration;
                }
            }

            vTaskDelay(5000 / portTICK_PERIOD_MS);  // Send every 2 seconds and then check for incoming data
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

