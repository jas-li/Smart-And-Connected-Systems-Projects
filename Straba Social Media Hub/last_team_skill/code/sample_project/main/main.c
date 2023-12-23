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
#define UDP_SERVER_IP  "192.168.1.33"
#define UDP_PORT       8080
#define ESP32_HOSTNAME "ESP32"

static const char *TAG = "UDP_CLIENT";

static void wifi_init(void);
static void udp_server_task(void *pvParameters);

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
            
            if (strcmp(rx_buffer, "TURN_ON") == 0) {
                gpio_set_level(LED_GPIO_PIN, 1); // Turn on the LED
                ESP_LOGI(TAG, "%s", "on");
                const char* ack_msg = "LED_ON_ACK";
                sendto(server_sock, ack_msg, strlen(ack_msg), 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
            } else {
                gpio_set_level(LED_GPIO_PIN, 0); // Turn on the LED
                ESP_LOGI("LED", "%s", "off");
                const char* ack_msg = "LED_OFF_ACK";
                sendto(server_sock, ack_msg, strlen(ack_msg), 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
            }
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
    
    esp_rom_gpio_pad_select_gpio(LED_GPIO_PIN);
    gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init();
    xTaskCreate(udp_server_task, "udp_server_task", 4096, NULL, 5, NULL);
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


