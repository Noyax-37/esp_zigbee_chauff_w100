#include "esp_zigbee_core.h"
#include "esp_zigbee_ha_standard.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "cJSON.h"
#include "zigbee.h"
#include "wifi.h"
#include "thermostat.h"
#include "utils.h"

static const char *TAG = "ZB_THERMOSTAT";

void app_main(void)
{
    init_led_control();
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_led(0, 10, 0, 0, false);   // Rouge, clignotant
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_led(1, 10, 0, 0, false);   // Rouge, clignotant
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_led(2, 10, 0, 0, false);   // Rouge, clignotant
    vTaskDelay(pdMS_TO_TICKS(2000));
        
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    load_settings_from_nvs(); // Charger les paramètres au démarrage
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    wifi_init();
    xTaskCreate(watchdog_task, "watchdog_task", 2048, NULL, 1, NULL);
    xTaskCreate(counter_task, "counter_task", 2048, NULL, 1, NULL);
}
