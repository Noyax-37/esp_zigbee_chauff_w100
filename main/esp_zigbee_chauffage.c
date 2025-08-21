/* SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "esp_zigbee_chauffage.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include "zboss_api_core.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_server.h"
#include <lwip/ip_addr.h>
#include "esp_coexist.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include <stdlib.h>
#include "esp_task_wdt.h"
#include <inttypes.h>
#include <esp_timer.h>
#include "cJSON.h"
#include "zboss_api.h"
#include "zboss_api_af.h"
#include "zboss_api_zcl.h"
#include "zboss_api_buf.h"
#include "esp_mac.h"
#include "esp_zigbee_core.h"

static const char *TAG = "ESP_ZIGBEE_CHAUFFAGE";

// Variables globales
int16_t last_temperature = INT16_MIN;
int16_t last_humidity = INT16_MIN;
int16_t last_setpoint = INT16_MIN; // Maintenu localement
int16_t input_setpoint = INT16_MIN;
uint16_t input_high_hyst = 10; // 0.1°C par défaut
uint16_t input_low_hyst = 10;  // 0.1°C par défaut
static uint8_t relay_actual_state = 0xFF; // 0xFF = inconnu, 0 = OFF, 1 = ON
static char *update_status = NULL;
static char *status = NULL;
static char *operating_time = NULL; // Format: "Xd Yh Zm"
static uint8_t last_command_sent = 0xFF;

static int s_retry_num = 0;
static bool wifi_failed = false;
static TaskHandle_t zb_task_handle = NULL;
static bool first_request = true;
static bool zigbee_network_initialized = false;
static bool update_status_allocated = false;

/* Compteur global pour les headers Lumi */
static uint8_t lumi_counter = 0x10;

// Prototypes de fonctions
static void esp_zb_task(void *pvParameters);
static httpd_handle_t start_webserver(void);
static void read_thermostat_attributes(void);
static void read_relay_state(void);
static void write_thermostat_attributes(int16_t new_setpoint, uint16_t new_high_hyst, uint16_t new_low_hyst,
                                       bool setpoint_updated, bool hysteresis_high_updated, bool hysteresis_low_updated);
static void set_sensor_mode(const char *mode);
static void set_external_temperature(int16_t setpoint);
static void set_external_humidity(uint8_t humidity_percent);
static void save_settings_to_nvs(void);
static void load_settings_from_nvs(void);
static void send_on_off_command(uint8_t command_id);
static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message);
static esp_err_t post_handler(httpd_req_t *req);
static esp_err_t data_handler(httpd_req_t *req);
static uint8_t construct_lumi_header(uint8_t *buffer, uint8_t counter, uint8_t cmd_len, uint8_t cmd_id);
static void update_server_attributes(void);
void update_attributes_task(void *pvParameters);
static void test_setpoint(void);

static void save_settings_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i16(nvs_handle, "setpoint", last_setpoint);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save setpoint: %s", esp_err_to_name(err));
    }

    err = nvs_set_u16(nvs_handle, "high_hyst", input_high_hyst);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save high_hyst: %s", esp_err_to_name(err));
    }

    err = nvs_set_u16(nvs_handle, "low_hyst", input_low_hyst);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save low_hyst: %s", esp_err_to_name(err));
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Settings saved to NVS: setpoint=%d, high_hyst=%u, low_hyst=%u", 
             last_setpoint, input_high_hyst, input_low_hyst);
}

static void load_settings_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS: %s, using default settings", esp_err_to_name(err));
        last_setpoint = 1900; // 19.0°C
        input_high_hyst = 10; // 0.1°C
        input_low_hyst = 10;  // 0.1°C
        return;
    }

    err = nvs_get_i16(nvs_handle, "setpoint", &last_setpoint);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read setpoint: %s, using default (1900)", esp_err_to_name(err));
        last_setpoint = 1900;
    }

    err = nvs_get_u16(nvs_handle, "high_hyst", &input_high_hyst);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read high_hyst: %s, using default (10)", esp_err_to_name(err));
        input_high_hyst = 10;
    }

    err = nvs_get_u16(nvs_handle, "low_hyst", &input_low_hyst);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read low_hyst: %s, using default (10)", esp_err_to_name(err));
        input_low_hyst = 10;
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Settings loaded from NVS: setpoint=%d, high_hyst=%u, low_hyst=%u", 
             last_setpoint, input_high_hyst, input_low_hyst);
}

static void bdb_start_top_level_commissioning_wrapper(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    case ESP_ZB_BDB_SIGNAL_STEERING:
        ESP_LOGI(TAG, "Signal: %s, Status: %s (0x%x)", esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status), err_status);
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Joined Zigbee network successfully (PAN ID: 0x%04hx, Channel: %d, Short Address: 0x%04hx)",
                    esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            set_sensor_mode("external");  /* Activer le mode external au démarrage */
            read_thermostat_attributes();
            if (last_setpoint != INT16_MIN) {
                ESP_LOGI(TAG, "Applying setpoint from NVS: %.1f °C", last_setpoint / 100.0);
                set_external_temperature(last_setpoint);
            }
            // Mettre le relais à OFF après l'initialisation Zigbee
            send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID);
            read_relay_state();
            zigbee_network_initialized = true;
            xTaskCreate(update_attributes_task, "Update_Attributes", 2048, NULL, 1, NULL);
        } else {
            ESP_LOGW(TAG, "Network %s failed with status: %s (0x%x)", esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status), err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_wrapper, ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
        break;
    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        ESP_LOGW(TAG, "ZDO Device Unavailable detected, status: %s (0x%x)", esp_err_to_name(err_status), err_status);
        if (update_status && update_status_allocated) {
            ESP_LOGI(TAG, "Freeing update_status at address %p due to device unavailability", update_status);
            free(update_status);
            update_status = NULL;
            update_status_allocated = false;
        }
        asprintf(&update_status, "Échec de l'écriture, périphérique indisponible");
        update_status_allocated = true;
        esp_zb_zcl_read_attr_cmd_t read_cmd = {
            .zcl_basic_cmd = {
                .dst_addr_u.addr_short = THERMOSTAT,
                .dst_endpoint = 1,
                .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .clusterID = 0xFCC0,
            .manuf_specific = 1,
            .manuf_code = MANUFACTURER_CODE,
            .attr_number = 1,
            .attr_field = (uint16_t[]){0x0172},
        };
        esp_zb_zcl_read_attr_cmd_req(&read_cmd);
        ESP_LOGI(TAG, "Sent read request for mode attribute (0x0172)");
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s (0x%x)", 
                 esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status), err_status);
        break;
    }
}

static void send_on_off_command(uint8_t command_id)
{
    if (command_id == last_command_sent) {
        ESP_LOGI(TAG, "Skipping %s command to Shelly relay (0x%04x, endpoint %d): already in this state",
                 (command_id == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", RELAY_CHAUFF, RELAY_BINDING_EP);
        return;
    }

    esp_zb_zcl_on_off_cmd_t cmd_req;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = RELAY_CHAUFF;
    cmd_req.zcl_basic_cmd.dst_endpoint = RELAY_BINDING_EP;
    cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd_req.on_off_cmd_id = command_id;

    ESP_LOGI(TAG, "Sending %s command to Shelly relay (0x%04x, endpoint %d)", 
             (command_id == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", RELAY_CHAUFF, RELAY_BINDING_EP);

    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t tsn = esp_zb_zcl_on_off_cmd_req(&cmd_req);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Sent %s command to Shelly relay (0x%04x, endpoint %d) with TSN 0x%02x", 
             (command_id == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", RELAY_CHAUFF, RELAY_BINDING_EP, tsn);

    last_command_sent = command_id;
}

static void read_relay_state(void)
{
    esp_zb_zcl_read_attr_cmd_t read_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = RELAY_CHAUFF,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        .manuf_specific = 0,
        .attr_number = 1,
        .attr_field = (uint16_t[]){0x0000},
    };
    esp_zb_zcl_read_attr_cmd_req(&read_cmd);
    ESP_LOGI(TAG, "Sent read request for relay state (0x0000)");
}

static void read_thermostat_attributes(void)
{
    // Lecture de la température locale
    esp_zb_zcl_read_attr_cmd_t cmd_temp = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = THERMOSTAT,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .manuf_specific = 0,
        .manuf_code = MANUFACTURER_CODE,
        .attr_number = 1,
        .attr_field = (uint16_t[]){ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID},
    };
    esp_zb_zcl_read_attr_cmd_req(&cmd_temp);

    // Lecture de l'humidité
    esp_zb_zcl_read_attr_cmd_t cmd_humidity = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = THERMOSTAT,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        .manuf_specific = 0,
        .manuf_code = MANUFACTURER_CODE,
        .attr_number = 1,
        .attr_field = (uint16_t[]){ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID},
    };
    esp_zb_zcl_read_attr_cmd_req(&cmd_humidity);

    // Lecture du mode capteur
    esp_zb_zcl_read_attr_cmd_t cmd_specific = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = THERMOSTAT,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = 0xFCC0,
        .manuf_specific = 1,
        .manuf_code = MANUFACTURER_CODE,
        .attr_number = 1,
        .attr_field = (uint16_t[]){0x0172}, // sensor
    };
    esp_zb_zcl_read_attr_cmd_req(&cmd_specific);

    ESP_LOGI(TAG, "Sent read requests for thermostat attributes");
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message) 
{ 
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message"); 
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)", message->status);
    ESP_LOGI(TAG, "Received report from address(0x%04x) src endpoint(%d) to dst endpoint(%d) cluster(0x%04x)", 
         message->src_address.u.short_addr, message->src_endpoint, message->dst_endpoint, message->cluster);
    ESP_LOGI(TAG, "Report information: attribute(0x%04x), type(0x%02x), value(%d)", 
            message->attribute.id, message->attribute.data.type, 
            message->attribute.data.value ? *(int16_t *)message->attribute.data.value : 0);

    if (message->src_address.u.short_addr == THERMOSTAT) {
        if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID) {
                last_temperature = *(int16_t *)message->attribute.data.value;
                ESP_LOGI(TAG, "Thermostat 0x%04x Température: %d.%d °C", 
                        message->src_address.u.short_addr, 
                        last_temperature / 100, abs(last_temperature % 100));
            }
        } else if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID) {
                last_humidity = *(uint16_t *)message->attribute.data.value;
                ESP_LOGI(TAG, "Thermostat 0x%04x Humidité: %d.%d %%", 
                        message->src_address.u.short_addr, 
                        last_humidity / 100, abs(last_humidity % 100));
            }
        } else if (message->cluster == 0xFCC0) {
            if (message->attribute.id == 0x0172) {
                uint8_t sensor_mode = *(uint8_t *)message->attribute.data.value;
                const char *mode_str = (sensor_mode == 2 || sensor_mode == 3) ? "external" : "internal";
                ESP_LOGI(TAG, "Thermostat 0x%04x Sensor mode: %s (raw: %u)", 
                        message->src_address.u.short_addr, mode_str, sensor_mode);
            }
        } else if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_MULTI_INPUT) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID) {
                uint16_t button_value = *(uint16_t *)message->attribute.data.value;
                if (button_value == 1) {
                    if (message->src_endpoint == 1) {
                        ESP_LOGI(TAG, "Button + pressed (single_plus, endpoint 1)");
                        if (last_setpoint != INT16_MIN) {
                            int16_t new_setpoint = last_setpoint + 10; // Augmenter de 0.1°C
                            if (new_setpoint <= ESP_ZB_ZCL_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_MAX_VALUE) {
                                set_external_temperature(new_setpoint);
                                last_setpoint = new_setpoint;
                                save_settings_to_nvs();
                                ESP_LOGI(TAG, "Setpoint increased to %d.%d °C", new_setpoint / 100, abs(new_setpoint % 100));
                            } else {
                                ESP_LOGW(TAG, "Setpoint not increased, max limit reached (%d)", new_setpoint);
                            }
                        }
                    } else if (message->src_endpoint == 2) {
                        ESP_LOGI(TAG, "Button center pressed (single_center, endpoint 2)");
                        // Action pour le bouton central (par exemple, toggle relais ou mode)
                        send_on_off_command(relay_actual_state == 1 ? ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID : ESP_ZB_ZCL_CMD_ON_OFF_ON_ID);
                        read_relay_state();
                    } else if (message->src_endpoint == 3) {
                        ESP_LOGI(TAG, "Button - pressed (single_minus, endpoint 3)");
                        if (last_setpoint != INT16_MIN) {
                            int16_t new_setpoint = last_setpoint - 10; // Diminuer de 0.1°C
                            if (new_setpoint >= ESP_ZB_ZCL_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_MIN_VALUE) {
                                set_external_temperature(new_setpoint);
                                last_setpoint = new_setpoint;
                                save_settings_to_nvs();
                                ESP_LOGI(TAG, "Setpoint decreased to %d.%d °C", new_setpoint / 100, abs(new_setpoint % 100));
                            } else {
                                ESP_LOGW(TAG, "Setpoint not decreased, min limit reached (%d)", new_setpoint);
                            }
                        }
                    }
                }
            }
        }

        test_setpoint(); // Appeler la fonction de test du setpoint

        // Mettre à jour les attributs serveurs après réception des données
        if (zigbee_network_initialized) {
            update_server_attributes();
        }
    }

    if (message->src_address.u.short_addr == RELAY_CHAUFF && message->cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
            relay_actual_state = (*(uint8_t *)message->attribute.data.value != 0) ? 1 : 0;
            ESP_LOGI(TAG, "Relay 0x%04x On/Off state: %s", 
                    message->src_address.u.short_addr, 
                    (relay_actual_state == 1) ? "ON" : "OFF");
            if (zigbee_network_initialized) {
                update_server_attributes();
            }
        }
    }

    return ESP_OK;
}

static void test_setpoint(void)
{
        // Logique du relais avec last_setpoint, input_high_hyst et input_low_hyst
        if (last_temperature != INT16_MIN && last_setpoint != INT16_MIN && 
            input_high_hyst != 0 && input_low_hyst != 0) {
            if (last_temperature <= last_setpoint - (int16_t)input_low_hyst) {
                send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_ON_ID);
                read_relay_state();
            } else if (last_temperature >= last_setpoint + (int16_t)input_high_hyst) {
                send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID);
                read_relay_state();
            }
        } else {
            ESP_LOGI(TAG, "Waiting for all data: temp=%d, setpoint=%d, high_hyst=%u, low_hyst=%u",
                     last_temperature, last_setpoint, input_high_hyst, input_low_hyst);
        }

}

static void reset_update_status_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(5000)); // Attendre 5 secondes
    if (update_status && update_status_allocated) {
        ESP_LOGI(TAG, "Freeing update_status at address %p after timeout", update_status);
        free(update_status);
        update_status = NULL;
        update_status_allocated = false;
    }
    vTaskDelete(NULL);
}

static void write_thermostat_attributes(int16_t new_setpoint, uint16_t new_high_hyst, uint16_t new_low_hyst,
                                       bool setpoint_updated, bool hysteresis_high_updated, bool hysteresis_low_updated)
{
    // Réinitialiser update_status
    if (update_status && update_status_allocated) {
        ESP_LOGI(TAG, "Freeing update_status at address %p", update_status);
        free(update_status);
        update_status = NULL;
        update_status_allocated = false;
    }

    // Mettre à jour le setpoint via Zigbee si nécessaire
    if (setpoint_updated) {
        set_external_temperature(new_setpoint);
        last_setpoint = new_setpoint; // Mettre à jour localement
        save_settings_to_nvs(); // Sauvegarder dans NVS
        test_setpoint();
        asprintf(&update_status, "Setpoint modifié avec succès");
        update_status_allocated = true;
        ESP_LOGI(TAG, "Update status set to: %s", update_status);
        xTaskCreate(reset_update_status_task, "Reset_Update_Status", 2048, NULL, 1, NULL);
    }

    // Mettre à jour les hystérésis localement et dans NVS
    if (hysteresis_high_updated) {
        input_high_hyst = new_high_hyst; // Mettre à jour localement
        save_settings_to_nvs(); // Sauvegarder dans NVS
        ESP_LOGI(TAG, "High hysteresis updated locally: %.1f °C", input_high_hyst / 100.0);
    }

    if (hysteresis_low_updated) {
        input_low_hyst = new_low_hyst; // Mettre à jour localement
        save_settings_to_nvs(); // Sauvegarder dans NVS
        ESP_LOGI(TAG, "Low hysteresis updated locally: %.1f °C", input_low_hyst / 100.0);
    }

    if (!setpoint_updated && !hysteresis_high_updated && !hysteresis_low_updated) {
        ESP_LOGW(TAG, "No attributes to write, skipping operation");
        return;
    }
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
    {
        esp_zb_zcl_cmd_default_resp_message_t *resp = (esp_zb_zcl_cmd_default_resp_message_t *)message;
        ESP_LOGI(TAG, "Received ZCL Default Response from address(0x%04x) endpoint(%d) cluster(0x%04x) command(0x%02x) status(0x%02x)",
                resp->info.src_address.u.short_addr, resp->info.src_endpoint, resp->info.cluster, 
                resp->resp_to_cmd, resp->status_code);
        if (resp->info.src_address.u.short_addr == THERMOSTAT && resp->info.cluster == 0xFCC0) {
            if (resp->status_code == 0) {
                ESP_LOGI(TAG, "Write command to thermostat (0x%04x) succeeded", THERMOSTAT);
                if (update_status && update_status_allocated) {
                    ESP_LOGI(TAG, "Freeing update_status at address %p", update_status);
                    free(update_status);
                    update_status = NULL;
                    update_status_allocated = false;
                }
                asprintf(&update_status, "Setpoint modifié avec succès");
                update_status_allocated = true;
                ESP_LOGI(TAG, "Update status set to: %s", update_status);
                // Lancer la tâche pour réinitialiser update_status après 5 secondes
                xTaskCreate(reset_update_status_task, "Reset_Update_Status", 2048, NULL, 1, NULL);
                read_thermostat_attributes();
            } else {
                ESP_LOGE(TAG, "Write failed for thermostat (0x%04x), status: 0x%02x", THERMOSTAT, resp->status_code);
                if (update_status && update_status_allocated) {
                    ESP_LOGI(TAG, "Freeing update_status at address %p due to write failure", update_status);
                    free(update_status);
                    update_status = NULL;
                    update_status_allocated = false;
                }
                asprintf(&update_status, "Échec de l'écriture, statut: 0x%02x", resp->status_code);
                update_status_allocated = true;
                // Lancer la tâche pour réinitialiser update_status après 5 secondes
                xTaskCreate(reset_update_status_task, "Reset_Update_Status", 2048, NULL, 1, NULL);
                read_thermostat_attributes();
            }
        } else if (resp->info.src_address.u.short_addr == RELAY_CHAUFF && resp->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            ESP_LOGI(TAG, "Command %s (0x%02x) to Relay (0x%04x) %s",
                    (resp->resp_to_cmd == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", resp->resp_to_cmd,
                    RELAY_CHAUFF, (resp->status_code == 0) ? "succeeded" : "failed");
        }
        break;
    }
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
    {
        esp_zb_zcl_cmd_read_attr_resp_message_t *resp = (esp_zb_zcl_cmd_read_attr_resp_message_t *)message;
        ESP_LOGI(TAG, "Received Read Attribute Response from address(0x%04x) endpoint(%d) cluster(0x%04x)",
                resp->info.src_address.u.short_addr, resp->info.src_endpoint, resp->info.cluster);
        esp_zb_zcl_read_attr_resp_variable_t *variable = resp->variables;
        while (variable != NULL) {
            if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS) {
                if (resp->info.cluster == 0xFCC0 && variable->attribute.id == 0x0172) {
                    uint8_t mode = *(uint8_t *)variable->attribute.data.value;
                    ESP_LOGI(TAG, "Read mode attribute (0x0172, cluster 0xFCC0): %d", mode);
                    if (mode == 0x02 || mode == 0x03) {
                        ESP_LOGI(TAG, "Thermostat in external sensor mode");
                    } else {
                        ESP_LOGW(TAG, "Thermostat not in external sensor mode (mode=%d)", mode);
                    }
                }
            } else {
                ESP_LOGW(TAG, "Attribute 0x%04x read failed with status 0x%02x", variable->attribute.id, variable->status);
            }
            variable = variable->next;
        }
        break;
    }
    default:
        ESP_LOGW(TAG, "Received Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static esp_err_t example_set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
{
    if (addr && (addr != IPADDR_NONE)) {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = addr;
        dns.ip.type = IPADDR_TYPE_V4;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
    }
    return ESP_OK;
}

static void example_set_static_ip(esp_netif_t *netif)
{
    if (esp_netif_dhcpc_stop(netif) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop dhcp client");
        return;
    }
    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(EXAMPLE_STATIC_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(EXAMPLE_STATIC_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(EXAMPLE_STATIC_GW_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ip info");
        return;
    }
    ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", EXAMPLE_STATIC_IP_ADDR, EXAMPLE_STATIC_NETMASK_ADDR, EXAMPLE_STATIC_GW_ADDR);
    ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    esp_netif_t *netif = (esp_netif_t *)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        example_set_static_ip(netif);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Disconnected from Wi-Fi, retrying... (Attempt %d/%d)", s_retry_num + 1, WIFI_MAX_RETRIES);
        ESP_LOGE(TAG, "!! STA Disconnected! Reason: %d", event->reason);

        if (s_retry_num < WIFI_MAX_RETRIES) {
            s_retry_num++;
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "Max Wi-Fi retries reached (%d), starting Zigbee fallback.", WIFI_MAX_RETRIES);
            wifi_failed = true;
            if (zb_task_handle == NULL) {
                xTaskCreate(esp_zb_task, "Zigbee_main", 4096 * 4, NULL, 2, &zb_task_handle);
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected to Wi-Fi, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        ESP_LOGI(TAG, "Free heap size after IP: %lu bytes", esp_get_free_heap_size());
        if (zb_task_handle == NULL) {
            xTaskCreate(esp_zb_task, "Zigbee_main", 4096 * 4, NULL, 2, &zb_task_handle);
        }
        httpd_handle_t server = start_webserver();
        if (server == NULL) {
            ESP_LOGE(TAG, "Failed to start web server");
        } else {
            ESP_LOGI(TAG, "Web server started successfully");
        }
        if (esp_coex_wifi_i154_enable() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable Wi-Fi and 802.15.4 coexistence");
        } else {
            ESP_LOGI(TAG, "Wi-Fi and 802.15.4 coexistence enabled");
        }
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        sta_netif,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        sta_netif,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static esp_err_t get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Received request: URI=%s, Content-Length=%d, last_temperature=%d, last_humidity=%d",
             req->uri, req->content_len, last_temperature, last_humidity);

    FILE *f = fopen("/spiffs/index.html", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open index.html");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    rewind(f);
    ESP_LOGI(TAG, "Index.html size: %ld bytes", file_size);
    if (file_size > 20000) {
        ESP_LOGE(TAG, "Index.html too large (%" PRId32 " bytes), max is 20000 bytes", (int32_t)file_size);
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *response = (char *)calloc(20000, 1);
    if (response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for response");
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    char *updated_response = (char *)calloc(20000, 1);
    if (updated_response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for updated_response");
        free(response);
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t len = fread(response, 1, 20000, f);
    fclose(f);
    ESP_LOGI(TAG, "Read %u bytes from index.html", len);

    if (len <= 0) {
        ESP_LOGE(TAG, "Failed to read index.html");
        free(response);
        free(updated_response);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    response[len] = '\0';

    first_request = true;

    char temp_str[16], humi_str[16], setpoint_str[16], relay_str[4], running_state_str[5];
    snprintf(temp_str, sizeof(temp_str), "%.1f", last_temperature != INT16_MIN ? last_temperature / 100.0 : 0.0);
    snprintf(humi_str, sizeof(humi_str), "%.1f", last_humidity != INT16_MIN ? last_humidity / 100.0 : 0.0);
    snprintf(setpoint_str, sizeof(setpoint_str), "%.1f", last_setpoint != INT16_MIN ? last_setpoint / 100.0 : 0.0);
    snprintf(relay_str, sizeof(relay_str), "%s", last_command_sent == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID ? "ON" : "OFF");

    int res_len = snprintf(updated_response, 20000,
                           response,
                           temp_str, humi_str, setpoint_str, relay_str, running_state_str, setpoint_str);
    if (res_len >= 20000) {
        ESP_LOGE(TAG, "Buffer overflow in get_handler, response truncated, res_len=%d", res_len);
        free(response);
        free(updated_response);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, updated_response, strlen(updated_response));
    free(response);
    free(updated_response);
    return ESP_OK;
}

static esp_err_t favicon_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Received request for favicon: URI=%s", req->uri);

    FILE *f = fopen("/spiffs/favicon.ico", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open favicon.ico");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    rewind(f);
    ESP_LOGI(TAG, "Favicon.ico size: %ld bytes", file_size);
    if (file_size > 10000) {
        ESP_LOGE(TAG, "Favicon.ico too large (%" PRId32 " bytes), max is 10000 bytes", (int32_t)file_size);
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *response = (char *)calloc(file_size + 1, 1);
    if (response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for favicon response");
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t len = fread(response, 1, file_size, f);
    fclose(f);
    if (len <= 0) {
        ESP_LOGE(TAG, "Failed to read favicon.ico");
        free(response);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, response, len);
    free(response);
    return ESP_OK;
}

/* Fonction pour gérer les requêtes POST */
static esp_err_t post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Received POST request: URI=%s, Content-Length=%d", req->uri, req->content_len);

    char buf[100];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)-1));
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        buf[ret] = '\0';
        remaining -= ret;
    }

    // Réinitialiser update_status avant de traiter la nouvelle requête
    if (update_status && update_status_allocated) {
        ESP_LOGI(TAG, "Freeing update_status at address %p before new POST", update_status);
        free(update_status);
        update_status = NULL;
        update_status_allocated = false;
    }

    char setpoint_str[16], high_hyst_str[16], low_hyst_str[16];
    int16_t new_setpoint = last_setpoint;
    uint16_t new_high_hyst = input_high_hyst;
    uint16_t new_low_hyst = input_low_hyst;
    bool setpoint_updated = false;
    bool hysteresis_high_updated = false;
    bool hysteresis_low_updated = false;

    if (httpd_query_key_value(buf, "setpoint", setpoint_str, sizeof(setpoint_str)) == ESP_OK) {
        new_setpoint = (int16_t)(atof(setpoint_str) * 100 + 0.05);
        ESP_LOGI(TAG, "New setpoint received: %.1f °C, stored as %d", atof(setpoint_str), new_setpoint);
        setpoint_updated = true;
    }

    if (httpd_query_key_value(buf, "high_hyst", high_hyst_str, sizeof(high_hyst_str)) == ESP_OK) {
        new_high_hyst = (uint16_t)(atof(high_hyst_str) * 100 + 0.05);
        ESP_LOGI(TAG, "New high hysteresis received: %.1f °C, stored as %u", atof(high_hyst_str), new_high_hyst);
        hysteresis_high_updated = true;
    }

    if (httpd_query_key_value(buf, "low_hyst", low_hyst_str, sizeof(low_hyst_str)) == ESP_OK) {
        new_low_hyst = (uint16_t)(atof(low_hyst_str) * 100 + 0.05);
        ESP_LOGI(TAG, "New low hysteresis received: %.1f °C, stored as %u", atof(low_hyst_str), new_low_hyst);
        hysteresis_low_updated = true;
    }

    // Exécuter write_thermostat_attributes directement
    if (setpoint_updated || hysteresis_high_updated || hysteresis_low_updated) {
        write_thermostat_attributes(
            new_setpoint,
            new_high_hyst,
            new_low_hyst,
            setpoint_updated,
            hysteresis_high_updated,
            hysteresis_low_updated
        );
    }

    char temp_str[16], humi_str[16], setpoint_str_resp[16], relay_actual_str[16], relay_commanded_str[16];
    char high_hyst_str_resp[16], low_hyst_str_resp[16];
    snprintf(temp_str, sizeof(temp_str), "%.1f", last_temperature != INT16_MIN ? last_temperature / 100.0 : 0.0);
    snprintf(humi_str, sizeof(humi_str), "%.1f", last_humidity != INT16_MIN ? last_humidity / 100.0 : 0.0);
    snprintf(setpoint_str_resp, sizeof(setpoint_str_resp), "%.1f", new_setpoint != INT16_MIN ? new_setpoint / 100.0 : 0.0);
    snprintf(relay_actual_str, sizeof(relay_actual_str), "%s", (relay_actual_state != 0xFF) ? 
             ((relay_actual_state == 1) ? "ON" : "OFF") : "N/A");
    snprintf(relay_commanded_str, sizeof(relay_commanded_str), "%s", (last_command_sent != 0xFF) ? 
             ((last_command_sent == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF") : "N/A");
    snprintf(high_hyst_str_resp, sizeof(high_hyst_str_resp), "%.1f", new_high_hyst != 0 ? new_high_hyst / 100.0 : 0.0);
    snprintf(low_hyst_str_resp, sizeof(low_hyst_str_resp), "%.1f", new_low_hyst != 0 ? new_low_hyst / 100.0 : 0.0);

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON_AddStringToObject(root, "temperature", temp_str);
    cJSON_AddStringToObject(root, "humidity", humi_str);
    cJSON_AddStringToObject(root, "setpoint", setpoint_str_resp);
    cJSON_AddStringToObject(root, "relay_actual", relay_actual_str);
    cJSON_AddStringToObject(root, "relay_commanded", relay_commanded_str);
    cJSON_AddStringToObject(root, "update_status", update_status ? update_status : "");
    cJSON_AddStringToObject(root, "high_hyst", high_hyst_str_resp);
    cJSON_AddStringToObject(root, "low_hyst", low_hyst_str_resp);

    char *json_response = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json_response) {
        ESP_LOGE(TAG, "Failed to allocate JSON response");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sending JSON response after POST: %s", json_response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, strlen(json_response));
    free(json_response);

    return ESP_OK;
}

static esp_err_t data_handler(httpd_req_t *req)
{
    static int16_t last_temperature_sent = INT16_MIN;
    static int16_t last_humidity_sent = INT16_MIN;
    static int16_t last_setpoint_sent = INT16_MIN;
    static uint8_t last_relay_actual_state_sent = 0xFF;
    static uint16_t last_high_hyst_sent = 0;
    static uint16_t last_low_hyst_sent = 0;
    static char *last_update_status_sent = NULL;

    bool data_changed = false;

    if (first_request || 
        last_temperature_sent != last_temperature ||
        last_humidity_sent != last_humidity ||
        last_setpoint_sent != last_setpoint ||
        last_relay_actual_state_sent != relay_actual_state ||
        last_high_hyst_sent != input_high_hyst ||
        last_low_hyst_sent != input_low_hyst ||
        (update_status && !last_update_status_sent) ||
        (!update_status && last_update_status_sent) ||
        (update_status && last_update_status_sent && strcmp(update_status, last_update_status_sent) != 0)) {
        data_changed = true;
    }

    if (!data_changed) {
        ESP_LOGD(TAG, "No data change detected, skipping JSON response");
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }

    char temp_str[16], humi_str[16], setpoint_str[16], relay_actual_str[16], relay_commanded_str[16], high_hyst_str[16], low_hyst_str[16];
    snprintf(temp_str, sizeof(temp_str), "%.1f", last_temperature != INT16_MIN ? last_temperature / 100.0 : 0.0);
    snprintf(humi_str, sizeof(temp_str), "%.1f", last_humidity != INT16_MIN ? last_humidity / 100.0 : 0.0);
    snprintf(setpoint_str, sizeof(setpoint_str), "%.1f", last_setpoint != INT16_MIN ? last_setpoint / 100.0 : 0.0);
    snprintf(relay_actual_str, sizeof(relay_actual_str), "%s", (relay_actual_state != 0xFF) ? 
             ((relay_actual_state == 1) ? "ON" : "OFF") : "N/A");
    snprintf(relay_commanded_str, sizeof(relay_commanded_str), "%s", (last_command_sent != 0xFF) ? 
             ((last_command_sent == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF") : "N/A");
    snprintf(high_hyst_str, sizeof(high_hyst_str), "%.1f", input_high_hyst / 100.0);
    snprintf(low_hyst_str, sizeof(low_hyst_str), "%.1f", input_low_hyst / 100.0);

    char *json_response = NULL;
    int res_len = asprintf(&json_response, "{\"temperature\":\"%s\",\"humidity\":\"%s\",\"setpoint\":\"%s\",\"relay_actual\":\"%s\",\"relay_commanded\":\"%s\",\"high_hyst\":\"%s\",\"low_hyst\":\"%s\",\"update_status\":\"%s\"}",
                           temp_str, humi_str, setpoint_str, relay_actual_str, relay_commanded_str, high_hyst_str, low_hyst_str, update_status ? update_status : "");
    if (res_len < 0 || json_response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate JSON response, sending default JSON");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"error\":\"Failed to generate JSON\",\"temperature\":\"0.0\",\"humidity\":\"0.0\",\"setpoint\":\"0.0\",\"relay_actual\":\"N/A\",\"relay_commanded\":\"N/A\",\"high_hyst\":\"0.1\",\"low_hyst\":\"0.1\",\"update_status\":\"\"}", 139);
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, strlen(json_response));
    
    last_temperature_sent = last_temperature;
    last_humidity_sent = last_humidity;
    last_setpoint_sent = last_setpoint;
    last_relay_actual_state_sent = relay_actual_state;
    last_high_hyst_sent = input_high_hyst;
    last_low_hyst_sent = input_low_hyst;
    if (last_update_status_sent) free(last_update_status_sent);
    last_update_status_sent = update_status ? strdup(update_status) : NULL;
    first_request = false;

    free(json_response);
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    char *json_response = NULL;
    int res_len = asprintf(&json_response, "{\"status\":\"%s\"}", status ? status : "");
    if (res_len < 0 || json_response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate JSON response, sending default JSON");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"error\":\"Failed to generate JSON\",\"status\":\"\"}", 166);
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, strlen(json_response));
    
    free(json_response);
    return ESP_OK;
}

static esp_err_t operating_time_handler(httpd_req_t *req)
{
    char *res_car = operating_time ? operating_time : "0d 00h 00m";

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, res_car, strlen(res_car));
    
    // Do not free res_car, as it may point to a string literal or global buffer
    return ESP_OK;
}


static httpd_handle_t start_webserver(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        return NULL;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 114688;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t get_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = get_handler,
            .user_ctx = NULL
        };
        httpd_uri_t favicon_uri = {
            .uri = "/favicon.ico",
            .method = HTTP_GET,
            .handler = favicon_handler,
            .user_ctx = NULL
        };
        httpd_uri_t post_uri = {
            .uri = "/update",
            .method = HTTP_POST,
            .handler = post_handler,
            .user_ctx = NULL
        };
        httpd_uri_t data_uri = {
            .uri = "/data",
            .method = HTTP_GET,
            .handler = data_handler,
            .user_ctx = NULL
        };
        httpd_uri_t status_uri = {
            .uri = "/status",
            .method = HTTP_GET,
            .handler = status_handler,
            .user_ctx = NULL
        };
        httpd_uri_t operating_time_uri = {
            .uri = "/operating_time",
            .method = HTTP_GET,
            .handler = operating_time_handler,
            .user_ctx = NULL            
        };

        httpd_register_uri_handler(server, &get_uri);
        httpd_register_uri_handler(server, &favicon_uri);
        httpd_register_uri_handler(server, &post_uri);
        httpd_register_uri_handler(server, &data_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &operating_time_uri);
        ESP_LOGI(TAG, "Web server started on port %d", config.server_port);
    } else {
        ESP_LOGE(TAG, "Failed to start web server");
    }
    return server;
}

static void esp_zb_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Free heap size at Zigbee task start: %lu bytes", esp_get_free_heap_size());
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // Cluster Basic (serveur)
    uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    uint16_t app_version = 1;
    uint16_t stack_version = 0x0003;
    uint8_t hw_version = 1;
    char manu_name[] = ESP_MANUFACTURER_NAME;
    char model_id[] = ESP_MODEL_IDENTIFIER;
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    if (esp_zb_basic_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Basic cluster list");
        return;
    }
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &app_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &stack_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &hw_version);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manu_name);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model_id);

    // Clusters pour le thermostat et le relais (clients)
    esp_zb_attribute_list_t *esp_zb_thermostat_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT);
    if (esp_zb_thermostat_client_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Thermostat client cluster list");
        return;
    }
    esp_zb_attribute_list_t *esp_zb_on_off_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    if (esp_zb_on_off_client_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create On/Off client cluster list");
        return;
    }

    // Clusters pour le binding avec le W100 (clients)
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    if (esp_zb_identify_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Identify cluster list");
        return;
    }
    esp_zb_attribute_list_t *esp_zb_multi_state_input_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_MULTI_INPUT);
    if (esp_zb_multi_state_input_client_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Multistate Input client cluster list");
        return;
    }
    esp_zb_attribute_list_t *esp_zb_ota_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE);
    if (esp_zb_ota_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create OTA cluster list");
        return;
    }
    esp_zb_attribute_list_t *esp_zb_power_cfg_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    if (esp_zb_power_cfg_client_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Power Config client cluster list");
        return;
    }
    esp_zb_attribute_list_t *esp_zb_manu_specific_lumi_cluster = esp_zb_zcl_attr_list_create(0xFCC0); // Cluster personnalisé
    if (esp_zb_manu_specific_lumi_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Manufacturer Specific cluster list");
        return;
    }
    uint8_t mode = 0; // Valeur par défaut
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_cluster, 0x0009, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &mode);
    uint32_t sampling_period = 30000; // 30 secondes (en ms)
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_cluster, 0x0162, ESP_ZB_ZCL_ATTR_TYPE_U32, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &sampling_period);
    uint8_t sensor_type = 0; // 0: interne, 2: externe
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_cluster, 0x0172, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &sensor_type);
    uint8_t control_data = 0; // Données de contrôle pour 0xFFF2
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_cluster, 0xFFF2, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &control_data);
    esp_zb_attribute_list_t *esp_zb_humidity_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    if (esp_zb_humidity_client_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Humidity client cluster list");
        return;
    }
    esp_zb_attribute_list_t *esp_zb_temperature_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    if (esp_zb_temperature_client_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Temperature client cluster list");
        return;
    }

    ////////////////////// Clusters serveurs ////////////////////
    // Initialisation explicite du cluster Basic
    esp_zb_attribute_list_t *esp_zb_on_off_server_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    if (esp_zb_on_off_server_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create On/Off server cluster list");
        return;
    }
    uint8_t on_off_value = 0; // Initialisé à OFF
    esp_zb_zcl_status_t status = esp_zb_on_off_cluster_add_attr(esp_zb_on_off_server_cluster, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &on_off_value);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add On/Off attribute: status 0x%02x", status);
        return;
    }

    // Initialisation explicite du cluster Temperature Measurement
    esp_zb_zcl_temp_measurement_init_server();
    esp_zb_attribute_list_t *esp_zb_temperature_server_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    if (esp_zb_temperature_server_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Temperature server cluster list");
        return;
    }
    int16_t temp_value = 0; // Valeur initiale
    status = esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_server_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temp_value);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Temperature MeasuredValue attribute: status 0x%02x", status);
        return;
    }
    int16_t min_temp_value = ESP_ZB_ZCL_TEMP_MEASUREMENT_MIN_MEASURED_VALUE_MINIMUM; // -273.15°C
    status = esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_server_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &min_temp_value);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Temperature MinMeasuredValue attribute: status 0x%02x", status);
        return;
    }
    int16_t max_temp_value = ESP_ZB_ZCL_TEMP_MEASUREMENT_MAX_MEASURED_VALUE_MAXIMUM; // 327.67°C
    status = esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_server_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &max_temp_value);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Temperature MaxMeasuredValue attribute: status 0x%02x", status);
        return;
    }

    // Initialisation explicite du cluster Relative Humidity Measurement
    esp_zb_zcl_rel_humidity_measurement_init_server();
    esp_zb_attribute_list_t *esp_zb_humidity_server_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    if (esp_zb_humidity_server_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Humidity server cluster list");
        return;
    }
    uint16_t humidity_value = 0; // Valeur initiale
    status = esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_server_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity_value);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Humidity MeasuredValue attribute: status 0x%02x", status);
        return;
    }
    uint16_t min_humidity_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MIN_MEASURED_VALUE_MINIMUM; // 0%
    status = esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_server_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &min_humidity_value);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Humidity MinMeasuredValue attribute: status 0x%02x", status);
        return;
    }
    uint16_t max_humidity_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MAX_MEASURED_VALUE_MAXIMUM; // 100%
    status = esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_server_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &max_humidity_value);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Humidity MaxMeasuredValue attribute: status 0x%02x", status);
        return;
    }

    // Initialisation explicite du cluster Thermostat
    zb_zcl_thermostat_init_server();
    esp_zb_attribute_list_t *esp_zb_thermostat_server_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT);
    if (esp_zb_thermostat_server_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Thermostat server cluster list");
        return;
    }
    int16_t local_temp = 0; // Température locale initiale
    status = esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_server_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, &local_temp);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Thermostat LocalTemperature attribute: status 0x%02x", status);
        return;
    }
    int16_t cool_setpoint = ESP_ZB_ZCL_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_DEFAULT_VALUE;
    status = esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_server_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID, &cool_setpoint);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Thermostat OccupiedCoolingSetpoint attribute: status 0x%02x", status);
        return;
    }
    int16_t heat_setpoint = ESP_ZB_ZCL_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_DEFAULT_VALUE;
    status = esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_server_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &heat_setpoint);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Thermostat OccupiedHeatingSetpoint attribute: status 0x%02x", status);
        return;
    }
    int8_t sequence_operation = ESP_ZB_ZCL_THERMOSTAT_CONTROL_SEQ_OF_OPERATION_HEATING_ONLY; // Séquence d'opération par défaut chauffage seulement
    status = esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_server_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_CONTROL_SEQUENCE_OF_OPERATION_ID, &sequence_operation);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Thermostat ControlSequenceOfOperation attribute: status 0x%02x", status);
        return;
    }
    int8_t sys_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT; // Mode système par défaut chauffage
    status = esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_server_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, &sys_mode);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Thermostat SystemMode attribute: status 0x%02x", status);
        return;
    }

    // Ajout du cluster Multistate Input en mode serveur avec tous les attributs obligatoires
    esp_zb_attribute_list_t *esp_zb_multistate_input_server_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_MULTI_INPUT);
    if (esp_zb_multistate_input_server_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Multistate Input server cluster list");
        return;
    }
    // Attributs obligatoires pour le cluster Multistate Input
    bool out_of_service = ESP_ZB_ZCL_MULTI_INPUT_OUT_OF_SERVICE_DEFAULT_VALUE; // false
    status = esp_zb_multistate_input_cluster_add_attr(esp_zb_multistate_input_server_cluster, 
                                                                         ESP_ZB_ZCL_ATTR_MULTI_INPUT_OUT_OF_SERVICE_ID, 
                                                                         &out_of_service);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Multistate Input OutOfService attribute: status 0x%02x", status);
        return;
    }
    uint16_t present_value = ESP_ZB_ZCL_MULTI_INPUT_PRESENT_VALUE_DEFAULT_VALUE; // 0
    status = esp_zb_multistate_input_cluster_add_attr(esp_zb_multistate_input_server_cluster, 
                                                     ESP_ZB_ZCL_ATTR_MULTI_INPUT_PRESENT_VALUE_ID, 
                                                     &present_value);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Multistate Input PresentValue attribute: status 0x%02x", status);
        return;
    }
    uint8_t status_flags = ESP_ZB_ZCL_MULTI_INPUT_STATUS_FLAGS_DEFAULT_VALUE; // 0
    status = esp_zb_multistate_input_cluster_add_attr(esp_zb_multistate_input_server_cluster, 
                                                     ESP_ZB_ZCL_ATTR_MULTI_INPUT_STATUS_FLAGS_ID, 
                                                     &status_flags);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Multistate Input StatusFlags attribute: status 0x%02x", status);
        return;
    }
    uint16_t number_of_states = 3; // 3 états pour correspondre aux boutons (peut être ajusté selon besoin)
    status = esp_zb_multistate_input_cluster_add_attr(esp_zb_multistate_input_server_cluster, 
                                                     ESP_ZB_ZCL_ATTR_MULTI_INPUT_NUMBER_OF_STATES_ID, 
                                                     &number_of_states);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Multistate Input NumberOfStates attribute: status 0x%02x", status);
        return;
    }    

    ////////////////////// Liste des clusters ////////////////////
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    if (esp_zb_cluster_list == NULL) {
        ESP_LOGE(TAG, "Failed to create cluster list");
        return;
    }
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_multistate_value_cluster(esp_zb_cluster_list, esp_zb_multi_state_input_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_ota_cluster(esp_zb_cluster_list, esp_zb_ota_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_power_config_cluster(esp_zb_cluster_list, esp_zb_power_cfg_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_manu_specific_lumi_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humidity_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_thermostat_cluster(esp_zb_cluster_list, esp_zb_thermostat_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    // Ajout des clusters serveurs
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_cluster_list, esp_zb_temperature_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_cluster_list, esp_zb_humidity_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_thermostat_cluster(esp_zb_cluster_list, esp_zb_thermostat_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_multistate_value_cluster(esp_zb_cluster_list, esp_zb_multistate_input_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);    


    // Création de la liste des endpoints
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    if (esp_zb_ep_list == NULL) {
        ESP_LOGE(TAG, "Failed to create endpoint list");
        return;
    }
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_THERMOSTAT_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(true));
    ESP_LOGI(TAG, "Free heap size after Zigbee start: %lu bytes", esp_get_free_heap_size());

    // Mettre à jour les attributs serveurs initialement
    update_server_attributes();

    while (1) {
        esp_zb_stack_main_loop();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// Fonction update_attributes_task
void update_attributes_task(void *pvParameters)
{
    while (1) {
        update_server_attributes();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// Fonction update_server_attributes
static void update_server_attributes(void)
{
    if (!zigbee_network_initialized) {
        ESP_LOGW(TAG, "Zigbee network not initialized, skipping attribute update");
        return;
    }
    esp_zb_zcl_status_t status;

    // Mettre à jour l'attribut OnOff du cluster On/Off serveur
    uint8_t on_off_value = (relay_actual_state != 0xFF) ? relay_actual_state : 0;
    status = esp_zb_zcl_set_attribute_val(
        HA_ONOFF_SWITCH_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
        &on_off_value,
        false
    );
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to update On/Off attribute: status 0x%02x", status);
    }

    // Mettre à jour l'attribut MeasuredValue du cluster Relative Humidity Measurement serveur
    ESP_LOGI(TAG, "last_humidity = %d", last_humidity); // Log pour débogage
    uint16_t humidity_value = (last_humidity != INT16_MIN && last_humidity >= 0) ? last_humidity : 0;
    if (humidity_value > ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MAX_MEASURED_VALUE_MAXIMUM) { // Plage : 0 à 10000
        ESP_LOGW(TAG, "Humidity value out of range: %u, setting to 0", humidity_value);
        humidity_value = 0;
    }
    status = esp_zb_zcl_set_attribute_val(
        HA_ONOFF_SWITCH_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
        &humidity_value,
        false
    );
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to update Humidity MeasuredValue attribute: status 0x%02x", status);
    }

    // Mettre à jour l'attribut MeasuredValue du cluster Temperature Measurement serveur
    ESP_LOGI(TAG, "last_temperature = %d", last_temperature); // Log pour débogage
    int16_t temp_value = (last_temperature != INT16_MIN && last_temperature >= ESP_ZB_ZCL_TEMP_MEASUREMENT_MIN_MEASURED_VALUE_MINIMUM) ? last_temperature : 0;
    status = esp_zb_zcl_set_attribute_val(
        HA_ONOFF_SWITCH_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &temp_value,
        false
    );
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to update Temperature MeasuredValue attribute: status 0x%02x", status);
    }

    ESP_LOGI(TAG, "Updated server attributes: OnOff=%u, Temp=%d, Humidity=%u",
             on_off_value, temp_value, humidity_value);
}

/* Fonction pour construire le header Lumi */
static uint8_t construct_lumi_header(uint8_t *header, uint8_t counter, uint8_t params_length, uint8_t action) {
    header[0] = 0xAA;
    header[1] = 0x71;
    header[2] = params_length + 3;
    header[3] = 0x44;
    header[4] = counter;
    int sum = header[0] + header[1] + header[2] + header[3] + header[4];
    header[5] = 512 - sum;
    header[6] = action;
    header[7] = 0x41;
    header[8] = params_length;
    return 9;  /* Taille du header */
}

/* Fonction pour définir le mode du capteur (internal ou external) */
static void set_sensor_mode(const char *mode) {
    bool is_external = (strcmp(mode, "external") == 0);
    uint8_t action = is_external ? 0x02 : 0x04;

    /* Timestamp BE (uptime en secondes) */
    uint64_t us = esp_timer_get_time();
    uint32_t sec = (uint32_t)(us / 1000000);
    uint8_t timestamp[4] = { (sec >> 24) & 0xFF, (sec >> 16) & 0xFF, (sec >> 8) & 0xFF, sec & 0xFF };

    uint8_t device_ieee[8] = THERMOSTAT_IEEE;
    uint8_t fictive_sensor[8] = {0x00, 0x15, 0x8D, 0x00, 0x01, 0x9D, 0x1B, 0x98};
    uint8_t chinese_humi[6] = {0xE6, 0xB9, 0xBF, 0xE5, 0xBA, 0xA6};  /* "湿度" */
    uint8_t chinese_temp[6] = {0xE6, 0xB8, 0xA9, 0xE5, 0xBA, 0xA6};  /* "温度" */

    /* Params pour humidity (0x15) */
    uint8_t params_humi[4 + 1 + 8 + 8 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 6 + 5 + 1 + 1 + 1 + 1];  /* Max size 45 */
    int idx = 0;
    memcpy(params_humi + idx, timestamp, 4); idx += 4;
    params_humi[idx++] = 0x15;
    memcpy(params_humi + idx, device_ieee, 8); idx += 8;
    if (is_external) {
        memcpy(params_humi + idx, fictive_sensor, 8); idx += 8;
        params_humi[idx++] = 0x00;
        params_humi[idx++] = 0x02;
        params_humi[idx++] = 0x00;
        params_humi[idx++] = 0x55;
        params_humi[idx++] = 0x15;
        params_humi[idx++] = 0x0A;
        params_humi[idx++] = 0x01;
        params_humi[idx++] = 0x00;
        params_humi[idx++] = 0x00;
        params_humi[idx++] = 0x01;
        params_humi[idx++] = 0x06;
        memcpy(params_humi + idx, chinese_humi, 6); idx += 6;
        memset(params_humi + idx, 0x00, 5); idx += 5;
        params_humi[idx++] = 0x01;
        params_humi[idx++] = 0x02;
        params_humi[idx++] = 0x08;
        params_humi[idx++] = 0x65;
    } else {
        memset(params_humi + idx, 0x00, 12); idx += 12;
    }
    uint8_t params_humi_len = idx;

    /* Construire val pour humi */
    uint8_t header[9];
    uint8_t header_len = construct_lumi_header(header, lumi_counter++, params_humi_len, action);
    uint8_t val_humi[header_len + params_humi_len];
    memcpy(val_humi, header, header_len);
    memcpy(val_humi + header_len, params_humi, params_humi_len);

    /* Préparer l'attribut octet string (length + data) */
    uint8_t attr_buf_humi[1 + sizeof(val_humi)];
    attr_buf_humi[0] = sizeof(val_humi);
    memcpy(attr_buf_humi + 1, val_humi, attr_buf_humi[0]);

    esp_zb_zcl_attribute_t attr = {
        .id = 0xFFF2,
        .data.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
        .data.value = attr_buf_humi
    };

    esp_zb_zcl_write_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = THERMOSTAT,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = 0xFCC0,
        .manuf_specific = 1,
        .manuf_code = MANUFACTURER_CODE,
        .attr_number = 1,
        .attr_field = &attr,
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_write_attr_cmd_req(&cmd);
    esp_zb_lock_release();

    /* Params pour temperature (0x14) - similaire */
    uint8_t params_temp[4 + 1 + 8 + 8 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 6 + 5 + 1 + 1 + 1 + 1];
    idx = 0;
    memcpy(params_temp + idx, timestamp, 4); idx += 4;
    params_temp[idx++] = 0x14;
    memcpy(params_temp + idx, device_ieee, 8); idx += 8;
    if (is_external) {
        memcpy(params_temp + idx, fictive_sensor, 8); idx += 8;
        params_temp[idx++] = 0x00;
        params_temp[idx++] = 0x01;
        params_temp[idx++] = 0x00;
        params_temp[idx++] = 0x55;
        params_temp[idx++] = 0x15;
        params_temp[idx++] = 0x0A;
        params_temp[idx++] = 0x01;
        params_temp[idx++] = 0x00;
        params_temp[idx++] = 0x00;
        params_temp[idx++] = 0x01;
        params_temp[idx++] = 0x06;
        memcpy(params_temp + idx, chinese_temp, 6); idx += 6;
        memset(params_temp + idx, 0x00, 5); idx += 5;
        params_temp[idx++] = 0x01;
        params_temp[idx++] = 0x02;
        params_temp[idx++] = 0x07;
        params_temp[idx++] = 0x63;
    } else {
        memset(params_temp + idx, 0x00, 12); idx += 12;
    }
    uint8_t params_temp_len = idx;

    /* Construire val pour temp */
    header_len = construct_lumi_header(header, lumi_counter++, params_temp_len, action);
    uint8_t val_temp[header_len + params_temp_len];
    memcpy(val_temp, header, header_len);
    memcpy(val_temp + header_len, params_temp, params_temp_len);

    uint8_t attr_buf_temp[1 + sizeof(val_temp)];
    attr_buf_temp[0] = sizeof(val_temp);
    memcpy(attr_buf_temp + 1, val_temp, attr_buf_temp[0]);

    attr.data.value = attr_buf_temp;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_write_attr_cmd_req(&cmd);
    esp_zb_lock_release();

    /* Lire le mode pour confirmation */
    esp_zb_zcl_read_attr_cmd_t read_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = THERMOSTAT,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = 0xFCC0,
        .manuf_specific = 1,
        .manuf_code = MANUFACTURER_CODE,
        .attr_number = 1,
        .attr_field = (uint16_t[]){0x0172},
    };
    esp_zb_zcl_read_attr_cmd_req(&read_cmd);

    ESP_LOGI(TAG, "Mode sensor défini sur %s", mode);
}

/* Fonction pour définir external_temperature (= setpoint) */
static void set_external_temperature(int16_t setpoint)
{
    uint8_t fictive_sensor[8] = {0x00, 0x15, 0x8D, 0x00, 0x01, 0x9D, 0x1B, 0x98};

    /* Préparer le buffer float BE (setpoint est déjà en centièmes, e.g. 1900 pour 19.0°C) */
    float f = (float)setpoint;
    union { float fl; uint32_t u; } fu;
    fu.fl = f;
    uint8_t temp_buf[4] = { (fu.u >> 24) & 0xFF, (fu.u >> 16) & 0xFF, (fu.u >> 8) & 0xFF, fu.u & 0xFF };
    ESP_LOGI(TAG, "Setpoint float value: %.2f, encoded as: %02x %02x %02x %02x", 
             f, temp_buf[0], temp_buf[1], temp_buf[2], temp_buf[3]);

    /* Params */
    uint8_t params[8 + 1 + 1 + 1 + 1 + 4];  /* Size 16 */
    int idx = 0;
    memcpy(params + idx, fictive_sensor, 8); idx += 8;
    params[idx++] = 0x00;
    params[idx++] = 0x01;  /* 0x01 pour température */
    params[idx++] = 0x00;
    params[idx++] = 0x55;
    memcpy(params + idx, temp_buf, 4); idx += 4;
    uint8_t params_len = idx;

    /* Construire val */
    uint8_t header[9];
    uint8_t header_len = construct_lumi_header(header, lumi_counter++, params_len, 0x05);
    uint8_t val[header_len + params_len];
    memcpy(val, header, header_len);
    memcpy(val + header_len, params, params_len);

    /* Log du buffer envoyé */
    ESP_LOG_BUFFER_HEX(TAG, val, sizeof(val));

    /* Attribut octet string */
    uint8_t attr_buf[1 + sizeof(val)];
    attr_buf[0] = sizeof(val);
    memcpy(attr_buf + 1, val, attr_buf[0]);

    esp_zb_zcl_attribute_t attr = {
        .id = 0xFFF2,
        .data.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
        .data.value = attr_buf
    };

    esp_zb_zcl_write_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = THERMOSTAT,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = 0xFCC0,
        .manuf_specific = 1,
        .manuf_code = MANUFACTURER_CODE,
        .attr_number = 1,
        .attr_field = &attr,
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_write_attr_cmd_req(&cmd);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "External temperature (setpoint) défini sur %d.%d °C, TSN unknown", setpoint / 100, abs(setpoint % 100));
}

/* Fonction pour définir external_humidity (% batterie, à implémenter plus tard) */
static void set_external_humidity(uint8_t battery_percent) {
    uint8_t fictive_sensor[8] = {0x00, 0x15, 0x8D, 0x00, 0x01, 0x9D, 0x1B, 0x98};

    float f = (float)battery_percent;
    union { float fl; uint32_t u; } fu;
    fu.fl = f;
    uint8_t humi_buf[4] = { (fu.u >> 24) & 0xFF, (fu.u >> 16) & 0xFF, (fu.u >> 8) & 0xFF, fu.u & 0xFF };
    ESP_LOGI(TAG, "Setpoint batterie value: %.2f, encoded as: %02x %02x %02x %02x", 
             f, humi_buf[0], humi_buf[1], humi_buf[2], humi_buf[3]);

    /* Params */
    uint8_t params[8 + 1 + 1 + 1 + 1 + 4];
    int idx = 0;
    memcpy(params + idx, fictive_sensor, 8); idx += 8;
    params[idx++] = 0x00;
    params[idx++] = 0x02;  /* 0x02 pour humidity */
    params[idx++] = 0x00;
    params[idx++] = 0x55;
    memcpy(params + idx, humi_buf, 4); idx += 4;
    uint8_t params_len = idx;

    /* Construire val */
    uint8_t header[9];
    uint8_t header_len = construct_lumi_header(header, lumi_counter++, params_len, 0x05);
    uint8_t val[header_len + params_len];
    memcpy(val, header, header_len);
    memcpy(val + header_len, params, params_len);

    /* Attribut octet string */
    uint8_t attr_buf[1 + sizeof(val)];
    attr_buf[0] = sizeof(val);
    memcpy(attr_buf + 1, val, attr_buf[0]);

    esp_zb_zcl_attribute_t attr = {
        .id = 0xFFF2,
        .data.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
        .data.value = attr_buf
    };

    esp_zb_zcl_write_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = THERMOSTAT,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = 0xFCC0,
        .manuf_specific = 1,
        .manuf_code = MANUFACTURER_CODE,
        .attr_number = 1,
        .attr_field = &attr,
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_write_attr_cmd_req(&cmd);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "External temperature (setpoint) défini sur %d.%d °C, TSN unknown", battery_percent / 100, abs(battery_percent % 100));
}

void watchdog_task(void *pvParameters)
{
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = (1 << 0),
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL);
    int watchdog_counter = 0;
    uint64_t total_seconds = 0;
    while (1) {
        watchdog_counter++;
        total_seconds++;
        if (watchdog_counter >= 60) {
            int jours = total_seconds / 86400;
            int heures = (total_seconds % 86400) / 3600;
            int minutes = (total_seconds % 3600) / 60;
            watchdog_counter = 0;
            //if (operating_time) free(operating_time);
            operating_time = malloc(32);
            if (operating_time) {
                snprintf(operating_time, 32, "%dd %02dh %02dm", jours, heures, minutes);
            }
            ESP_LOGW("WATCHDOG", "Je nourris le chien depuis %dj %02dh %02dm", jours, heures, minutes);
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    load_settings_from_nvs(); // Charger les paramètres au démarrage
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    wifi_init();
    xTaskCreate(watchdog_task, "watchdog_task", 2048, NULL, 1, NULL);
}