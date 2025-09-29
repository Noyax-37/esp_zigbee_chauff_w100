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
#include "driver/ledc.h"

static const char *TAG = "ESP_ZIGBEE_CHAUFFAGE";

// Variables globales
int16_t last_temperature = INT16_MIN;
int16_t last_humidity = INT16_MIN;
int16_t last_heating_setpoint = INT16_MIN; // Maintenu localement
int16_t last_cooling_setpoint = COOLING_SETPOINT_DEFAULT;
int16_t input_setpoint = INT16_MIN;
uint16_t input_high_hyst = HIGH_HYST_DEFAULT;
uint16_t input_low_hyst = LOW_HYST_DEFAULT;
static uint8_t relay_actual_state = 0xFF; // 0xFF = inconnu, 0 = OFF, 1 = ON
static char *update_status = NULL;
static char *status = NULL;
static char *operating_time = NULL; // Format: "Xd Yh Zm"
static uint8_t last_command_sent = 0xFF;
static uint8_t tsn_counter = 0;
static int s_retry_num = 0;
static bool wifi_failed = false;
static bool first_boot = false;
static TaskHandle_t zb_task_handle = NULL;
static bool first_request = true;
static bool zigbee_network_initialized = false;
static bool update_status_allocated = false;
static bool response_received = false;
static uint8_t last_tsn = 0;
static char ieee_addr_w100[20] = {0}; // Buffer pour "0x" + 16 caractères + \0
static char ieee_addr_relay[20] = {0}; // Buffer pour "0x" + 16 caractères + \0
static char short_addr_w100[10] = {0}; // Buffer pour "0x" + 4 caractères + \0
static char short_addr_relay[10] = {0}; // Buffer pour "0x" + 4 caractères + \0
static char mode_rout_coord[20] = {0}; // Buffer pour "router" ou "coordinator"
static uint8_t ieee_addr_w100_bytes[8] = {0};
static uint8_t ieee_addr_relay_bytes[8] = {0};
static uint16_t short_addr_w100_value = 0;
static uint16_t short_addr_relay_value = 0;

/* Compteur global pour les headers Lumi */
static uint8_t lumi_counter = 0x10;

// Prototypes de fonctions
static void esp_zb_task(void *pvParameters);
static httpd_handle_t start_webserver(void);
static void read_thermostat_attributes(void);
static void read_thermostat_attributes_pmtsd(void);
static void read_relay_state(void);
static void write_thermostat_attributes(int16_t new_setpoint, uint16_t new_high_hyst, uint16_t new_low_hyst,
                                       bool setpoint_updated, bool hysteresis_high_updated, bool hysteresis_low_updated);
static void set_sensor_mode(const char *mode_ext_int);
static void set_external_temperature(int16_t setpoint);
static void set_external_humidity(uint16_t humidity_percent);
static void save_settings_to_nvs(void);
static esp_err_t load_settings_from_nvs(void);
static void send_on_off_command(uint8_t command_id);
static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message);
static esp_err_t post_handler(httpd_req_t *req);
static esp_err_t data_handler(httpd_req_t *req);
static uint8_t construct_lumi_header(uint8_t *buffer, uint8_t counter, uint8_t cmd_len, uint8_t cmd_id);
static void update_server_attributes(void);
void update_attributes_task(void *pvParameters);
static void test_setpoint(void);
static void send_hvac_on_command(void);
static void send_hvac_off_command(void);
static void send_pmtsd_command(uint8_t power, uint8_t mode, float temp, uint8_t speed, uint8_t display);


// ////////////////////////////// zone de test ///////////////////////////////



// //////////////////////////////////// fin tests /////////////////////////////////////


esp_err_t convert_short_address(const char *addr_str, uint16_t *out_value) {
    if (addr_str == NULL || strlen(addr_str) != 6 || strncmp(addr_str, "0x", 2) != 0) {
        ESP_LOGE(TAG, "Invalid short address format: %s", addr_str ? addr_str : "NULL");
        return ESP_FAIL;
    }

    const char *hex_str = addr_str + 2; // Passer après "0x"
    char hex_buf[5] = {0}; // Buffer pour les 4 caractères + \0
    strncpy(hex_buf, hex_str, 4);

    // Convertir en majuscules pour uniformité
    for (int i = 0; hex_buf[i]; i++) {
        hex_buf[i] = toupper(hex_buf[i]);
    }

    *out_value = (uint16_t)strtol(hex_buf, NULL, 16);
    if (*out_value == 0 && (hex_buf[0] != '0' || hex_buf[1] != '0' || hex_buf[2] != '0' || hex_buf[3] != '0')) {
        ESP_LOGE(TAG, "Conversion failed for short address: %s", addr_str);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Converted short address: 0x%04x", *out_value);
    return ESP_OK;
}

esp_err_t convert_ieee_address(const char *addr_str, uint8_t out_array[8]) {
    if (addr_str == NULL || strlen(addr_str) != 18 || strncmp(addr_str, "0x", 2) != 0) {
        ESP_LOGE(TAG, "Invalid IEEE address format: %s", addr_str ? addr_str : "NULL");
        return ESP_FAIL;
    }

    const char *hex_str = addr_str + 2;
    for (int i = 0; i < 8; i++) {
        char byte_str[3] = {hex_str[i * 2], hex_str[i * 2 + 1], '\0'};
        char byte_str_upper[3];
        strcpy(byte_str_upper, byte_str);
        for (int j = 0; byte_str_upper[j]; j++) {
            byte_str_upper[j] = toupper(byte_str_upper[j]);
        }
        out_array[i] = (uint8_t)strtol(byte_str_upper, NULL, 16);
        if (out_array[i] == 0 && (byte_str[0] != '0' || byte_str[1] != '0')) {
            ESP_LOGE(TAG, "Conversion failed for byte %d: %s", i, byte_str);
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "Converted IEEE address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
             out_array[0], out_array[1], out_array[2], out_array[3],
             out_array[4], out_array[5], out_array[6], out_array[7]);
    return ESP_OK;
}

static void save_settings_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i16(nvs_handle, "setpoint", last_heating_setpoint);
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
             last_heating_setpoint, input_high_hyst, input_low_hyst);
}

static esp_err_t load_settings_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS: %s, using default settings", esp_err_to_name(err));
        last_heating_setpoint = HEATING_SETPOINT_DEFAULT;
        input_high_hyst = HIGH_HYST_DEFAULT;
        input_low_hyst = LOW_HYST_DEFAULT;
        short_addr_w100_value = 0;
        short_addr_relay_value = 0;
        memset(ieee_addr_w100_bytes, 0, sizeof(ieee_addr_w100_bytes));
        memset(ieee_addr_relay_bytes, 0, sizeof(ieee_addr_relay_bytes));
        strcpy(mode_rout_coord, "router"); // Default mode
        return ESP_FAIL;
    }

    err = nvs_get_i16(nvs_handle, "setpoint", &last_heating_setpoint);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read setpoint: %s, using default (%u)", esp_err_to_name(err), HEATING_SETPOINT_DEFAULT);
        last_heating_setpoint = HEATING_SETPOINT_DEFAULT;
    }

    err = nvs_get_u16(nvs_handle, "high_hyst", &input_high_hyst);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read high_hyst: %s, using default (%u)", esp_err_to_name(err), HIGH_HYST_DEFAULT);
        input_high_hyst = HIGH_HYST_DEFAULT;
    }

    err = nvs_get_u16(nvs_handle, "low_hyst", &input_low_hyst);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read low_hyst: %s, using default (%u)", esp_err_to_name(err), LOW_HYST_DEFAULT);
        input_low_hyst = LOW_HYST_DEFAULT;
    }

    size_t len = sizeof(ieee_addr_w100);
    err = nvs_get_str(nvs_handle, "ieee_addr_w100", ieee_addr_w100, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read ieee_addr_w100: %s, using default", esp_err_to_name(err));
        memset(ieee_addr_w100_bytes, 0, sizeof(ieee_addr_w100_bytes));
    } else {
        ESP_LOGI(TAG, "Loaded ieee_addr_w100 from NVS: %s (len=%zu)", ieee_addr_w100, len);
        if (convert_ieee_address(ieee_addr_w100, ieee_addr_w100_bytes) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to convert ieee_addr_w100");
            memset(ieee_addr_w100_bytes, 0, sizeof(ieee_addr_w100_bytes));
        }
    }

    len = sizeof(ieee_addr_relay);
    err = nvs_get_str(nvs_handle, "ieee_addr_relay", ieee_addr_relay, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read ieee_addr_relay: %s, using default", esp_err_to_name(err));
        memset(ieee_addr_relay_bytes, 0, sizeof(ieee_addr_relay_bytes));
    } else {
        ESP_LOGI(TAG, "Loaded ieee_addr_relay from NVS: %s (len=%zu)", ieee_addr_relay, len);
        if (convert_ieee_address(ieee_addr_relay, ieee_addr_relay_bytes) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to convert ieee_addr_relay");
            memset(ieee_addr_relay_bytes, 0, sizeof(ieee_addr_relay_bytes));
        }
    }

    len = sizeof(short_addr_w100);
    err = nvs_get_str(nvs_handle, "short_ad_w100", short_addr_w100, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read short_ad_w100: %s, using default", esp_err_to_name(err));
        short_addr_w100_value = 0;
    } else {
        ESP_LOGI(TAG, "Loaded short_ad_w100 from NVS: %s (len=%zu)", short_addr_w100, len);
        if (convert_short_address(short_addr_w100, &short_addr_w100_value) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to convert short_addr_w100");
            short_addr_w100_value = 0;
        }
    }

    len = sizeof(short_addr_relay);
    err = nvs_get_str(nvs_handle, "short_ad_relay", short_addr_relay, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read short_ad_relay: %s, using default", esp_err_to_name(err));
        short_addr_relay_value = 0;
    } else {
        ESP_LOGI(TAG, "Loaded short_ad_relay from NVS: %s (len=%zu)", short_addr_relay, len);
        if (convert_short_address(short_addr_relay, &short_addr_relay_value) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to convert short_addr_relay");
            short_addr_relay_value = 0;
        }
    }

    len = sizeof(mode_rout_coord);
    err = nvs_get_str(nvs_handle, "mode", mode_rout_coord, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read mode: %s, using default 'router'", esp_err_to_name(err));
        strcpy(mode_rout_coord, "router");
    }

    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "Settings loaded from NVS: setpoint=%d, high_hyst=%u, low_hyst=%u, "
             "mode=%s, ieee_addr_w100=%s, ieee_addr_relay=%s, short_addr_w100=%s, short_addr_relay=%s",
             last_heating_setpoint, input_high_hyst, input_low_hyst,
             mode_rout_coord, ieee_addr_w100, ieee_addr_relay, short_addr_w100, short_addr_relay);

    return ESP_OK;
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
    ESP_LOGI(TAG, "Zigbee signal received: %s, Status: %s (0x%x)", esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status), err_status);

    /* Gestion des signaux BDB */
    switch (sig_type) {
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    case ESP_ZB_BDB_SIGNAL_STEERING:
        ESP_LOGI(TAG, "Signal: %s, Status: %s (0x%x)", esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status), err_status);
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Joined Zigbee network successfully (PAN ID: 0x%04hx, Channel: %d, Short Address: 0x%04hx)",
                    esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            // set_sensor_mode("external");  /* Activer le mode external au démarrage */
            if (first_boot) {
                ESP_LOGI(TAG, "First boot detected, saving default settings to NVS");
                save_settings_to_nvs();
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp_restart();
            }
            read_thermostat_attributes();
            if (last_heating_setpoint != INT16_MIN) {
                ESP_LOGI(TAG, "Applying setpoint from NVS: %.1f °C", last_heating_setpoint / 100.0);
                set_external_temperature(last_heating_setpoint);
            }
            // Mettre le relais à OFF après l'initialisation Zigbee
            send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID);
            read_relay_state();
            // Activer le mode HVAC ON sur l'Aqara W100
            // send_hvac_on_command();
            // Activer la rangée centrale avec PMTSD
            // send_pmtsd_command(0, 1, (float)last_heating_setpoint / 100.0, 0, 1);
            // Mettre la temp ext au setpoint
            set_external_temperature(last_heating_setpoint);
            // Mettre l'humidité ext à 0 (provisoire)
            set_external_humidity(relay_actual_state == 1 ? 9900 : 0); // 99% si le relais est ON, sinon 0%
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
            asprintf(&update_status, "Échec de l'écriture, périphérique indisponible");
            ESP_LOGI(TAG, "Freeing update_status at address %p due to device unavailability", update_status);
            free(update_status);
            update_status = NULL;
            update_status_allocated = false;
        }
        update_status_allocated = true;
        esp_zb_zcl_read_attr_cmd_t read_cmd = {
            .zcl_basic_cmd = {
                .dst_addr_u.addr_short = short_addr_w100_value,
                .dst_endpoint = 1,
                .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .clusterID = 0xFCC0,
            .manuf_specific = 1,
            .manuf_code = MANUFACTURER_CODE,
            .attr_number = 2,
            .attr_field = (uint16_t[]){0x0172, 0xFFF2},
        };
        esp_zb_zcl_read_attr_cmd_req(&read_cmd);
        ESP_LOGI(TAG, "Sent read request for mode attribute (0x0172)");
        break;
    case ZB_ZCL_CMD_WRITE_ATTRIB:
        ESP_LOGI(TAG, "Write attribute command received, processing...");
        esp_zb_zcl_write_attr_cmd_t *write_cmd = (esp_zb_zcl_write_attr_cmd_t *)signal_struct->p_app_signal;
        for (uint8_t i = 0; i < write_cmd->attr_number; i++) {
            if (write_cmd->attr_field[i].id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
                last_heating_setpoint = *(int16_t *)write_cmd->attr_field[i].data.value;
                ESP_LOGI(TAG, "Updated occupied_heating_setpoint to %.1f°C", last_heating_setpoint / 100.0);
                set_external_temperature(last_heating_setpoint); // Appliquer la nouvelle valeur
            }
        }
        // Préparer et envoyer la réponse par défaut
        uint8_t seq_num = tsn_counter++;
        if (seq_num == 254) seq_num = 0;
        zb_bufid_t resp_buffer = zb_buf_get_out();
        if (resp_buffer) {
            ZB_ZCL_SEND_DEFAULT_RESP_EXT(
                resp_buffer,
                write_cmd->zcl_basic_cmd.dst_addr_u.addr_short,         // _dst_addr
                ZB_APS_ADDR_MODE_16_ENDP_PRESENT,                       // _dst_addr_mode
                write_cmd->zcl_basic_cmd.dst_endpoint,                  // _dst_ep
                write_cmd->zcl_basic_cmd.src_endpoint,                  // _src_ep
                ZB_AF_HA_PROFILE_ID,                                   // _prof_id (Home Automation)
                ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,                      // _cluster_id
                seq_num,                                               // _seq_num (corrigé via contexte)
                ZB_ZCL_CMD_WRITE_ATTRIB,                               // _cmd
                ZB_ZCL_STATUS_SUCCESS,                                 // _status_code
                ZB_ZCL_FRAME_DIRECTION_TO_SRV,                         // _direction (réponse au serveur)
                0,                                                     // _is_manuf_specific
                0,                                                     // _manuf_code
                NULL                                                   // _callback
            );
        }
        break;
    default:
        /* Gestion des commandes ZCL */
        if (sig_type == ZB_ZCL_CMD_READ_ATTRIB_RESP) {
            esp_zb_zcl_command_send_status_message_t *resp = (esp_zb_zcl_command_send_status_message_t *)signal_struct->p_app_signal;
            if (resp->status == ESP_OK) {
                ESP_LOGI(TAG, "Read attribute response received, TSN: %d, Endpoint: %d", resp->tsn, resp->dst_endpoint);
                /* Pour parser les valeurs des attributs, vous devrez peut-être utiliser ZB_ZCL_GENERAL_GET_READ_ATTRIB_RESP */
                /* Exemple : zb_zcl_read_attr_res_t *attr_resp = ZB_ZCL_GENERAL_GET_READ_ATTRIB_RESP(...); */
            } else {
                ESP_LOGW(TAG, "Read attribute failed, status: %s (0x%x)", esp_err_to_name(resp->status), resp->status);
            }
        } else if (sig_type == ZB_ZCL_CMD_WRITE_ATTRIB_RESP) {
            esp_zb_zcl_command_send_status_message_t *resp = (esp_zb_zcl_command_send_status_message_t *)signal_struct->p_app_signal;
            if (resp->status == ESP_OK) {
                ESP_LOGI(TAG, "Write attribute response received, TSN: %d, Endpoint: %d", resp->tsn, resp->dst_endpoint);
            } else {
                ESP_LOGW(TAG, "Write attribute failed, status: %s (0x%x)", esp_err_to_name(resp->status), resp->status);
            }
        } else if (sig_type == ZB_ZCL_CMD_CONFIG_REPORT_RESP) { /* Utiliser ZB_ZCL_CMD_CONFIG_REPORT_RESP au lieu de ZB_ZCL_CMD_CONFIG_REPORT */
            esp_zb_zcl_command_send_status_message_t *resp = (esp_zb_zcl_command_send_status_message_t *)signal_struct->p_app_signal;
            if (resp->status == ESP_OK) {
                ESP_LOGI(TAG, "Configure report response received, TSN: %d, Endpoint: %d", resp->tsn, resp->dst_endpoint);
            } else {
                ESP_LOGW(TAG, "Configure report failed, status: %s (0x%x)", esp_err_to_name(resp->status), resp->status);
            }
        } else if (sig_type == ZB_ZCL_CMD_REPORT_ATTRIB) {
            esp_zb_zcl_report_attr_cmd_t *report = (esp_zb_zcl_report_attr_cmd_t *)signal_struct->p_app_signal;
            ESP_LOGI(TAG, "Attribute report received, ClusterID: 0x%04x, AttributeID: 0x%04x", 
                     report->clusterID, report->attributeID);
            /* Les valeurs d'attributs doivent être parsées séparément via esp_zb_zcl_attribute_t */
            /* Exemple : utiliser ZB_ZCL_GET_ATTRIBUTE_VALUE si disponible */
        } else {
            ESP_LOGI(TAG, "Unknown signal: %s (0x%x), status: %s (0x%x), payload ptr: %p, signal_struct ptr: %p", 
                     esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status), err_status, signal_struct->p_app_signal, signal_struct);
        }
        break;
    }
}

static void send_on_off_command(uint8_t command_id)
{
    if ((command_id == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID && relay_actual_state == 1) || (command_id == ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID && relay_actual_state == 0)) {
        ESP_LOGI(TAG, "Skipping %s command to Shelly relay (0x%04x, endpoint %d): already in this state",
                 (command_id == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", short_addr_relay_value, RELAY_BINDING_EP);
        return;
    }

    esp_zb_zcl_on_off_cmd_t cmd_req;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr_relay_value;
    cmd_req.zcl_basic_cmd.dst_endpoint = RELAY_BINDING_EP;
    cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd_req.on_off_cmd_id = command_id;

    ESP_LOGI(TAG, "Sending %s command to Shelly relay (0x%04x, endpoint %d)", 
             (command_id == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", short_addr_relay_value, RELAY_BINDING_EP);

    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t tsn = esp_zb_zcl_on_off_cmd_req(&cmd_req);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Sent %s command to Shelly relay (0x%04x, endpoint %d) with TSN 0x%02x", 
             (command_id == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", short_addr_relay_value, RELAY_BINDING_EP, tsn);

    last_command_sent = command_id;
}

static void read_relay_state(void)
{
    esp_zb_zcl_read_attr_cmd_t read_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr_relay_value,
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

static void read_thermostat_attributes_pmtsd(void)
{
    // Lecture du pmtsd
    esp_zb_zcl_read_attr_cmd_t cmd_pmtsd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr_w100_value,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = 0xFCC0,
        .manuf_specific = 1,
        .manuf_code = MANUFACTURER_CODE,
        .attr_number = 1,
        .attr_field = (uint16_t[]){0xFFF2},
    };
    esp_zb_zcl_read_attr_cmd_req(&cmd_pmtsd);

    ESP_LOGI(TAG, "Sent read requests for thermostat attributes");
}

static void read_thermostat_attributes(void)
{
    // Lecture de la température locale
    esp_zb_zcl_read_attr_cmd_t cmd_temp = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr_w100_value,
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
            .dst_addr_u.addr_short = short_addr_w100_value,
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

    // Lecture du mode capteur 1
    esp_zb_zcl_read_attr_cmd_t cmd_specific_1 = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr_w100_value,
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
    esp_zb_zcl_read_attr_cmd_req(&cmd_specific_1);

    // Lecture du mode capteur 2
    esp_zb_zcl_read_attr_cmd_t cmd_specific_2 = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr_w100_value,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = 0xFCC0,
        .manuf_specific = 1,
        .manuf_code = MANUFACTURER_CODE,
        .attr_number = 1,
        .attr_field = (uint16_t[]){0xFFF2},
    };
    esp_zb_zcl_read_attr_cmd_req(&cmd_specific_2);

    /*
    // Lecture du cluster thermostat
    esp_zb_zcl_read_attr_cmd_t cmd_thermostat = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr_w100_value,
            .dst_endpoint = 1,
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        .manuf_specific = 0,
        .manuf_code = MANUFACTURER_CODE,
        .attr_number = 3,
        .attr_field = (uint16_t[]){ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID},
    };
    esp_zb_zcl_read_attr_cmd_req(&cmd_thermostat);
*/    

    ESP_LOGI(TAG, "Sent read requests for thermostat attributes");
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message) 
{ 
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message"); 
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)", message->status);
    ESP_LOGI(TAG, "Received report from address(0x%04x) src endpoint(%d) to dst endpoint(%d) cluster(0x%04x)", 
             message->src_address.u.short_addr, message->src_endpoint, message->dst_endpoint, message->cluster);
    ESP_LOGI(TAG, "Report information: attribute(0x%04x), type(0x%02x), size(%d)", 
             message->attribute.id, message->attribute.data.type, message->attribute.data.size);

    // Afficher la valeur en fonction du type
    if (message->attribute.data.value) {
        switch (message->attribute.data.type) {
            case ESP_ZB_ZCL_ATTR_TYPE_U8:
            case ESP_ZB_ZCL_ATTR_TYPE_BOOL:
                ESP_LOGI(TAG, "Attribute value: %u (uint8_t)", *(uint8_t *)message->attribute.data.value);
                break;
            case ESP_ZB_ZCL_ATTR_TYPE_U16:
                ESP_LOGI(TAG, "Attribute value: %u (uint16_t)", *(uint16_t *)message->attribute.data.value);
                break;
            case ESP_ZB_ZCL_ATTR_TYPE_S16:
                ESP_LOGI(TAG, "Attribute value: %d (int16_t)", *(int16_t *)message->attribute.data.value);
                break;
            case ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING:
            case ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING:
                ESP_LOGI(TAG, "Attribute value: %s (string)", (char *)message->attribute.data.value);
                ESP_LOG_BUFFER_HEX(TAG, message->attribute.data.value, message->attribute.data.size);
                break;
            default:
                ESP_LOGW(TAG, "Unsupported attribute type: 0x%02x", message->attribute.data.type);
                ESP_LOG_BUFFER_HEX(TAG, message->attribute.data.value, message->attribute.data.size);
                break;
        }
    } else {
        ESP_LOGW(TAG, "Attribute value is NULL");
    }

    if (message->src_address.u.short_addr == short_addr_w100_value) {
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
        } else if (message->cluster == 0xFCC0) { // manuSpecificLumi cluster
            ESP_LOGI(TAG, "Processing manuSpecificLumi attribute (0x%04x)", message->attribute.id);
            
            if (message->attribute.id == 0x0009) { // Attribut 9
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                    uint8_t attr_9_value = *(uint8_t *)message->attribute.data.value;
                    ESP_LOGI(TAG, "Thermostat 0x%04x Attribute 9: %u", 
                             message->src_address.u.short_addr, attr_9_value);
                } else {
                    ESP_LOGW(TAG, "Unexpected type for attribute 9: 0x%02x", message->attribute.data.type);
                    ESP_LOG_BUFFER_HEX(TAG, message->attribute.data.value, message->attribute.data.size);
                }
            } else if (message->attribute.id == 0x0020) { // Attribut 32
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                    uint8_t attr_32_value = *(uint8_t *)message->attribute.data.value;
                    ESP_LOGI(TAG, "Thermostat 0x%04x Attribute 32: %u", 
                             message->src_address.u.short_addr, attr_32_value);
                } else {
                    ESP_LOGW(TAG, "Unexpected type for attribute 32: 0x%02x", message->attribute.data.type);
                    ESP_LOG_BUFFER_HEX(TAG, message->attribute.data.value, message->attribute.data.size);
                }
            } else if (message->attribute.id == 0x0172) {
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                    uint8_t sensor_mode = *(uint8_t *)message->attribute.data.value;
                    const char *mode_str = (sensor_mode == 2 || sensor_mode == 3) ? "external" : "internal";
                    ESP_LOGI(TAG, "Thermostat 0x%04x Sensor mode: %s (raw: %u)", 
                             message->src_address.u.short_addr, mode_str, sensor_mode);
                } else {
                    ESP_LOGW(TAG, "Unexpected type for attribute 0x0172: 0x%02x", message->attribute.data.type);
                }
            } else if (message->attribute.id == 0xFFF2) { // 65522
                ESP_LOGI(TAG, "Received PMTSD report from Aqara W100 (0x%04x, endpoint %d)", 
                         message->src_address.u.short_addr, message->src_endpoint);

                // Vérifier le type et la taille des données
                if (message->attribute.data.type != ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING) {
                    ESP_LOGE(TAG, "Unexpected attribute type for 0xFFF2: 0x%02x", message->attribute.data.type);
                    ESP_LOG_BUFFER_HEX(TAG, message->attribute.data.value, message->attribute.data.size);
                    return ESP_OK;
                }
                if (message->attribute.data.size <= 0 || message->attribute.data.size > 100) {
                    ESP_LOGE(TAG, "Invalid data size for 0xFFF2: %d", message->attribute.data.size);
                    return ESP_OK;
                }

                // Afficher le buffer brut
                ESP_LOG_BUFFER_HEX(TAG, message->attribute.data.value, message->attribute.data.size);

                // Vérifier si c'est une demande PMTSD (08:00:08:44 à la fin)
                uint8_t *data = (uint8_t *)message->attribute.data.value;
                int data_len = message->attribute.data.size;
                bool is_pmtsd_request = (data_len >= 4 && 
                                         data[data_len - 4] == 0x08 && 
                                         data[data_len - 3] == 0x00 && 
                                         data[data_len - 2] == 0x08 && 
                                         data[data_len - 1] == 0x44);

                if (is_pmtsd_request) {
                    ESP_LOGI(TAG, "Detected PMTSD request from Aqara W100, responding with PMTSD command");
                    send_hvac_on_command(); // Réponse actuelle
                } else {
                    // Chercher la séquence 08:00:08:44 pour trouver le début de la partie ASCII
                    int ascii_start = -1;
                    for (int i = 0; i <= data_len - 4; i++) {
                        if (data[i] == 0x08 && data[i + 1] == 0x00 && 
                            data[i + 2] == 0x08 && data[i + 3] == 0x44) {
                            ascii_start = i + 4;
                            break;
                        }
                    }

                    if (ascii_start == -1 || ascii_start >= data_len) {
                        ESP_LOGE(TAG, "No valid PMTSD ASCII part found in buffer");
                        ESP_LOG_BUFFER_HEX(TAG, message->attribute.data.value, message->attribute.data.size);
                        return ESP_OK;
                    }

                    // Extraire la partie ASCII
                    char ascii_part[100] = {0};
                    int ascii_len = 0;
                    for (int i = ascii_start; i < data_len && data[i] != 0x00; i++) {
                        ascii_part[ascii_len++] = data[i];
                    }
                    ascii_part[ascii_len] = '\0';
                    ESP_LOGI(TAG, "PMTSD ASCII part: %s (length: %d)", ascii_part, ascii_len);

                    // Parser la chaîne ASCII avec strtok pour gérer les données partielles
                    char p_value[4] = {0}, m_value[4] = {0}, t_value[8] = {0}, s_value[4] = {0}, d_value[4] = {0};
                    char *token = strtok(ascii_part, "_");
                    while (token) {
                        if (token[0] == 'P') strncpy(p_value, token + 1, sizeof(p_value) - 1);
                        else if (token[0] == 'M') strncpy(m_value, token + 1, sizeof(m_value) - 1);
                        else if (token[0] == 'T') strncpy(t_value, token + 1, sizeof(t_value) - 1);
                        else if (token[0] == 'S') strncpy(s_value, token + 1, sizeof(s_value) - 1);
                        else if (token[0] == 'D') strncpy(d_value, token + 1, sizeof(d_value) - 1);
                        token = strtok(NULL, "_");
                    }
                    ESP_LOGI(TAG, "Decoded PMTSD: P=%s, M=%s, T=%s, S=%s, D=%s", 
                             p_value, m_value, t_value, s_value, d_value);

                    // Mettre à jour l'attribut manuSpecificLumi sur l'ESP32
                    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
                        HA_ONOFF_SWITCH_ENDPOINT, 
                        0xFCC0, 
                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
                        0xFFF2, 
                        (uint8_t *)ascii_part, 
                        true // Check attribute validity
                    );
                    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
                        ESP_LOGE(TAG, "Failed to set PMTSD attribute: %d", status);
                    }

                    // Rapporter l'attribut à Z2M
                    esp_zb_zcl_report_attr_cmd_t report_cmd = {
                        .zcl_basic_cmd = {
                            .dst_addr_u.addr_short = 0x0000, // Coordinator
                            .dst_endpoint = 1,
                            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT
                        },
                        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                        .clusterID = 0xFCC0,
                        .manuf_specific = 1,
                        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                        .dis_default_resp = 1,
                        .manuf_code = 4447, // Aqara manufacturer code
                        .attributeID = 0xFFF2
                    };
                    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "PMTSD attribute reported to Z2M");
                    } else {
                        ESP_LOGE(TAG, "Failed to report PMTSD attribute: %d", err);
                    }
                }
            } else {
                ESP_LOGW(TAG, "Unknown manuSpecificLumi attribute ID: 0x%04x", message->attribute.id);
                ESP_LOG_BUFFER_HEX(TAG, message->attribute.data.value, message->attribute.data.size);
            }
        } else if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_MULTI_INPUT) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID) {
                uint16_t button_value = *(uint16_t *)message->attribute.data.value;
                if (button_value == 1) {
                    if (message->src_endpoint == 1) {
                        ESP_LOGI(TAG, "Button + pressed (single_plus, endpoint 1)");
                        if (last_heating_setpoint != INT16_MIN) {
                            int16_t new_setpoint = last_heating_setpoint + 10;
                            set_external_temperature(new_setpoint);
                            send_pmtsd_command(0, 1, (float)new_setpoint / 100.0, 0, 1);
                            last_heating_setpoint = new_setpoint;
                            save_settings_to_nvs();
                            ESP_LOGI(TAG, "Setpoint increased to %d.%d °C", new_setpoint / 100, abs(new_setpoint % 100));
                        }
                    } else if (message->src_endpoint == 2) {
                        ESP_LOGI(TAG, "Button center pressed (single_center, endpoint 2)");
                        send_on_off_command(relay_actual_state == 1 ? ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID : ESP_ZB_ZCL_CMD_ON_OFF_ON_ID);
                        read_relay_state();
                    } else if (message->src_endpoint == 3) {
                        ESP_LOGI(TAG, "Button - pressed (single_minus, endpoint 3)");
                        if (last_heating_setpoint != INT16_MIN) {
                            int16_t new_setpoint = last_heating_setpoint - 10;
                            set_external_temperature(new_setpoint);
                            send_pmtsd_command(0, 1, (float)new_setpoint / 100.0, 0, 1);
                            last_heating_setpoint = new_setpoint;
                            save_settings_to_nvs();
                            ESP_LOGI(TAG, "Setpoint decreased to %d.%d °C", new_setpoint / 100, abs(new_setpoint % 100));
                        }
                    }
                }
            }
        } else if (message->cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
                last_heating_setpoint = *(int16_t *)message->attribute.data.value;
                ESP_LOGI(TAG, "Thermostat 0x%04x Setpoint: %d.%d °C", 
                         message->src_address.u.short_addr, 
                         last_heating_setpoint / 100, abs(last_heating_setpoint % 100));
            } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID) {
                last_temperature = *(int16_t *)message->attribute.data.value;
                ESP_LOGI(TAG, "Thermostat 0x%04x Local Temperature: %d.%d °C", 
                         message->src_address.u.short_addr, 
                         last_temperature / 100, abs(last_temperature % 100));
            } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID) {
                last_cooling_setpoint = *(int16_t *)message->attribute.data.value;
                ESP_LOGI(TAG, "Thermostat 0x%04x Cooling Setpoint: %d.%d °C", 
                         message->src_address.u.short_addr, 
                         last_cooling_setpoint / 100, abs(last_cooling_setpoint % 100));
            }
        }

        test_setpoint();
        if (zigbee_network_initialized) {
            update_server_attributes();
        }
    }

    if (message->src_address.u.short_addr == short_addr_relay_value && message->cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
            uint8_t relay_historical_state = relay_actual_state;
            relay_actual_state = (*(uint8_t *)message->attribute.data.value != 0) ? 1 : 0;
            ESP_LOGI(TAG, "Relay 0x%04x On/Off state: %s", 
                     message->src_address.u.short_addr, 
                     (relay_actual_state == 1) ? "ON" : "OFF");
            if (relay_historical_state != relay_actual_state) {
                ESP_LOGI(TAG, "Relay state changed from %s to %s",
                         (relay_historical_state == 1) ? "ON" : "OFF",
                         (relay_actual_state == 1) ? "ON" : "OFF");
                set_external_humidity(relay_actual_state == 1 ? 9900 : 0); // 99% si le relais est ON, sinon 0%
            }
            if (zigbee_network_initialized) {
                update_server_attributes();
            }
        }
    }

    return ESP_OK;
}

static void test_setpoint(void)
{
        // Logique du relais avec last_heating_setpoint, input_high_hyst et input_low_hyst
        if (last_temperature != INT16_MIN && last_heating_setpoint != INT16_MIN && 
            input_high_hyst != 0 && input_low_hyst != 0) {
            if (last_temperature <= last_heating_setpoint - (int16_t)input_low_hyst) {
                send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_ON_ID);
                read_relay_state();
            } else if (last_temperature >= last_heating_setpoint + (int16_t)input_high_hyst) {
                send_on_off_command(ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID);
                read_relay_state();
            }
        } else {
            ESP_LOGI(TAG, "Waiting for all data: temp=%d, setpoint=%d, high_hyst=%u, low_hyst=%u",
                     last_temperature, last_heating_setpoint, input_high_hyst, input_low_hyst);
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
        send_pmtsd_command(0, 1, (float)(new_setpoint) / 100.0, 0, 1);
        last_heating_setpoint = new_setpoint; // Mettre à jour localement
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
        if (resp->info.src_address.u.short_addr == short_addr_w100_value && resp->info.cluster == 0xFCC0) {
            if (resp->status_code == 0) {
                ESP_LOGI(TAG, "Write command to thermostat (0x%04x) succeeded", short_addr_w100_value);
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
                ESP_LOGE(TAG, "Write failed for thermostat (0x%04x), status: 0x%02x", short_addr_w100_value, resp->status_code);
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
        } else if (resp->info.src_address.u.short_addr == short_addr_relay_value && resp->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            ESP_LOGI(TAG, "Command %s (0x%02x) to Relay (0x%04x) %s",
                    (resp->resp_to_cmd == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? "ON" : "OFF", resp->resp_to_cmd,
                    short_addr_relay_value, (resp->status_code == 0) ? "succeeded" : "failed");
            // set_external_humidity((resp->resp_to_cmd == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID) ? 9900 : 0); // 99% si le relais est ON, sinon 0%
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
                if (resp->info.cluster == 0xFCC0){
                    ESP_LOGI(TAG, "0xFFC0 Attribute ID: 0x%04x, Type: 0x%02x, Status: 0x%02x, Value Size: %d, Value: %p", 
                        variable->attribute.id, variable->attribute.data.type, variable->status, variable->attribute.data.size, variable->attribute.data.value);
                    if (variable->attribute.id == 0x0172) {
                        uint8_t mode_ext_int = *(uint8_t *)variable->attribute.data.value;
                        const char *mode_str = (mode_ext_int == 0x02 || mode_ext_int == 0x03) ? "external" : "internal";
                        ESP_LOGI(TAG, "Sensor mode: %s (raw: %u)", mode_str, mode_ext_int);
                        if (mode_ext_int != 0x02 && mode_ext_int != 0x03) {
                            ESP_LOGW(TAG, "Thermostat not in external sensor mode, setting to external");
                            set_sensor_mode("external");
                        }
                    }
                    if (variable->attribute.id == 0xFFF2) {
                        uint8_t *data = (uint8_t *)variable->attribute.data.value;
                        uint16_t data_len = variable->attribute.data.size;
                        ESP_LOGI(TAG, "Reception sur attribut 0xFFF2, length: %d", data_len);
                        ESP_LOG_BUFFER_HEX(TAG, data, data_len);
                        // Vérifier si c'est une requête PMTSD
                        if (data_len >= 4 && data[data_len - 4] == 0x08 && data[data_len - 3] == 0x00 &&
                            data[data_len - 2] == 0x08 && data[data_len - 1] == 0x44) {
                            ESP_LOGI(TAG, "Detected PMTSD request, sending response");
                            send_pmtsd_command(0, 1, (float) last_heating_setpoint / 100.0, 0, 1);
                        } else {
                            // Tenter de décoder comme ASCII (PMTSD)
                            char pmtsd_str[32] = {0};
                            if (data_len < sizeof(pmtsd_str) && data[0] >= 'P' && data[0] <= 'Z') {
                                memcpy(pmtsd_str, data, data_len);
                                ESP_LOGI(TAG, "PMTSD data: %s", pmtsd_str);
                                char *token = strtok(pmtsd_str, "_");
                                while (token != NULL) {
                                    if (token[0] == 'P') ESP_LOGI(TAG, "Power: %s", token + 1);
                                    else if (token[0] == 'M') ESP_LOGI(TAG, "Mode: %s", token + 1);
                                    else if (token[0] == 'T') ESP_LOGI(TAG, "Temperature: %s", token + 1);
                                    else if (token[0] == 'S') ESP_LOGI(TAG, "Speed: %s", token + 1);
                                    else if (token[0] == 'D') ESP_LOGI(TAG, "D: %s", token + 1);
                                    token = strtok(NULL, "_");
                                }
                            } else {
                                // Rapport non-ASCII (par exemple, état HVAC)
                                ESP_LOGI(TAG, "Non-ASCII PMTSD report, length: %d", data_len);
                                // Logique inspirée de DecodePMTSD_FD (à implémenter selon le convertisseur)
                                if (data_len == 9 && data[0] == 0x02 && data[1] == 0x41 && data[2] == 0x2f) {
                                    // Exemple hypothétique basé sur la structure du rapport
                                    ESP_LOGI(TAG, "Possible HVAC state report");
                                    // Ajouter une logique pour extraire des informations spécifiques
                                }
                            }
                        }
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

static void wifi_set_static_ip(esp_netif_t *netif)
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
        wifi_set_static_ip(netif);
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
    if (file_size > 40000) {
        ESP_LOGE(TAG, "Index.html too large (%" PRId32 " bytes), max is 40000 bytes", (int32_t)file_size);
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *response = (char *)calloc(40000, 1);
    if (response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for response");
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    char *updated_response = (char *)calloc(40000, 1);
    if (updated_response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for updated_response");
        free(response);
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t len = fread(response, 1, 40000, f);
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
    snprintf(setpoint_str, sizeof(setpoint_str), "%.1f", last_heating_setpoint != INT16_MIN ? last_heating_setpoint / 100.0 : 0.0);
    snprintf(relay_str, sizeof(relay_str), "%s", last_command_sent == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID ? "ON" : "OFF");

    int res_len = snprintf(updated_response, 40000,
                           response,
                           temp_str, humi_str, setpoint_str, relay_str, running_state_str, setpoint_str);
    if (res_len >= 40000) {
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

static esp_err_t parametres_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Received request for /parametres");

    FILE *f = fopen("/spiffs/parametres.html", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open parametres.html");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    rewind(f);
    ESP_LOGI(TAG, "parametres.html size: %ld bytes", file_size);
    if (file_size > 30000) {
        ESP_LOGE(TAG, "parametres.html too large (%" PRId32 " bytes), max is 30000 bytes", (int32_t)file_size);
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *response = (char *)calloc(30000, 1);
    if (response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for response");
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t len = fread(response, 1, 30000, f);
    fclose(f);
    ESP_LOGI(TAG, "Read %u bytes from parametres.html", len);

    if (len <= 0) {
        ESP_LOGE(TAG, "Failed to read parametres.html");
        free(response);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    response[len] = '\0';

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, strlen(response));
    free(response);
    return ESP_OK;
}

static esp_err_t save_config_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Received request for /saveConfig");

    char buf[200];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive data");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON *mode_rout_coord = cJSON_GetObjectItem(json, "mode");
    cJSON *ieeeAddrW100 = cJSON_GetObjectItem(json, "ieeeAddrW100");
    cJSON *ieeeAddrRelay = cJSON_GetObjectItem(json, "ieeeAddrRelay");
    cJSON *shortAddrW100 = cJSON_GetObjectItem(json, "shortAddrW100");
    cJSON *shortAddrRelay = cJSON_GetObjectItem(json, "shortAddrRelay");

    if (mode_rout_coord && ieeeAddrW100 && ieeeAddrRelay) {
        nvs_handle_t nvs_handle;
        esp_err_t err;
        if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
            // Lire le mode actuel pour comparaison
            char current_mode[20] = {0};
            size_t len = sizeof(current_mode);
            err = nvs_get_str(nvs_handle, "mode", current_mode, &len);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Current mode not found, assuming no change");
                strcpy(current_mode, ""); // Par défaut si non trouvé
            }

            // Sauvegarder les nouvelles valeurs
            err = nvs_set_str(nvs_handle, "mode", mode_rout_coord->valuestring);
            if (err != ESP_OK) ESP_LOGE(TAG, "Failed to set mode in NVS: %s", esp_err_to_name(err));
            else ESP_LOGI(TAG, "Mode set: %s", mode_rout_coord->valuestring);

            err = nvs_set_str(nvs_handle, "ieee_addr_w100", ieeeAddrW100->valuestring);
            if (err != ESP_OK) ESP_LOGE(TAG, "Failed to set ieee_addr_w100 in NVS: %s", esp_err_to_name(err));
            else ESP_LOGI(TAG, "IEEE W100 address set: %s", ieeeAddrW100->valuestring);

            err = nvs_set_str(nvs_handle, "ieee_addr_relay", ieeeAddrRelay->valuestring);
            if (err != ESP_OK) ESP_LOGE(TAG, "Failed to set ieee_addr_relay in NVS: %s", esp_err_to_name(err));
            else ESP_LOGI(TAG, "IEEE Relay address set: %s", ieeeAddrRelay->valuestring);

            if (cJSON_IsString(shortAddrW100) && cJSON_IsString(shortAddrRelay)) {
                err = nvs_set_str(nvs_handle, "short_ad_w100", shortAddrW100->valuestring);
                if (err != ESP_OK) ESP_LOGE(TAG, "Failed to set short_ad_w100 in NVS: %s", esp_err_to_name(err));
                else ESP_LOGI(TAG, "Short W100 address set: %s", shortAddrW100->valuestring);
                err = nvs_set_str(nvs_handle, "short_ad_relay", shortAddrRelay->valuestring);
                if (err != ESP_OK) ESP_LOGE(TAG, "Failed to set short_ad_relay in NVS: %s", esp_err_to_name(err));
                else ESP_LOGI(TAG, "Short Relay address set: %s", shortAddrRelay->valuestring);
            }

            err = nvs_commit(nvs_handle);
            if (err != ESP_OK) ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
            else ESP_LOGI(TAG, "NVS commit successful");

            nvs_close(nvs_handle);

            ESP_LOGI(TAG, "Config saved: Mode=%s, IEEE W100=%s, IEEE Relay=%s, Short W100=%s, Short Relay=%s",
                     mode_rout_coord->valuestring, ieeeAddrW100->valuestring, ieeeAddrRelay->valuestring,
                     shortAddrW100 ? shortAddrW100->valuestring : "N/A",
                     shortAddrRelay ? shortAddrRelay->valuestring : "N/A");

            httpd_resp_sendstr(req, "{\"status\":\"success\"}");

            // Redémarrer uniquement si le mode a changé
            if (strcmp(current_mode, mode_rout_coord->valuestring) != 0) {
                ESP_LOGI(TAG, "Mode changed from %s to %s, restarting...", current_mode, mode_rout_coord->valuestring);
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp_restart();
            } else {
                ESP_LOGI(TAG, "No mode change, no restart needed");
            }
        } else {
            ESP_LOGE(TAG, "Failed to open NVS");
            httpd_resp_sendstr(req, "{\"status\":\"error\"}");
        }
    } else {
        ESP_LOGE(TAG, "Missing JSON fields");
        httpd_resp_sendstr(req, "{\"status\":\"error\"}");
    }

    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t get_config_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Received request for /getConfig");

    nvs_handle_t nvs_handle;
    char mode_rout_coord[20] = {0};
    char ieee_addr_w100[20] = {0};
    char ieee_addr_relay[20] = {0};
    char short_addr_w100[10] = {0};
    char short_addr_relay[10] = {0};
    size_t len;

    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS, error: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    len = sizeof(mode_rout_coord);
    err = nvs_get_str(nvs_handle, "mode", mode_rout_coord, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Mode not found in NVS, using default 'router'");
        strcpy(mode_rout_coord, "router");
    } else {
        ESP_LOGI(TAG, "Loaded mode from NVS: %s (len=%zu)", mode_rout_coord, len);
    }

    len = sizeof(ieee_addr_w100);
    err = nvs_get_str(nvs_handle, "ieee_addr_w100", ieee_addr_w100, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ieee_addr_w100 not found in NVS");
        strcpy(ieee_addr_w100, "");
    } else {
        ESP_LOGI(TAG, "Loaded ieee_addr_w100 from NVS: %s (len=%zu)", ieee_addr_w100, len);
    }

    len = sizeof(ieee_addr_relay);
    err = nvs_get_str(nvs_handle, "ieee_addr_relay", ieee_addr_relay, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ieee_addr_relay not found in NVS");
        strcpy(ieee_addr_relay, "");
    } else {
        ESP_LOGI(TAG, "Loaded ieee_addr_relay from NVS: %s (len=%zu)", ieee_addr_relay, len);
    }

    len = sizeof(short_addr_w100);
    err = nvs_get_str(nvs_handle, "short_ad_w100", short_addr_w100, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "short_ad_w100 not found in NVS");
        strcpy(short_addr_w100, "");
    } else {
        ESP_LOGI(TAG, "Loaded short_ad_w100 from NVS: %s (len=%zu)", short_addr_w100, len);
    }

    len = sizeof(short_addr_relay);
    err = nvs_get_str(nvs_handle, "short_ad_relay", short_addr_relay, &len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "short_ad_relay not found in NVS");
        strcpy(short_addr_relay, "");
    } else {
        ESP_LOGI(TAG, "Loaded short_ad_relay from NVS: %s (len=%zu)", short_addr_relay, len);
    }

    nvs_close(nvs_handle);

    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON_AddStringToObject(json, "mode", mode_rout_coord);
    cJSON_AddStringToObject(json, "ieeeAddrW100", ieee_addr_w100);
    cJSON_AddStringToObject(json, "ieeeAddrRelay", ieee_addr_relay);
    cJSON_AddStringToObject(json, "shortAddrW100", short_addr_w100);
    cJSON_AddStringToObject(json, "shortAddrRelay", short_addr_relay);

    char *json_str = cJSON_Print(json);
    if (json_str == NULL) {
        ESP_LOGE(TAG, "Failed to print JSON");
        cJSON_Delete(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sending JSON response: %s", json_str);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_str);
    cJSON_free(json_str);
    cJSON_Delete(json);
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
    int16_t new_setpoint = last_heating_setpoint;
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
    static int16_t last_heating_setpoint_sent = INT16_MIN;
    static uint8_t last_relay_actual_state_sent = 0xFF;
    static uint16_t last_high_hyst_sent = 0;
    static uint16_t last_low_hyst_sent = 0;
    static char *last_update_status_sent = NULL;

    bool data_changed = false;

    if (first_request || 
        last_temperature_sent != last_temperature ||
        last_humidity_sent != last_humidity ||
        last_heating_setpoint_sent != last_heating_setpoint ||
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
    snprintf(setpoint_str, sizeof(setpoint_str), "%.1f", last_heating_setpoint != INT16_MIN ? last_heating_setpoint / 100.0 : 0.0);
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
    last_heating_setpoint_sent = last_heating_setpoint;
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

static esp_err_t hvac_on_handler(httpd_req_t *req)
{
    char response[100];
    send_hvac_on_command();
    snprintf(response, sizeof(response), "{\"status\":\"HVAC ON command sent\"}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static esp_err_t hvac_off_handler(httpd_req_t *req)
{
    char response[100];
    send_hvac_off_command();
    snprintf(response, sizeof(response), "{\"status\":\"HVAC OFF command sent\"}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static esp_err_t pmtsd_handler(httpd_req_t *req) {
    char buf[256];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"status\":\"Erreur : Corps de la requête trop volumineux\"}");
        return ESP_FAIL;
    }

    // Lire le corps de la requête
    ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf) - 1));
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "{\"status\":\"Erreur : Timeout de la requête\"}");
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0'; // Terminer la chaîne

    // Extraire les paramètres
    char p_pmtsd[16] = {0}, m_pmtsd[16] = {0}, t_pmtsd[16] = {0}, s_pmtsd[16] = {0}, d_pmtsd[16] = {0};
    httpd_query_key_value(buf, "p_pmtsd", p_pmtsd, sizeof(p_pmtsd));
    httpd_query_key_value(buf, "m_pmtsd", m_pmtsd, sizeof(m_pmtsd));
    httpd_query_key_value(buf, "t_pmtsd", t_pmtsd, sizeof(t_pmtsd));
    httpd_query_key_value(buf, "s_pmtsd", s_pmtsd, sizeof(s_pmtsd));
    httpd_query_key_value(buf, "d_pmtsd", d_pmtsd, sizeof(d_pmtsd));

    // Vérifier si toutes les valeurs sont présentes
    if (strlen(p_pmtsd) == 0 || strlen(m_pmtsd) == 0 || strlen(t_pmtsd) == 0 || 
        strlen(s_pmtsd) == 0 || strlen(d_pmtsd) == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"status\":\"Erreur : Tous les champs PMTSD doivent être remplis\"}");
        return ESP_FAIL;
    }

    // Convertir les paramètres en types appropriés
    uint8_t power = atoi(p_pmtsd);
    uint8_t mode = atoi(m_pmtsd);
    float temp = atof(t_pmtsd);
    uint8_t speed = atoi(s_pmtsd);
    uint8_t display = atoi(d_pmtsd);

    // Valider les plages des paramètres
    if (power > 1 || mode > 2 || temp < 5.0 || temp > 30.0 || speed > 4) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "{\"status\":\"Erreur : Valeurs PMTSD hors des plages valides (P:0-1, M:0-2, T:5.0-30.0, S:0-4, D:0-1)\"}");
        return ESP_FAIL;
    }

    // Appeler la fonction pour envoyer la commande PMTSD
    send_pmtsd_command(power, mode, temp, speed, display);

    // Réponse HTTP
    char resp[128];
    snprintf(resp, sizeof(resp), "{\"status\":\"Commande PMTSD envoyée : P%d_M%d_T%.1f_S%d_D%d\"}", 
             power, mode, temp, speed, display);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));

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
    config.max_uri_handlers = 12;
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
        httpd_uri_t hvac_on = {
            .uri       = "/hvac_on",
            .method    = HTTP_POST,
            .handler   = hvac_on_handler,
            .user_ctx  = NULL
        };
        httpd_uri_t hvac_off = {
            .uri       = "/hvac_off",
            .method    = HTTP_POST,
            .handler   = hvac_off_handler,
            .user_ctx  = NULL
        };
        httpd_uri_t pmtsd = {
            .uri       = "/pmtsd",
            .method    = HTTP_POST,
            .handler   = pmtsd_handler,
            .user_ctx  = NULL
        };
        httpd_uri_t parametres_uri = {
            .uri = "/parametres",
            .method = HTTP_GET,
            .handler = parametres_handler,
            .user_ctx = NULL
        };
        // Handler pour saveConfig
        httpd_uri_t save_config_uri = {
            .uri = "/saveConfig",
            .method = HTTP_POST,
            .handler = save_config_handler,
            .user_ctx = NULL
        };
        httpd_uri_t get_config_uri = {
            .uri = "/getConfig",
            .method = HTTP_GET,
            .handler = get_config_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &get_uri);
        httpd_register_uri_handler(server, &favicon_uri);
        httpd_register_uri_handler(server, &post_uri);
        httpd_register_uri_handler(server, &data_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &operating_time_uri);
        httpd_register_uri_handler(server, &hvac_on);
        httpd_register_uri_handler(server, &hvac_off);
        httpd_register_uri_handler(server, &pmtsd);
        httpd_register_uri_handler(server, &parametres_uri);
        httpd_register_uri_handler(server, &save_config_uri);
        httpd_register_uri_handler(server, &get_config_uri);
        ESP_LOGI(TAG, "Web server started on port %d", config.server_port);
    } else {
        ESP_LOGE(TAG, "Failed to start web server");
    }
    return server;
}

static void esp_zb_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Free heap size at Zigbee task start: %lu bytes", esp_get_free_heap_size());
    
    // Configurer la structure Zigbee en fonction du mode
    esp_zb_cfg_t zb_nwk_cfg;
    if (strcmp(mode_rout_coord, "coordinator") == 0) {
        ESP_LOGI(TAG, "Starting in Coordinator mode");
        zb_nwk_cfg = (esp_zb_cfg_t){
            .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
            .install_code_policy = INSTALLCODE_POLICY_ENABLE,
            .nwk_cfg.zczr_cfg = {
                .max_children = 10,
            },
        };
    } else { // Par défaut ou "router"
        ESP_LOGI(TAG, "Starting in Router mode");
        zb_nwk_cfg = (esp_zb_cfg_t){
            .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
            .install_code_policy = INSTALLCODE_POLICY_ENABLE,
            .nwk_cfg.zczr_cfg = {
                .max_children = 10,
            },
        };
    }
    
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
    esp_zb_attribute_list_t *esp_zb_manu_specific_lumi_client_cluster = esp_zb_zcl_attr_list_create(0xFCC0);
    if (esp_zb_manu_specific_lumi_client_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Manufacturer Specific client cluster list");
        return;
    }
    uint8_t mode_ext_int = 0; // Valeur par défaut
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_client_cluster, 0x0009, ESP_ZB_ZCL_ATTR_TYPE_U8, 
                                         ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &mode_ext_int);
    uint8_t unknow_attr_20 = 0; // Valeur par défaut
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_client_cluster, 0x0020, ESP_ZB_ZCL_ATTR_TYPE_U8, 
                                         ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &unknow_attr_20);
    uint32_t sampling_period = 30000; // 30 secondes (en ms)
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_client_cluster, 0x0162, ESP_ZB_ZCL_ATTR_TYPE_U32, 
                                         ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &sampling_period);
    uint8_t sensor_type = 2; // 0: interne, 2: externe
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_client_cluster, 0x0172, ESP_ZB_ZCL_ATTR_TYPE_U8, 
                                         ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &sensor_type);
    char pmtsd_data[32] = "P0_M0_T20_S0_D0"; // Valeur initiale pour PMTSD
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_client_cluster, 0xFFF2, ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING, 
                                         ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, pmtsd_data);
    
    // Cluster manuSpecificLumi en mode serveur
    esp_zb_attribute_list_t *esp_zb_manu_specific_lumi_server_cluster = esp_zb_zcl_attr_list_create(0xFCC0);
    if (esp_zb_manu_specific_lumi_server_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Manufacturer Specific server cluster list");
        return;
    }
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_server_cluster, 0x0009, ESP_ZB_ZCL_ATTR_TYPE_U8, 
                                         ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &mode_ext_int);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_server_cluster, 0x0020, ESP_ZB_ZCL_ATTR_TYPE_U8, 
                                         ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &unknow_attr_20);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_server_cluster, 0x0162, ESP_ZB_ZCL_ATTR_TYPE_U32, 
                                         ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &sampling_period);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_server_cluster, 0x0172, ESP_ZB_ZCL_ATTR_TYPE_U8, 
                                         ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &sensor_type);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_server_cluster, 0xFFF2, ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING, 
                                         ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, pmtsd_data);

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

    // Clusters serveurs
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
    int16_t min_heat_setpoint = ESP_ZB_ZCL_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_MIN_VALUE;
    status = esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_server_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID, &min_heat_setpoint);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Thermostat OccupiedHeatingSetpointMin attribute: status 0x%02x", status);
        return;
    }
    int16_t max_heat_setpoint = ESP_ZB_ZCL_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_MAX_VALUE;
    status = esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_server_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID, &max_heat_setpoint);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Thermostat OccupiedHeatingSetpointMax attribute: status 0x%02x", status);
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
    int8_t running_state = 0; // Séquence d'opération par défaut chauffage seulement
    status = esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_server_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID, &running_state);
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

    esp_zb_attribute_list_t *esp_zb_multistate_input_server_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_MULTI_INPUT);
    if (esp_zb_multistate_input_server_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Multistate Input server cluster list");
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
    uint16_t number_of_states = 3; // 3 états pour correspondre aux boutons
    status = esp_zb_multistate_input_cluster_add_attr(esp_zb_multistate_input_server_cluster, 
                                                     ESP_ZB_ZCL_ATTR_MULTI_INPUT_NUMBER_OF_STATES_ID, 
                                                     &number_of_states);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Multistate Input NumberOfStates attribute: status 0x%02x", status);
        return;
    }    
    bool out_of_service = false;
    status = esp_zb_multistate_input_cluster_add_attr(esp_zb_multistate_input_server_cluster, 
                                                     ESP_ZB_ZCL_ATTR_MULTI_INPUT_OUT_OF_SERVICE_ID, 
                                                     &out_of_service);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Multistate Input OutOfService attribute: status 0x%02x", status);
        return;
    }

    // Liste des clusters
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
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_manu_specific_lumi_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_manu_specific_lumi_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
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

    // Mettre à jour l'attribut LocalTemperature du cluster Thermostat serveur
    ESP_LOGI(TAG, "thermostat last local temperature = %d", last_temperature); // Log pour débogage
    int16_t thermostat_temp = (last_temperature != INT16_MIN && last_temperature >= ESP_ZB_ZCL_TEMP_MEASUREMENT_MIN_MEASURED_VALUE_MINIMUM) ? last_temperature : 0;
    status = esp_zb_zcl_set_attribute_val(
        HA_ONOFF_SWITCH_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
        &thermostat_temp,
        false
    );
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to update Thermostat LocalTemperature attribute: status 0x%02x", status);
    }

    // mettre à jour l'attribut heating setpoint du cluster Thermostat serveur
    int16_t heat_setpoint = (last_heating_setpoint != INT16_MIN && last_heating_setpoint >= 500) ? last_heating_setpoint : ESP_ZB_ZCL_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_DEFAULT_VALUE;
    ESP_LOGI(TAG, "thermostat last heating setpoint = %d and heat setpoint = %d", last_heating_setpoint, heat_setpoint); // Log pour débogage
    status = esp_zb_zcl_set_attribute_val(
        HA_ONOFF_SWITCH_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
        &heat_setpoint,
        false
    );
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to update Thermostat OccupiedHeatingSetpoint attribute: status 0x%02x", status);
    }

    // mettre à jour l'attibut cooling setpoint du cluster thermostat
    if (((last_cooling_setpoint >= last_heating_setpoint) ? last_cooling_setpoint : ESP_ZB_ZCL_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_DEFAULT_VALUE) > 
                ((last_heating_setpoint != INT16_MIN && last_heating_setpoint >= 500) ? last_heating_setpoint : ESP_ZB_ZCL_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_DEFAULT_VALUE)) {
        int16_t cool_setpoint = (last_cooling_setpoint >= last_heating_setpoint) ? last_cooling_setpoint : ESP_ZB_ZCL_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_DEFAULT_VALUE;
        ESP_LOGI(TAG, "thermostat last cooling setpoint = %d and cool setpoint = %d", last_cooling_setpoint, cool_setpoint); // Log pour débogage
        status = esp_zb_zcl_set_attribute_val(
            HA_ONOFF_SWITCH_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
            &cool_setpoint,
            false
        );
        if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGW(TAG, "Failed to update Thermostat OccupiedCoolingSetpoint attribute: status 0x%02x", status);
        }
    } else {
        ESP_LOGW(TAG, "Failed to update Thermostat OccupiedCoolingSetpoint incorrect value: last cooling setpoint %d must be >= last heating setpoint %d", last_cooling_setpoint, last_heating_setpoint);
    }

    // mettre à jour l'attribut SystemMode du cluster Thermostat serveur
    int8_t sys_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT; // Mode système chauffage
    status = esp_zb_zcl_set_attribute_val(
        HA_ONOFF_SWITCH_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
        &sys_mode,
        false
    );
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to update Thermostat SystemMode attribute: status 0x%02x", status);
    }

    // mettre à jour l'attribut RunningState du cluster Thermostat serveur
    int8_t running_state = (relay_actual_state == 1) ? 1 : 0;
    status = esp_zb_zcl_set_attribute_val(
        HA_ONOFF_SWITCH_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID,
        &running_state,
        false
    );
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to update Thermostat RunningState attribute: status 0x%02x", status);
    }

    ESP_LOGI(TAG, "Updated server attributes: OnOff=%u, Temp=%d, Humidity=%u",
             on_off_value, temp_value, humidity_value);
}

static void send_hvac_on_command(void) {
    // Adresses MAC
    uint8_t device_mac[8]; // Définir une taille fixe de 8 octets pour une adresse IEEE
    memcpy(device_mac, ieee_addr_w100_bytes, sizeof(device_mac));
    uint8_t hub_mac[] = HUB_IEEE;

    // Préfixe Zigbee
    uint8_t prefix[] = {0xAA, 0x71, 0x32, 0x44};
    uint8_t random_bytes[2];
    random_bytes[0] = (uint8_t)(esp_random() & 0xFF);
    random_bytes[1] = (uint8_t)(esp_random() & 0xFF);

    // En-tête Zigbee
    uint8_t zigbee_header[] = {0x02, 0x41, 0x2F, 0x68, 0x91};

    // ID de message (2 octets aléatoires) + contrôle (0x18)
    uint8_t message_id[2];
    message_id[0] = (uint8_t)(esp_random() & 0xFF);
    message_id[1] = (uint8_t)(esp_random() & 0xFF);
    uint8_t message_control = 0x18;

    // Adresses MAC (device + 00:00 + hub)
    uint8_t payload_macs[16];
    memcpy(payload_macs, device_mac, 8);
    payload_macs[8] = 0x00;
    payload_macs[9] = 0x00;
    memcpy(payload_macs + 10, hub_mac, 6);

    // Charge utile (power=0 pour ON)
    uint8_t payload_tail[] = {0x08, 0x00, 0x08, 0x44, 0x15, 0x0a, 0x01, 0x09, 0xe7, 0xa9, 0xba, 0xe8, 0xb0, 0x83, 0xe5, 0x8a, 0x9f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2a, 0x40};

    // Construire la trame
    uint8_t frame[64];
    int idx = 0;
    for (int i = 0; i < sizeof(prefix); i++) {
        frame[idx++] = prefix[i];
    }
    for (int i = 0; i < sizeof(random_bytes); i++) {
        frame[idx++] = random_bytes[i];
    }
    for (int i = 0; i < sizeof(zigbee_header); i++) {
        frame[idx++] = zigbee_header[i];
    }
    for (int i = 0; i < sizeof(message_id); i++) {
        frame[idx++] = message_id[i];
    }
    frame[idx++] = message_control;
    for (int i = 0; i < sizeof(payload_macs); i++) {
        frame[idx++] = payload_macs[i];
    }
    for (int i = 0; i < sizeof(payload_tail); i++) {
        frame[idx++] = payload_tail[i];
    }

    // Log de la trame pour débogage
    ESP_LOGI(TAG, "HVAC ON trame envoyée au w100:");
    ESP_LOG_BUFFER_HEX(TAG, frame, idx);

    // Préparer l'attribut OCTET_STRING
    uint8_t attr_buf[1 + idx];
    attr_buf[0] = idx; // Longueur de la trame
    memcpy(attr_buf + 1, frame, idx);

    esp_zb_zcl_attribute_t attr = {
        .id = 0xFFF2, // Attribut 65522
        .data.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
        .data.value = attr_buf
    };

    esp_zb_zcl_write_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr_w100_value,
            .dst_endpoint = 1, // Endpoint 1 du thermostat
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT, // Endpoint local
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = 0xFCC0, // Cluster manuSpecificLumi
        .manuf_specific = 1,
        .manuf_code = MANUFACTURER_CODE,
        .attr_number = 1,
        .attr_field = &attr,
    };

    esp_zb_zcl_status_t tsn;
    response_received = false;

    esp_zb_lock_acquire(portMAX_DELAY);
    tsn = esp_zb_zcl_write_attr_cmd_req(&cmd);
    esp_zb_lock_release();

    last_tsn = tsn;
    ESP_LOGI(TAG, "HVAC ON command sent to Aqara W100 (0x%04x, endpoint 1, cluster 0xFCC0), TSN: 0x%02x", short_addr_w100_value, tsn);

}

static void send_hvac_off_command(void)
{
    // Adresse MAC du thermostat
    uint8_t device_mac[8]; // Définir une taille fixe de 8 octets pour une adresse IEEE
    memcpy(device_mac, ieee_addr_w100_bytes, sizeof(device_mac));

    // Préfixe Zigbee pour la commande OFF
    uint8_t prefix[] = {0xAA, 0x71, 0x1C, 0x44, 0x69, 0x1C};

    // En-tête Zigbee
    uint8_t zigbee_header[] = {0x04, 0x41, 0x19, 0x68, 0x91};

    // ID de message (2 octets aléatoires) + contrôle (0x18)
    uint8_t message_id[2];
    message_id[0] = (uint8_t)(esp_random() & 0xFF); // Frame ID
    message_id[1] = (uint8_t)(esp_random() & 0xFF); // Sequence
    uint8_t message_control = 0x18;

    // Construire la trame
    uint8_t frame[34]; // Longueur fixe de 34 octets
    int idx = 0;

    // Ajouter le préfixe
    for (int i = 0; i < sizeof(prefix); i++) {
        frame[idx++] = prefix[i];
    }

    // Ajouter l'en-tête Zigbee
    for (int i = 0; i < sizeof(zigbee_header); i++) {
        frame[idx++] = zigbee_header[i];
    }

    // Ajouter l'ID de message et le contrôle
    for (int i = 0; i < sizeof(message_id); i++) {
        frame[idx++] = message_id[i];
    }
    frame[idx++] = message_control;

    // Ajouter l'adresse MAC du thermostat
    for (int i = 0; i < 8; i++) {
        frame[idx++] = device_mac[i];
    }

    // Remplir avec des zéros jusqu'à 34 octets
    while (idx < 34) {
        frame[idx++] = 0x00;
    }

    // Log de la trame pour débogage
    ESP_LOGI(TAG, "HVAC OFF trame envoyée au w100:");
    ESP_LOG_BUFFER_HEX(TAG, frame, idx);

    // Préparer l'attribut OCTET_STRING
    uint8_t attr_buf[1 + idx];
    attr_buf[0] = idx; // Longueur de la trame
    memcpy(attr_buf + 1, frame, idx);

    esp_zb_zcl_attribute_t attr = {
        .id = 0xFFF2, // Attribut 65522
        .data.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
        .data.value = attr_buf
    };

    esp_zb_zcl_write_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr_w100_value,
            .dst_endpoint = 1, // Endpoint 1 du thermostat
            .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT // Endpoint local
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = 0xFCC0, // Cluster manuSpecificLumi
        .manuf_specific = 1,
        .manuf_code = MANUFACTURER_CODE, // 0x115F
        .attr_number = 1,
        .attr_field = &attr
    };

    response_received = false;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_write_attr_cmd_req(&cmd);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "HVAC OFF command sent to Aqara W100 (0x%04x, endpoint 1, cluster 0xFCC0)", short_addr_w100_value);

    // Attendre une réponse ZCL
    vTaskDelay(pdMS_TO_TICKS(2000));

    read_thermostat_attributes_pmtsd();

}

static void send_pmtsd_command(uint8_t power, uint8_t mode, float temp, uint8_t speed, uint8_t display) {

    // Construire T comme chaîne avec décimale (ex: "26.5")
    char temp_str[6];
    snprintf(temp_str, sizeof(temp_str), "%.1f", temp);

    // Vérifier la plage de température (5.0°C à 30.0°C)
    if (temp < 5.0 || temp > 30.0) {
        ESP_LOGE(TAG, "Temperature out of range (5.0-30.0°C)");
        return;
    }

    ESP_LOGI(TAG, "Preparing to set PMTSD with temperature: %.1f°C", temp);

    ESP_LOGI(TAG, "Setting PMTSD: Power=%d, Mode=%d, Temp=%.1f (°C, encoded as T%s), Speed=%d, Display=%d",
             power, mode, temp, temp_str, speed, display);

    // Construire la chaîne PMTSD avec T comme chaîne décimale
    char pmtsd_str[32];
    snprintf(pmtsd_str, sizeof(pmtsd_str), "P%d_M%d_T%s_S%d_D%d", power, mode, temp_str, speed, display);
    uint8_t pmtsd_bytes[32];
    uint8_t pmtsd_len = strlen(pmtsd_str);
    for (uint8_t i = 0; i < pmtsd_len; i++) {
        pmtsd_bytes[i] = pmtsd_str[i];
    }

    // Construire la trame
    uint8_t frame[64];
    uint8_t idx = 0;

    // En-tête fixe aligné sur Zigbee2MQTT
    uint8_t fixed_header[] = {
        0xAA, 0x71, 0x1F, 0x44, // Préfixe Lumi
        0x00, // Counter (rempli plus tard)
        0x00, // Checksum (rempli plus tard)
        0x05, 0x41, 0x1C, // Action et en-tête
        0x00, 0x00, // Réservé
        0x54, 0xEF, 0x44, 0x80, 0x71, 0x1A, // Adresse MAC du hub
        0x08, 0x00, 0x08, 0x44, // Suffixe
        pmtsd_len // Longueur PMTSD
    };
    uint8_t counter = (uint8_t)(esp_random() & 0xFF);
    fixed_header[4] = counter;

    // Copier l'en-tête
    for (uint8_t i = 0; i < sizeof(fixed_header); i++) {
        frame[idx++] = fixed_header[i];
    }

    // Ajouter la charge utile PMTSD
    for (uint8_t i = 0; i < pmtsd_len; i++) {
        frame[idx++] = pmtsd_bytes[i];
    }

    // Calculer le checksum
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < idx; i++) {
        checksum = (checksum + frame[i]) & 0xFF;
    }
    frame[5] = checksum;

    // Log de la trame pour débogage
    ESP_LOGI(TAG, "Set PMTSD trame envoyée au W100:");
    ESP_LOG_BUFFER_HEX(TAG, frame, idx);

    // Préparer l'attribut OCTET_STRING
    uint8_t attr_buf[1 + idx];
    attr_buf[0] = idx; // Longueur de la chaîne OCTET_STRING
    memcpy(attr_buf + 1, frame, idx);

    esp_zb_zcl_attribute_t attr = {
        .id = 0xFFF2,
        .data.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
        .data.value = attr_buf
    };

    esp_zb_zcl_write_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr_w100_value,
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

    read_thermostat_attributes_pmtsd();
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
static void set_sensor_mode(const char *mode_ext_int) {
    bool is_external = (strcmp(mode_ext_int, "external") == 0);
    uint8_t action = is_external ? 0x02 : 0x04;

    /* Timestamp BE (uptime en secondes) */
    uint64_t us = esp_timer_get_time();
    uint32_t sec = (uint32_t)(us / 1000000);
    uint8_t timestamp[4] = { (sec >> 24) & 0xFF, (sec >> 16) & 0xFF, (sec >> 8) & 0xFF, sec & 0xFF };

    uint8_t device_ieee[8]; // Définir une taille fixe de 8 octets pour une adresse IEEE
    memcpy(device_ieee, ieee_addr_w100_bytes, sizeof(device_ieee));
    uint8_t fictive_sensor[8] = {0x00, 0x15, 0x8D, 0x00, 0x01, 0x9D, 0x1B, 0x98};
    uint8_t chinese_humi[6] = {0xE6, 0xB9, 0xBF, 0xE5, 0xBA, 0xA6};
    uint8_t chinese_temp[6] = {0xE6, 0xB8, 0xA9, 0xE5, 0xBA, 0xA6};

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
            .dst_addr_u.addr_short = short_addr_w100_value,
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
            .dst_addr_u.addr_short = short_addr_w100_value,
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

    ESP_LOGI(TAG, "Mode sensor défini sur %s", mode_ext_int);
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
            .dst_addr_u.addr_short = short_addr_w100_value,
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

/* Fonction pour définir external_humidity (% batterie, à implémenter plus tard) mais actuellement utilisée pour afficher 100% si chauffage actif */
static void set_external_humidity(uint16_t battery_percent) {
    uint8_t fictive_sensor[8] = {0x00, 0x15, 0x8D, 0x00, 0x01, 0x9D, 0x1B, 0x98};

    float f = (float)battery_percent;
    union { float fl; uint32_t u; } fu;
    fu.fl = f;
    uint8_t humi_buf[4] = { (fu.u >> 24) & 0xFF, (fu.u >> 16) & 0xFF, (fu.u >> 8) & 0xFF, fu.u & 0xFF };
    ESP_LOGI(TAG, "Batterie value: %.2f, encoded as: %02x %02x %02x %02x", 
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
            .dst_addr_u.addr_short = short_addr_w100_value,
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

    ESP_LOGI(TAG, "Batterie défini sur %d.%d %%, TSN unknown", battery_percent / 100, abs(battery_percent % 100));
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
    // Configuration du timer LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configuration du canal LEDC
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LED_GPIO,
        .duty           = 128, // 50% luminosité
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    // Allumer la LED
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);

    esp_err_t err;
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    err = load_settings_from_nvs(); // Charger les paramètres au démarrage
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Settings loaded from NVS successfully");
        wifi_init();
    } else {
        ESP_LOGW(TAG, "Failed to load settings from NVS, presume first boot, connect zigbee then reboot");
        first_boot = true;
        xTaskCreate(esp_zb_task, "Zigbee_main", 4096 * 4, NULL, 2, &zb_task_handle);
    }
    xTaskCreate(watchdog_task, "watchdog_task", 2048, NULL, 1, NULL);
}