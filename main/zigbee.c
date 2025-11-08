#include "zigbee.h"
#include "thermostat.h"
#include "utils.h"
#include "esp_zb_core.h"
#include "esp_zb_ha_standard.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ZB_CORE";

void esp_zb_task(void *pvParameters)
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
    esp_zb_attribute_list_t *esp_zb_manu_specific_lumi_cluster = esp_zb_zcl_attr_list_create(0xFCC0); // Cluster personnalisé
    if (esp_zb_manu_specific_lumi_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create Manufacturer Specific cluster list");
        return;
    }
    uint8_t mode = 0; // Valeur par défaut
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_cluster, 0x0009, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &mode);
    uint32_t sampling_period = 30000; // 30 secondes (en ms)
    esp_zb_custom_cluster_add_custom_attr(esp_zb_manu_specific_lumi_cluster, 0x0162, ESP_ZB_ZCL_ATTR_TYPE_U32, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &sampling_period);
    uint8_t sensor_type = 2; // 0: interne, 2: externe
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

    // Clusters serveurs
    esp_zb_attribute_list_t *esp_zb_on_off_server_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    if (esp_zb_on_off_server_cluster == NULL) {
        ESP_LOGE(TAG, "Failed to create On/Off server cluster list");
        return;
    }
    uint8_t init_on_off_value = 0; // Initialisé à OFF
    esp_zb_zcl_status_t status = esp_zb_on_off_cluster_add_attr(esp_zb_on_off_server_cluster, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &init_on_off_value);
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
    int16_t init_temp_value = 0; // Valeur initiale
    status = esp_zb_temperature_meas_cluster_add_attr(esp_zb_temperature_server_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &init_temp_value);
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
    uint16_t init_humidity_value = 0; // Valeur initiale
    status = esp_zb_humidity_meas_cluster_add_attr(esp_zb_humidity_server_cluster, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &init_humidity_value);
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
    int16_t init_cool_setpoint = 3500; // 35.00°C
    status = esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_server_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID, &init_cool_setpoint);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Thermostat OccupiedCoolingSetpoint attribute: status 0x%02x", status);
        return;
    }
    int16_t init_heat_setpoint = ESP_ZB_ZCL_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_DEFAULT_VALUE;
    status = esp_zb_thermostat_cluster_add_attr(esp_zb_thermostat_server_cluster, ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &init_heat_setpoint);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Thermostat OccupiedHeatingSetpoint attribute: status 0x%02x", status);
        return;
    }
    running_state = 0; // Séquence d'opération par défaut chauffage seulement
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
    uint16_t number_of_states = 12; // 3 boutons à 4 états pour correspondre aux boutons
    status = esp_zb_multistate_input_cluster_add_attr(esp_zb_multistate_input_server_cluster, 
                                                     ESP_ZB_ZCL_ATTR_MULTI_INPUT_NUMBER_OF_STATES_ID, 
                                                     &number_of_states);
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add Multistate Input NumberOfStates attribute: status 0x%02x", status);
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
    esp_zb_cluster_list_add_multistate_input_cluster(esp_zb_cluster_list, esp_zb_multi_state_input_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
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
    esp_zb_cluster_list_add_multistate_input_cluster(esp_zb_cluster_list, esp_zb_multistate_input_server_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);    

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
