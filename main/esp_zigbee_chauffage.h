/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#ifndef ESP_ZIGBEE_CHAUFFAGE_H
#define ESP_ZIGBEE_CHAUFFAGE_H

#include "esp_zigbee_core.h"
#include <string.h>
#include <ctype.h>

// configuration LED onboard
#define LED_GPIO 8
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER LEDC_TIMER_0

// Configuration Wi-Fi
#define WIFI_SSID "Bbox-417260CF-Legacy"
#define WIFI_PASSWORD "KqphCaz6QgxZc697GS"
#define WIFI_MAX_RETRIES 10

// Paramètres IP statique
#define EXAMPLE_STATIC_IP_ADDR "192.168.1.160"
#define EXAMPLE_STATIC_GW_ADDR "192.168.1.1"
#define EXAMPLE_STATIC_NETMASK_ADDR "255.255.255.0"
#define EXAMPLE_MAIN_DNS_SERVER EXAMPLE_STATIC_GW_ADDR
#define EXAMPLE_BACKUP_DNS_SERVER "8.8.8.8"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false
#define HA_ONOFF_SWITCH_ENDPOINT        10
#define ESP_ZB_PRIMARY_CHANNEL_MASK     (1l << 11)
#define RELAY_BINDING_EP                1
#define DEFAULT_OCCUPIED_HEATING_SETPOINT 2100
// gardé pour mémoire:
//#define RELAY_CHAUFF                    0xB377
//#define THERMOSTAT                      0xC0E4
//#define RELAY_CHAUFF_IEEE               0x7c2c67fffe75c28c
//#define THERMOSTAT_IEEE                 0x54ef441001263ef3
#define HUB_IEEE                        {0x54, 0xEF, 0x44, 0x80, 0x71, 0x1A}
#define HYSTERESIS_MOINS                10
#define HYSTERESIS_PLUS                 10
#define MANUFACTURER_CODE               0x115F
#define MAX_RETRIES                     3

/* Configuration par défaut */
#define HEATING_SETPOINT_DEFAULT        1900
#define COOLING_SETPOINT_DEFAULT        3500
#define HIGH_HYST_DEFAULT               10
#define LOW_HYST_DEFAULT                10

// Attributs personnalisés dans le cluster Basic
#define ZCL_THERMOSTAT_ATTR_RELAY_STATE          0xF000

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x08""@Noyax37"
#define ESP_MODEL_IDENTIFIER "\x0F""Thermostat_W100"

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

/**
 * Convertit une adresse IEEE au format "0xHHHHHHHHHHHHHHHH" en tableau d'octets.
 * @param addr_str Chaîne d'entrée (ex: "0x54ef441001263ef3")
 * @param out_array Tableau de sortie de 8 octets
 * @return ESP_OK si succès, ESP_FAIL sinon
 */
esp_err_t convert_ieee_address(const char *addr_str, uint8_t out_array[8]);

/**
 * Convertit une adresse courte au format "0xHHHH" en un uint16_t.
 * @param addr_str Chaîne d'entrée (ex: "0xC0E4")
 * @param out_value Pointeur vers la valeur uint16_t de sortie
 * @return ESP_OK si succès, ESP_FAIL sinon
 */
esp_err_t convert_short_address(const char *addr_str, uint16_t *out_value);


#endif /* ESP_ZIGBEE_CHAUFFAGE_H */