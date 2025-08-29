/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#ifndef ESP_ZIGBEE_CHAUFFAGE_H
#define ESP_ZIGBEE_CHAUFFAGE_H

#include "esp_zigbee_core.h"

// Configuration Wi-Fi
#define WIFI_SSID "Bbox-417260CF-Legacy"
#define WIFI_PASSWORD "KqphCaz6QgxZc697GS"
#define WIFI_MAX_RETRIES 10

// Paramètres IP statique
#define EXAMPLE_STATIC_IP_ADDR "192.168.1.160"  // Adresse IP statique pour l'appareil
#define EXAMPLE_STATIC_GW_ADDR "192.168.1.1"  // Adresse de la passerelle (votre routeur)
#define EXAMPLE_STATIC_NETMASK_ADDR "255.255.255.0"  // Masque de sous-réseau standard
#define EXAMPLE_MAIN_DNS_SERVER EXAMPLE_STATIC_GW_ADDR  // Serveur DNS principal (généralement la même que la passerelle)
#define EXAMPLE_BACKUP_DNS_SERVER "8.8.8.8"  // Serveur DNS de secours (Google DNS)

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false       /* enable the install code policy for security */
#define HA_ONOFF_SWITCH_ENDPOINT        10          /* esp switch device endpoint */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     (1l << 11)  /* Zigbee primary channel mask use in the example */
#define RELAY_CHAUFF                    0xB377      /* Relay for chauffage */
#define RELAY_BINDING_EP                1           /* Endpoint for binding to the relay */
#define DEFAULT_OCCUPIED_HEATING_SETPOINT 2100      /* en centièmes de degré */
#define THERMOSTAT                      0xC0E4      /* Thermostat device ID SMABIT = 0xD5AA Aqara W100 = 0xC0E4 */
#define THERMOSTAT_IEEE                 {0x54, 0xEF, 0x44, 0x10, 0x01, 0x26, 0x3E, 0xF3} /* IEEE address of the thermostat */
#define HUB_IEEE                        {0x54, 0xEF, 0x44, 0x80, 0x71, 0x1A} // Adresse MAC d'un hub M3 aqara, NE PAS MODIFIER
#define HYSTERESIS_MOINS                10          /* Hysteresis inférieur en centièmes de degré */
#define HYSTERESIS_PLUS                 10          /* Hysteresis supérieur en centièmes de degré */
#define MANUFACTURER_CODE               0x115F      // Code Bitron (SMABIT) = 0x1071 Aqara = 0x115F
#define MAX_RETRIES                     3           // Nombre maximum de tentatives

/* Configuration par défaut */
#define HEATING_SETPOINT_DEFAULT        1900        // heating setpoint par défaut pour le premier lancement
#define COOLING_SETPOINT_DEFAULT        3500        // cooling setpoint par défaut pour le premier lancement
#define HIGH_HYST_DEFAULT               10          // Hysteresis supérieur par défaut en centièmes de degré
#define LOW_HYST_DEFAULT                10          // Hysteresis inférieur par défaut en centièmes de degré

// Attributs personnalisés dans le cluster Basic
#define ZCL_THERMOSTAT_ATTR_RELAY_STATE          0xF000

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x08""@Noyax37"         /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x0F""Thermostat_W100"    /* Customized model identifier */

#define ESP_ZB_ZR_CONFIG()                                       \
    {                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,               \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,       \
        .nwk_cfg.zczr_cfg = {                                   \
            .max_children = 10,                                 \
        },                                                      \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

#endif /* ESP_ZIGBEE_CHAUFFAGE_H */