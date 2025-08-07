README - Contrôle Thermostat Zigbee Aqara W100
Description
Ce projet implémente un contrôleur de thermostat basé sur une puce ESP32, utilisant le protocole Zigbee pour communiquer avec la aqara w100 et un relais. L'application permet de surveiller la température, ajuster le setpoint (point de consigne) et contrôler l'état du relais via une interface web accessible sur le réseau local.
Prérequis

Matériel : Une carte ESP32 compatible avec le firmware ESP-ZB.
Logiciel : Environnement de développement ESP-IDF installé.
Réseau Zigbee : Un coordinateur Zigbee (par exemple, un dongle Zigbee connecté à un Raspberry Pi avec Zigbee2MQTT).

Configuration Initiale

Configuration Zigbee :

Avant de lancer le système, le réseau Zigbee doit être configuré via un outil comme Zigbee2MQTT.
Ajoutez le thermostat (adresse : 0xD5AA) et le relais (adresse : 0xB377) au réseau Zigbee.
Effectuez le binding entre l'ESP32 (endpoint 10) et les appareils Zigbee pour permettre la communication (voir les adresses IEEE dans esp_zigbee_chauffage.h).
Une fois le binding effectué, l'ESP32 peut fonctionner en autonomie sans dépendre de Zigbee2MQTT.


Configuration Wi-Fi :

Modifiez les constantes WIFI_SSID et WIFI_PASSWORD dans esp_zigbee_chauffage.h avec vos identifiants Wi-Fi.
Les paramètres IP statiques sont définis (par exemple, 192.168.1.160). Ajustez-les si nécessaire dans le même fichier.


Compilation et Téléversement :

Compilez le projet avec idf.py build.
Téléversez le firmware avec idf.py -p <PORT> flash.
Téléversez le système de fichiers SPIFFS contenant index.html avec idf.py -p <PORT> spiffs upload.



Fonctionnement

Une fois démarré, l'ESP32 se connecte au réseau Wi-Fi et rejoint le réseau Zigbee.
Une interface web est accessible à l'adresse IP statique (par défaut : 192.168.1.160).
Fonctionnalités :
Affichage en temps réel de la température, du setpoint, et de l'état du relais.
Mise à jour du setpoint via l'interface web (bouton "Update Setpoint").
Contrôle automatique du relais basé sur la température et le setpoint (avec hystérésis par défaut à 0.1°C si non configuré).


Les données sont mises à jour toutes les 5 secondes via des requêtes AJAX.

Dépannage

Consultez les logs via le moniteur série pour diagnostiquer les erreurs (connexion Wi-Fi, Zigbee, ou serveur web).
Si le binding Zigbee échoue, vérifiez les adresses IEEE et réinitialisez les appareils.

Contribution
Des contributions sont bienvenues pour améliorer la gestion des hystérésis ou ajouter de nouvelles fonctionnalités. Contactez l'auteur pour plus d'informations.

