README - Contrôle Thermostat Zigbee Aqara W100
Description
Ce projet implémente un contrôleur de thermostat basé sur une puce ESP32, utilisant le protocole Zigbee pour communiquer avec la aqara w100 et un relais. L'application permet de surveiller la température, ajuster le setpoint (point de consigne) et contrôler l'état du relais via une interface web accessible sur le réseau local.
Prérequis

Matériel : - Une carte ESP32 compatible avec le firmware ESP-ZB. 
           - 3 LED adressables pour reconnaitre l'état de fonctionnement de l'esp32, au démarrage toutes les LED sont allumées au rouge en attente d'initialisation: 
             - LED1 (bleu) : Indique l'état de la mémoire NVS (bleu clignotant: mémoire NVS vide ou certaines données manquantes, bleu fixe: OK).
             - LED2 (jeune) : Indique l'état de la connexion wifi (jaune clignotant si non connecté, fixe si connecté).
             - LED3 (verte) : Indique l'état de la connexion zigbee (même prinipe que la LED2 en vert).

Logiciel :  Codage sous VS code
            Environnement de développement ESP-IDF installé, version 5.5.1 
            SDK Zigbee version 1.6.6
Réseau Zigbee : Un coordinateur Zigbee. L'esp32 peut servir de routeur (voir les paramètres) mais non testé.

Configuration Initiale

Configuration Zigbee :

Avant de lancer le système, le réseau Zigbee doit être configuré via un outil comme Zigbee2MQTT.
Ajoutez le thermostat (adresse : 0xD5AA) et le relais (adresse : 0xB377) au réseau Zigbee.
Effectuez le binding entre l'ESP32 (endpoint 10) et les appareils Zigbee pour permettre la communication (voir les adresses IEEE dans esp_zigbee_chauffage.h).
Bindig entre l'aqara w100 et l'ESP32:
  1. Endpoint 1 vers ESP32, clusters à lier: genMultistateInput, manuSpecificLumi, msRelativeHumidity et msTemperatureMeasurement
  2. Endpoint 2 vers ESP32, cluster à lier: genMultistateInput
  3. Endpoint 3 vers ESP32, cluster à lier: genMultistateInput

Une fois le binding effectué, il est nécessaire de configurer les rapports (les valeurs ci dessous sont celles que j'appliquent, vous pouvez personnaliser comme bon vous semble mais attention à ne pas surcharger le réseau zigbee ni à perdre des informations):
  - Pour l'aqara w100 (endpoint 1):
    - Cluster msTemperatureMeasurement : intervalle min de rapport 0, rapport toutes les 300 secondes ou si changement min de rapport = 10 ( = 0.1°C).
    - Cluster msRelativeHumidity : intervalle min de rapport 0, rapport toutes les 900 secondes ou si changement min de rapport = 100 ( = 1%).
    - Cluster genMultistateInput : intervalle min de rapport 0, rapport toutes les 65534 secondes (inférieure à 1 de la valeur de désactivation) ou si changement min de rapport = 0 (tout changement est signalé).
  - Pour le relais (endpoint 1):
    - Cluster onOff : intervalle min de rapport 0, rapport toutes les 600 secondes ou si changement min de rapport = 0 (tout changement est signalé).


Configuration Wi-Fi :

Modifiez les constantes WIFI_SSID et WIFI_PASSWORD dans esp_zigbee_chauffage.h avec vos identifiants Wi-Fi.
Les paramètres IP statiques sont définis (par exemple, 192.168.1.160). Ajustez-les si nécessaire dans le même fichier.


Compilation et Téléversement :

Compilez le projet avec idf.py build.
Téléversez le firmware avec idf.py -p <PORT> flash.
Téléversez le système de fichiers SPIFFS contenant index.html avec idf.py -p <PORT> spiffs upload.



Fonctionnement

Au premier boot ou après un erase flash complet, l'esp démarre, se connecte au wifi puis initialise le réseau zigbee. Mettre z2mqtt en mode appairage, ne pas hésiter à positionner l'esp32-c6 au plus proche du coordinateur ou d'un routeur pendant l'appairage.

Une fois démarré, l'ESP32 se connecte au réseau Wi-Fi et rejoint le réseau Zigbee.
Une interface web est accessible à l'adresse IP statique (par défaut : 192.168.1.160).
Depuis cette inteface il est nécessaire d'aller dans les paramètres (lien en haut de la page) pour paramétrer à minima les adresses utiles au zigbee.
Une fois les adresses zigbee paramétré et retour à l'interface du départ alors il faut configurer le setpoint et les hystérésis (par défaut 19°C 0.1°C si non configuré et la LED bleu du NVS continue à clignoter).

Fonctionnalités :
Affichage de la température, du setpoint, et de l'état du relais sur l'interface web + réglage de certains paramètres.
Sur le W100:
  - Si la fonction thermostat est en service: le setpoint est affiché en température comme "secondary" sur l'écran principal et l'humidité du secondary sert à afficher l'état du relais: 0% = chauffage non actif, 99% = chauffage actif.
  - mise au point du setpoint via les boutons "+" et "-" un appui simple ajoute ou retire 0.1°C au setpoint et un double appui de même mais avec 1°C. 
  - Un appui long au centre met en service ou éteint la fonction thermostat.


Contrôle automatique du relais basé sur la température et le setpoint (avec hystérésis par défaut à 0.1°C si non configuré).
Possibilité de "jouer" avec la ligne du milieu du W100 mais pas encore exploité par le partie thermostat car la température ne peut pas être réglée au 10ème de degré celsius.


Les données sont mises à jour toutes les 5 secondes via des requêtes AJAX.

Dépannage

Consultez les logs via le moniteur série pour diagnostiquer les erreurs (connexion Wi-Fi, Zigbee, ou serveur web).
Si le binding Zigbee échoue, vérifiez les adresses IEEE et réinitialisez les appareils.

Contribution
Des contributions sont bienvenues pour améliorer la gestion des hystérésis ou ajouter de nouvelles fonctionnalités. Contactez l'auteur pour plus d'informations.

