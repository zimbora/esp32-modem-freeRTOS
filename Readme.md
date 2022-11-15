#esp32-modem-freeRTOS

## Important
All topics start with the prefix
:project/:uid/...
':project' and ':uid' are passed with mqtt_configure_connection method

## Description
This library implements an independent process to manage LTE and WiFi interfaces

## Functionalities
- Manage APN connection (LTE)
- Manage Context connection  
- Manage MQTT connection
- Manage TCP connection
- Read messages from Queue and sends to modem
- Get messages from modem and write on respective Queue

## Implementation
Establish connection to APN
Create contexts
Receives configurations for TCP
Receives configurations for MQTT
Create and keep persistent connections TCP/MQTT
Read and write on respective Queues

# Public Methods
