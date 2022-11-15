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

## Public Methods

[void init(uint16_t cops, uint8_t mode, uint8_t pwkey)](#Init)
[void loop()](#Loop)
[bool set_context(uint8_t contextID, String apn, String user, String pwd)](#Set-context)

### MQTT
[void mqtt_configure_connection(uint8_t clientID, uint8_t contextID, String project, String uid, String host, uint16_t port, String user, String pwd)](#MQTT-configure-connection)
[void mqtt_set_will_topic(uint8_t clientID, String topic, String payload)](#MQTT-set-will-topic)
[void mqtt_add_subscribe_topic(uint8_t clientID, uint8_t index, String topic)](#MQTT-add-subscribe-topic)
[void mqtt_setup(void(*callback)())](#MQTT-setup)
[bool mqtt_pushMessage(uint8_t clientID, const String& topic, const String& message, uint8_t qos, uint8_t retain)](#MQTT-pushMessage)
[MQTT_MSG* mqtt_getNextMessage(MQTT_MSG *pxRxedMessage)](#MQTT-getNextMessage)

## Examples
  Run programs inside examples folder to check how it works

### demo
  Establish connection to one mqtt broker, subscribes topics, receives and publish messages

## Unit Test with Arduino
  Not available for now
### unitTest
  Not available for now

## Public Methods - Extension

#### Init
```
void init(uint16_t cops, uint8_t mode, uint8_t pwkey);
```

#### Loop
```
void loop();
```

#### Set context
```
bool set_context(uint8_t contextID, String apn, String user, String pwd);
```

### MQTT
#### MQTT configure connection
```
void mqtt_configure_connection(uint8_t clientID, uint8_t contextID, String project, String uid, String host, uint16_t port, String user, String pwd);
```
#### MQTT set will topic
```
void mqtt_set_will_topic(uint8_t clientID, String topic, String payload);
```
#### MQTT add subscribe topic
```
void mqtt_add_subscribe_topic(uint8_t clientID, uint8_t index, String topic);
```
#### MQTT setup
```
void mqtt_setup(void(*callback)());
```
#### MQTT pushMessage
```
bool mqtt_pushMessage(uint8_t clientID, const String& topic, const String& message, uint8_t qos, uint8_t retain);
```
#### MQTT getNextMessage
```
void init(uint16_t cops, uint8_t mode, uint8_t pwkey);
```
