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

### TCP
[void tcp_configure_connection(uint8_t clientID, uint8_t contextID, String host, uint16_t port)](#TCP-configure-connection)
[void tcp_setup(void(*callback1)(uint8_t clientID),void(*callback2)(uint8_t clientID))](#TCP-setup)
[TCP_MSG* tcp_getNextMessage(TCP_MSG *pxRxedMessage)](#TCP-getNextMessage)
[bool tcp_pushMessage(uint8_t clientID, const char* data, uint16_t len)](#TCP-pushMessage)

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

### TCP

#TCP configure connection
* call it before tcp_setup
* changes tcp connection parameters
* while clientID has contextID != 0, loop function will try to keep connection activated
*
* @clientID 0-5, limited to MAX_TCP_CONNECTIONS defined in bgxx library
* @contextID 1-16, limited to MAX_CONNECTIONS defined in bgxx library
* @host - IP or DNS of server
* @port
```
void tcp_configure_connection(uint8_t clientID, uint8_t contextID, String host, uint16_t port)
```

#TCP setup
* configures callbacks to be called when connection is established and closed
```
void tcp_setup(void(*callback1)(uint8_t clientID),void(*callback2)(uint8_t clientID))
```

#TCP getNextMessage
* use it to get received messages.
*
* returns a pointer to TCP_MSG struct containing the received message
* if no message is available it returns NULL
```
TCP_MSG* tcp_getNextMessage(TCP_MSG *pxRxedMessage)
```
#TCP pushMessage
* use it to send tcp messages
*
* @clientID 0-5, tcp index client
* @data - payload to be sent
* @len - payload len
*
* @retain true|false
```
bool tcp_pushMessage(uint8_t clientID, const char* data, uint16_t len)
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
