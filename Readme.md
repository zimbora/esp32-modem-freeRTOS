#esp32-modem-freeRTOS

## Usage

  Edit editable_macros.h file to change macros according to your needs
  Check examples folder to see examples. Each example imports a credentials.h file that is not present on this repository. You have to create it at your own

## Important
  All topics start with the prefix
  :project/:uid/...
  ':project' and ':uid' are passed with mqtt_configure_connection method

## Description
  This library implements an independent process to manage LTE and WiFi interfaces

## Functionalities
  - Manage APN connection (LTE)
  - Manage Context connection (LTE)  
  - Manage WIFI connection  
  - Manage TCP+SSL multi-connection (LTE for now)
  - Manage MQTT/MQTTS multi-connection (WIFI+LTE)
  - Manage HTTP/HTTPS multi-requests (WIFI+LTE)
  - Read messages from Queue and sends to modem
  - Get messages from modem and write on respective Queue
  - SMS not supported for now

## Examples
 - HTTP (WIFI/LTE)
 - HTTPS (WIFI/LTE)
 - MQTT (WIFI/LTE)
 - TCP (LTE)

## Public Methods

[void init(const char* ssid, const char* password)](#Init-WiFi)
[void init(uint16_t cops, uint8_t mode, uint8_t pwkey)](#Init-LTE)
[void loop()](#Loop)
[bool set_context(uint8_t contextID, String apn, String user, String pwd)](#Set-context)

### TCP
[void tcp_configure_connection(uint8_t clientID, uint8_t contextID, String host, uint16_t port)](#TCP-configure-connection)
[void tcp_setup(void(*callback1)(uint8_t clientID),void(*callback2)(uint8_t clientID))](#TCP-setup)
[TCP_MSG* tcp_getNextMessage(TCP_MSG *pxRxedMessage)](#TCP-getNextMessage)
[bool tcp_pushMessage(uint8_t clientID, const char* data, uint16_t len)](#TCP-pushMessage)

### HTTP
[bool http_pushMessage(uint8_t contextID, uint8_t clientID, String host, String path, String method)](#HTTTP-pushMessage)
[HTTP_HEADER_MSG* http_header_getNextMessage(HTTP_HEADER_MSG *pxRxedMessage)](#HTTTP-header-getNextMessage)
[HTTP_BODY_MSG* http_body_getNextMessage(HTTP_BODY_MSG *pxRxedMessage)](#HTTTP-body-getNextMessage)

### MQTT
[void mqtt_configure_connection(uint8_t clientID, uint8_t contextID, String project, String uid, String host, uint16_t port, String user, String pwd)](#MQTT-configure-connection)
[void mqtt_set_will_topic(uint8_t clientID, String topic, String payload)](#MQTT-set-will-topic)
[void mqtt_add_subscribe_topic(uint8_t clientID, uint8_t index, String topic)](#MQTT-add-subscribe-topic)
[void mqtt_setup(void(*callback)())](#MQTT-setup)
[bool mqtt_pushMessage(uint8_t clientID, const String& topic, const String& message, uint8_t qos, uint8_t retain)](#MQTT-pushMessage)
[MQTT_MSG* mqtt_getNextMessage(MQTT_MSG *pxRxedMessage)](#MQTT-getNextMessage)

## Examples
  Run programs inside examples folder to check how it works

### demo-tcp
  Establishes connection to a server, do a request and reads its response

### demo-http
  Two processes running simultaneously:
    - One process is controlling the modem and executing requests
    - The other one is used to send and received requests to and from the first process

### demo-mqtt
  Two processes running simultaneously:
    - One process is controlling the modem, handling mqtt connection and executing requests
    - The other one is used to send and received requests to and from the first process


## Unit Test with Arduino
  Not available for now
### unitTest
  Not available for now

## Public Methods - Extension

#### Init WiFi
```
void init(const char* ssid, const char* password);
```

#### Init LTE
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

### HTTP

#### HTTTP pushMessage
* use it to do http requests
*
* @contextID 1-11, context id
* @clientID 0-5, tcp index client
* @host
* @port
* @path
* @method
```
bool http_pushMessage(uint8_t contextID, uint8_t clientID, String host, String path, String method)
```

#### HTTTP header getNextMessage
* use it to get http header messages.
*
* returns a pointer to HTTP_HEADER_MSG struct containing the received message
* if no message is available it returns NULL
```
HTTP_HEADER_MSG* http_header_getNextMessage(HTTP_HEADER_MSG *pxRxedMessage)
```

#### HTTTP body getNextMessage
* use it to get http body messages.
*
* returns a pointer to HTTP_BODY_MSG struct containing the received message
* if no message is available it returns NULL
```
HTTP_BODY_MSG* http_body_getNextMessage(HTTP_BODY_MSG *pxRxedMessage)
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
