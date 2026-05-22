#esp32-modem-freeRTOS

## Dependencies 
  - Board esp32 by Espressif Systems 3.0.7
  - WiFi.h v1.2.7
  - HTTPClient v2.2.0
  - EspMQTTClient.h v1.13.3
  - TimeLib v1.6.1
  - esp32-BG95 v1.0.5

## Usage
  Edit editable_macros.h file to change macros according to your needs
  Check examples folder to see examples. Each example imports a credentials.h file that is not present on this repository. You have to create it at your own

## Examples

### demo-mqtt
  Create the "credentials.h" file in demo-mqtt folder
  Edit the file according to your setup    
```
  // WIFI credentials
  #define WIFI_SSID "ssid"
  #define WIFI_PASSWORD "password"

  // MQTT credentials
  #define MQTT_HOST_1 "mqtt host"
  #define MQTT_PORT_1 1883
  #define MQTT_USER_1 "device"
  #define MQTT_PASSWORD_1 "device"
  #define MQTT_PROJECT "esp32/freeRTOS2"
  #define MQTT_UID_PREFIX "uid:"
  #define MQTT_WILL_SUBTOPIC "status"
  #define MQTT_WILL_PAYLOAD "offline"
```

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
  - Active ARP scan + reverse DNS (WiFi only)
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

### ARP
[uint8_t arp_scan(ARP_HOST* results, uint8_t maxResults, uint32_t timeout_ms)](#ARP-scan)
[bool arp_scan_ip(IPAddress ip, ARP_HOST* result, uint32_t timeout_ms)](#ARP-scan-ip)
[uint8_t arp_scan_with_names(NETWORK_HOST* results, uint8_t maxResults, uint32_t arp_timeout_ms, uint32_t dns_timeout_ms)](#ARP-scan-with-names)
[bool arp_scan_ip_with_name(IPAddress ip, NETWORK_HOST* result, uint32_t arp_timeout_ms, uint32_t dns_timeout_ms)](#ARP-scan-ip-with-name)

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

### demo-arp-scan
  Perform an active arp scan on network after registration

  build command
  ```
  project=demo-arp-scan
  arduino-cli compile -b esp32:esp32:esp32c5 \
  --build-property build.partitions=min_spiffs \
  --build-property upload.maximum_size=1966080 \
  --build-path ./build/${project} ./examples/${project}/${project}.ino 2>&1
  ```
  flash command
  ```
  filename=./build/demo-arp-scan/demo-arp-scan.ino.merged.bin
  port=/dev/cu.usbmodem1101
  sudo esptool --port ${port} erase_flash 
  sudo esptool --port ${port} --baud 460800 write_flash 0x0 ${filename}
  ```

### demo-arp-reverse
  Performs an ARP sweep of the entire subnet and then resolves each discovered
  IP to a hostname via reverse DNS (PTR query to the router's DNS server).
  Works for all devices on the network including phones, TVs, and IoT devices
  regardless of what software they run.

  build command
  ```
  project=demo-arp-reverse
  arduino-cli compile -b esp32:esp32:esp32c5 \
  --build-property build.partitions=min_spiffs \
  --build-property upload.maximum_size=1966080 \
  --build-path ./build/${project} ./examples/${project}/${project}.ino 2>&1
  ```
  flash command
  ```
  filename=./build/demo-arp-reverse/demo-arp-reverse.ino.merged.bin
  port=/dev/cu.usbmodem1101
  sudo esptool --port ${port} erase_flash
  sudo esptool --port ${port} --baud 460800 write_flash 0x0 ${filename}
  ```
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
### ARP

#### ARP scan
* Performs an active ARP sweep of the entire local subnet (WiFi only).
* Sends one ARP request per host address, waits `timeout_ms` ms for replies,
* then harvests the lwIP ARP cache and fills the `results` array.
* Not supported when compiled with `ENABLE_LTE` (returns 0).
*
* @results    - caller-allocated array of `ARP_HOST` structs (size >= maxResults)
* @maxResults - maximum entries to fill, must be <= `ARP_SCAN_MAX_HOSTS` (default 32)
* @timeout_ms - milliseconds to wait for ARP replies (default 2000)
*
* Returns the number of live hosts written into `results`.
```
uint8_t arp_scan(ARP_HOST* results, uint8_t maxResults, uint32_t timeout_ms = 2000);
```
Example:
```cpp
ARP_HOST results[32];
uint8_t found = mRTOS.arp_scan(results, 32, 3000);
for (uint8_t i = 0; i < found; i++) {
    Serial.printf("%s -> %02X:%02X:%02X:%02X:%02X:%02X\n",
        results[i].ip.toString().c_str(),
        results[i].mac[0], results[i].mac[1], results[i].mac[2],
        results[i].mac[3], results[i].mac[4], results[i].mac[5]);
}
```

#### ARP scan ip
* Resolves the MAC address of a single IP address via ARP (WiFi only).
* Sends one ARP request, waits `timeout_ms` ms, then reads the result from
* the lwIP ARP cache.
* Not supported when compiled with `ENABLE_LTE` (returns false).
*
* @ip         - target IPv4 address
* @result     - caller-allocated `ARP_HOST` struct to fill on success
* @timeout_ms - milliseconds to wait for the ARP reply (default 2000)
*
* Returns `true` if the MAC was resolved, `false` otherwise.
```
bool arp_scan_ip(IPAddress ip, ARP_HOST* result, uint32_t timeout_ms = 2000);
```
Example:
```cpp
ARP_HOST host;
if (mRTOS.arp_scan_ip(IPAddress(192, 168, 1, 1), &host)) {
    Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        host.mac[0], host.mac[1], host.mac[2],
        host.mac[3], host.mac[4], host.mac[5]);
}
```

#### ARP scan with names
* ARP sweep + reverse DNS (PTR) lookup \u2013 the most effective way to find
* device names (including phones) on the local network (WiFi only).
* Performs a full ARP scan to discover every live IP, then queries the
* router's DNS for the hostname of each IP via `gethostbyaddr()`.
* The router's DHCP server usually registers device names, so phones,
* laptops, and IoT devices typically resolve to their configured names.
* Not supported when compiled with `ENABLE_LTE` (returns 0).
*
* @results        - caller-allocated array of `NETWORK_HOST` (size >= maxResults)
* @maxResults     - maximum entries to fill, must be <= `ARP_SCAN_MAX_HOSTS` (default 32)
* @arp_timeout_ms - milliseconds to wait for ARP replies (default 2000)
* @dns_timeout_ms - milliseconds per reverse DNS query (default 1000)
*
* Returns the number of live hosts written into `results`.
* `hostname` field is an empty string if the router did not resolve it.
```
uint8_t arp_scan_with_names(NETWORK_HOST* results, uint8_t maxResults,
                             uint32_t arp_timeout_ms = 2000,
                             uint32_t dns_timeout_ms = 1000);
```
Example:
```cpp
NETWORK_HOST devices[32];
uint8_t n = mRTOS.arp_scan_with_names(devices, 32);
for (uint8_t i = 0; i < n; i++) {
    Serial.printf("%-15s  %s\n",
        devices[i].ip.toString().c_str(),
        devices[i].hostname[0] ? devices[i].hostname : "(unresolved)");
}
```

`NETWORK_HOST` struct fields:
| Field      | Type        | Description                                      |
|------------|-------------|--------------------------------------------------|
| `ip`       | `IPAddress` | IPv4 address                                     |
| `mac`      | `uint8_t[6]`| MAC address                                      |
| `hostname` | `char[64]`  | Device name from router DNS; empty if unresolved |

---

### ARP scan ip with name

Sends a single ARP request for the given IP address and, if a reply is received, performs a reverse DNS (PTR) lookup to retrieve the device hostname. Useful when you already know the IP and just want its MAC and name.

```
* @ip             - target IPv4 address
* @result         - caller-allocated NETWORK_HOST to fill
* @arp_timeout_ms - milliseconds to wait for the ARP reply (default 2000)
* @dns_timeout_ms - milliseconds for the reverse DNS query (default 1000)
*
* Returns true if the MAC was resolved.
* hostname is an empty string if the router did not have a PTR record for that IP.
```
```cpp
bool arp_scan_ip_with_name(IPAddress ip, NETWORK_HOST* result,
                            uint32_t arp_timeout_ms = 2000,
                            uint32_t dns_timeout_ms = 1000);
```
Example:
```cpp
NETWORK_HOST host;
if (mRTOS.arp_scan_ip_with_name(IPAddress(192, 168, 1, 1), &host)) {
    Serial.printf("IP: %s  MAC: %02X:%02X:%02X:%02X:%02X:%02X  Name: %s\n",
        host.ip.toString().c_str(),
        host.mac[0], host.mac[1], host.mac[2],
        host.mac[3], host.mac[4], host.mac[5],
        host.hostname[0] ? host.hostname : "(unresolved)");
}
```
