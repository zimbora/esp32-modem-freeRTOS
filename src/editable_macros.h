
// uncomment to use WIFI, otherwise it will use BG95 LTE modem
//#define ENABLE_LTE

// --- MACROS FOR DEBUG ---
//#define DEBUG_WIFI

//#define DEBUG_HTTP
#define WARNING_HTTP_GW
#define DEBUG_HTTP_ERROR
//#define DEBUG_ARP_SCAN

//#define DEBUG_MQTT_MSG
// --- ----- ---

#ifndef   MAX_CONNECTIONS
#define   MAX_CONNECTIONS       	    4
#endif

#ifndef   MAX_TCP_CONNECTIONS
#define   MAX_TCP_CONNECTIONS           2
#endif

#ifndef   MAX_MQTT_CONNECTIONS
#define   MAX_MQTT_CONNECTIONS          2
#endif

#ifndef   CONNECTION_BUFFER
#define   CONNECTION_BUFFER    	    	650 // bytes - HTTP body buffer size for both request and response
#endif

#ifndef   CONNECTION_STATE
#define   CONNECTION_STATE   			10000 // millis
#endif

#ifndef   SMS_CHECK_INTERVAL
#define   SMS_CHECK_INTERVAL 			30000 // millis
#endif

// TCP BUFFERS SIZE

#define TCP_RX_QUEUE_SIZE 2
#define TCP_TX_QUEUE_SIZE 2

// HTTP BUFFERS SIZE

#define HTTP_RX_QUEUE_SIZE 1 // !! do not change it
#define HTTP_TX_QUEUE_SIZE 1 // !! do not change it

// MQTT BUFFERS SIZE

#define MQTT_RX_QUEUE_SIZE 10
#define MQTT_TX_QUEUE_SIZE 20

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN                  75
#endif
#ifndef MQTT_TX_PAYLOAD_LEN
#define MQTT_TX_PAYLOAD_LEN             512
#endif
#ifndef MQTT_RX_PAYLOAD_LEN
#define MQTT_RX_PAYLOAD_LEN             1024
#endif

// Maximum number of hosts returned by arp_scan()
#ifndef ARP_SCAN_MAX_HOSTS
#define ARP_SCAN_MAX_HOSTS              32
#endif