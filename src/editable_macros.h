
// uncomment to use WIFI, otherwise it will use BG95 LTE modem
//#define ENABLE_LTE

// --- MACROS FOR DEBUG ---
//#define DEBUG_WIFI

//#define DEBUG_HTTP
#define WARNING_HTTP_GW
#define DEBUG_HTTP_ERROR

//#define DEBUG_MQTT_MSG
// --- ----- ---

#ifndef   MAX_CONNECTIONS
#define   MAX_CONNECTIONS       	4
#endif

#ifndef   MAX_TCP_CONNECTIONS
#define   MAX_TCP_CONNECTIONS     2
#endif

#ifndef   MAX_MQTT_CONNECTIONS
#define   MAX_MQTT_CONNECTIONS    2
#endif

#ifndef   CONNECTION_BUFFER
#define   CONNECTION_BUFFER    		650 // bytes
#endif

#ifndef   CONNECTION_STATE
#define   CONNECTION_STATE   			10000 // millis
#endif

#ifndef   SMS_CHECK_INTERVAL
#define   SMS_CHECK_INTERVAL 			30000 // milli
#endif
