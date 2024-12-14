
#ifndef MODEM_FREERTOS_H
#define MODEM_FREERTOS_H

#include "Arduino.h"
#include "WiFi.h"
#include "editable_macros.h"

#ifdef ENABLE_LTE
  #include "esp32-BG95.hpp"
#else
  #include "EspMQTTClient.h"
  #include <HTTPClient.h>
  #include <WiFiClientSecure.h>
  #include <TimeLib.h>
#endif

#ifndef CONNECTION_BUFFER
#define CONNECTION_BUFFER 650
#endif

// MQTT BUFFERS SIZE
#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 75
#endif
#ifndef MQTT_TX_PAYLOAD_LEN
#define MQTT_TX_PAYLOAD_LEN 255
#endif
#ifndef MQTT_RX_PAYLOAD_LEN
#define MQTT_RX_PAYLOAD_LEN 2048
#endif

struct HTTP_HEADER_MSG {
  uint8_t clientID;
  uint32_t body_len;
  String md5;
  String http_response;
};

struct HTTP_BODY_MSG {
  uint8_t clientID;
  uint16_t data_len;
  char data[CONNECTION_BUFFER];
};

struct HTTP_REQUEST_ {
  String protocol; // HTTP|HTTPS
  String host;
  String path;
  String method;
  String header_key;
  String header_value;
  String body;
  bool json;
  uint8_t contextID;
  uint8_t clientID;
  uint8_t sslClientID;
};

struct TCP_MSG {
  uint8_t clientID;
  uint16_t data_len;
  char data[CONNECTION_BUFFER];
};

struct TCP_SETUP {
	String host;
	uint16_t port;
	uint8_t contextID; // context id 1-16
	uint8_t socket_state;
	bool active;
	bool connected;
};

struct MQTT_MSG_TX {
  char topic[MQTT_TOPIC_LEN];
  char data[MQTT_TX_PAYLOAD_LEN];
  uint8_t qos;
  uint8_t retain;
  uint8_t clientID;
};

struct MQTT_MSG_RX {
  char topic[MQTT_TOPIC_LEN];
  char data[MQTT_RX_PAYLOAD_LEN];
  uint8_t qos;
  uint8_t retain;
  uint8_t clientID;
};

struct MQTT_SETUP {
  uint8_t contextID; // index for TCP tcp[] 1-16, limited to MAX_CONNECTIONS
  bool active;
  String nick;
  String prefix;
  String host;
  uint16_t port;
  String user;
  String pwd;
  uint8_t msg_id;
  String will_topic;
  String will_payload;
  String subscribe_topics[10];
};

class MODEMfreeRTOS{

  public:

    // public
    void init(const char* ssid, const char* password);
    void init(uint16_t cops, uint8_t mode, uint8_t pwkey);
    void loop();

    String get_manufacturer_identification();
    String get_model_identification();
    String get_firmware_version();

    void wifiReconnect(const char* ssid, const char* password);
    void wifi_configure_ap(const char* wifi_ssid, const char* wifi_pwd);
    static bool isWifiConnected();

    bool set_context(uint8_t contextID, String apn, String user, String pwd);
    bool set_ssl(uint8_t contextID);

    void tcp_configure_connection(uint8_t clientID, uint8_t contextID, String host, uint16_t port);
    void tcp_setup(void(*callback1)(uint8_t clientID),void(*callback2)(uint8_t clientID));
    TCP_MSG* tcp_getNextMessage(TCP_MSG *pxRxedMessage);
    bool tcp_pushMessage(uint8_t clientID, const char* data, uint16_t len);

    bool http_pushMessage(uint8_t contextID, uint8_t clientID, String host, String path, String method, String header_key = "", String header_value = "", String body = "", bool json = false);
    bool http_pushMessage(String host, String path, String method, String header_key = "", String header_value = "", String body = "", bool json = false);
    bool https_pushMessage(uint8_t contextID, uint8_t clientID, uint8_t sslClientID, String host, String path, String method, String header_key = "", String header_value = "", String body = "", bool json = false);
    bool https_pushMessage(String host, String path, String method, String header_key = "", String header_value = "", String body = "", bool json = false);
		HTTP_HEADER_MSG* http_header_getNextMessage(HTTP_HEADER_MSG *pxRxedMessage);
		HTTP_BODY_MSG* http_body_getNextMessage(HTTP_BODY_MSG *pxRxedMessage);

    void mqtt_configure_connection(uint8_t clientID, uint8_t contextID, String project, String uid, String host, uint16_t port, String user, String pwd);
    void mqtt_configure_connection(uint8_t clientID, const char* project, const char* uid, const char* host, uint16_t port, const char* user, const char* pwd);
    void mqtt_set_will_topic(uint8_t clientID, String topic, String payload);
    void mqtt_add_subscribe_topic(uint8_t clientID, uint8_t index, String topic);
    void mqtt_setup(void(*callback)(uint8_t clientID));
    void mqtt_wifi_setup(uint8_t cliendID, void(*callback)());
    void mqtt_subscribeTopics(uint8_t clientID);
    bool mqtt_isConnected(uint8_t clientID);
    bool mqtt_pushMessage(uint8_t clientID, const String& topic, const String& message, uint8_t qos, uint8_t retain);
    MQTT_MSG_RX* mqtt_getNextMessage(MQTT_MSG_RX *pxRxedMessage);

    void log_modem_status();
    int16_t get_rssi();
    String get_technology();
    String macAddress();
    bool isLTERegistered();
    uint32_t get_tz();
    void update_clock_sys();
    //bool mqtt_parse_msg(uint8_t clientID, String topic, String payload);
    /*
    int8_t mqtt_get_subscriptions_size();
    */

  private:

    #ifndef ENABLE_LTE
    static void WiFiEvent(WiFiEvent_t event);
    #endif

    void lte_loop();

    void    tcp_sendMessage();
    void    tcp_checkMessages();
    void    tcp_enqueue_msg(uint8_t clientID, const char* data, uint16_t data_len);

    bool    http_enqueue_header_msg(uint8_t clientID, uint32_t body_len, const char* md5, String  http_response);
    bool    http_enqueue_body_msg(uint8_t clientID, char* data, uint16_t data_len);
		bool    http_queue_body_has_space();
    void    http_execute_requests();
		bool    http_get_request(uint8_t clientID, uint8_t contextID, String host, String path);
    bool    https_request(uint8_t contextID, uint8_t clientID, uint8_t sslClientID, String host, String path, String token, String body, String method, bool json);

    bool    wifi_HTTP_REQUEST_(String host, String path, String method, String header_token, String header_value, String body, bool json);
    bool    wifi_https_request(String host, String path, String method, String header_token, String header_value, String body, bool json);

    String  wifi_IP();
    int16_t wifi_rssi();
    String  wifi_mac_address();

    #ifndef ENABLE_LTE
    void    sync_clock();
    #endif

    void    mqtt_sendMessage();

    /*
    bool mqtt_wifi_connect_configured();
    bool mqtt_wifi_connect_default();
    */
};


void mqtt_enqueue_msg(uint8_t clientID, String topic, String payload);

#endif
