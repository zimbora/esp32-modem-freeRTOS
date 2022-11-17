
#ifndef MQTT_H
#define MQTT_H

#include "Arduino.h"
#include "modem-bgxx.hpp"

struct HTTP_HEADER_MSG
{
  uint8_t clientID;
  uint16_t body_len;
  char md5[16];
  String http_response;
};

struct HTTP_BODY_MSG
{
  uint8_t clientID;
  uint16_t data_len;
  char data[CONNECTION_BUFFER];
};

struct HTTP_REQUEST
{
  String host;
  String path;
  String method;
  uint8_t contextID;
  uint8_t clientID;
};

struct TCP_MSG
{
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

struct MQTT_MSG
{
  char topic[100];
  char data[255];
  uint8_t qos;
  uint8_t retain;
  uint8_t clientID;
};

struct MQTT_SETUP
{
  uint8_t contextID; // index for TCP tcp[] 1-16, limited to MAX_CONNECTIONS
  String nick;
  String prefix;
  String host;
  uint16_t port;
  String user;
  String pwd;
  uint8_t msg_id;
  uint8_t clientID; // client id 0-5 (limited to MAX_MQTT_CONNECTIONS)
  String will_topic;
  String will_payload;
  String subscribe_topics[10];
};

class MODEMfreeRTOS{

  public:
    // public
    void init(uint16_t cops, uint8_t mode, uint8_t pwkey);
    void loop();
    bool set_context(uint8_t contextID, String apn, String user, String pwd);

    void tcp_configure_connection(uint8_t clientID, uint8_t contextID, String host, uint16_t port);
    void tcp_setup(void(*callback1)(uint8_t clientID),void(*callback2)(uint8_t clientID));
    TCP_MSG* tcp_getNextMessage(TCP_MSG *pxRxedMessage);
    bool tcp_pushMessage(uint8_t clientID, const char* data, uint16_t len);

    bool http_pushMessage(uint8_t contextID, uint8_t clientID, String host, String path, String method);
		HTTP_HEADER_MSG* http_header_getNextMessage(HTTP_HEADER_MSG *pxRxedMessage);
		HTTP_BODY_MSG* http_body_getNextMessage(HTTP_BODY_MSG *pxRxedMessage);

    void mqtt_configure_connection(uint8_t clientID, uint8_t contextID, String project, String uid, String host, uint16_t port, String user, String pwd);
    void mqtt_set_will_topic(uint8_t clientID, String topic, String payload);
    void mqtt_add_subscribe_topic(uint8_t clientID, uint8_t index, String topic);
    void mqtt_setup(void(*callback)());
    bool mqtt_pushMessage(uint8_t clientID, const String& topic, const String& message, uint8_t qos, uint8_t retain);
    MQTT_MSG* mqtt_getNextMessage(MQTT_MSG *pxRxedMessage);

    //bool mqtt_parse_msg(uint8_t clientID, String topic, String payload);
    /*
    int8_t mqtt_get_subscriptions_size();
    */

  private:
    void mqtt_sendMessage();

    void tcp_sendMessage();
    bool tcp_checkMessages();
    void tcp_enqueue_msg(uint8_t clientID, const char* data, uint16_t data_len);

    bool http_enqueue_header_msg(uint8_t clientID, uint16_t body_len, const char* md5, String http_response);
    bool http_enqueue_body_msg(uint8_t clientID, char* data, uint16_t data_len);
		bool http_queue_body_has_space();
    void http_execute_requests();
		bool http_get_request(uint8_t clientID, uint8_t contextID, String host, String path);
    /*
    bool mqtt_wifi_connect_configured();
    bool mqtt_wifi_connect_default();
    */
};

void mqtt_enqueue_msg(uint8_t clientID, String topic, String payload);

#endif
