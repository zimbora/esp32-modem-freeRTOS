
#ifndef MQTT_H
#define MQTT_H

#include "Arduino.h"
#include "modem-bgxx.hpp"

/*
#include "../../config/package.h"

#ifdef WIFI_MQTT_ENABLE
#include "../wifi/mqtt_wifi.h"
#endif

#include "../modem/network-gprs-mqtt.h"
*/

struct TCP_SETUP {
	char server[64];
	uint16_t port;
	uint8_t contextID; // context id 1-16
	uint8_t connectID; // connect id 0-11
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
    bool set_context(uint8_t contextID, String apn, String user, String pwd);
    void loop();

    void mqtt_configure_connection(uint8_t clientID, uint8_t contextID, String project, String uid, String host, uint16_t port, String user, String pwd);
    void mqtt_set_will_topic(uint8_t clientID, String topic, String payload);
    void mqtt_add_subscribe_topic(uint8_t clientID, uint8_t index, String topic);
    void mqtt_setup(void(*callback)());

    //bool mqtt_parse_msg(uint8_t clientID, String topic, String payload);
    /*
    int8_t mqtt_get_subscriptions_size();
    */
    bool mqtt_pushMessage(uint8_t clientID, const String& topic, const String& message, uint8_t qos, uint8_t retain);

    MQTT_MSG* mqtt_getMessageNextMessage(MQTT_MSG *pxRxedMessage);
  private:
    void mqtt_sendMessage();
    /*
    bool mqtt_wifi_connect_configured();
    bool mqtt_wifi_connect_default();
    */
};

void mqtt_enqueue_msg(uint8_t clientID, String topic, String payload);

#endif
