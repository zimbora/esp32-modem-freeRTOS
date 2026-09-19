
#include "credentials.h"
#include "modem-freeRTOS.hpp"
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

/*
* Edit editable_macros file in src path to change between WiFi and LTE
* This example uses MODEMfreeRTOS to manage WiFi connectivity and shows how to
* connect to an MQTT broker over TLS using either a hardcoded CA certificate or
* an insecure TLS connection.
* Configurations relative to WiFi and broker must be defined in an external file
* - "credentials.h"
*/

MODEMfreeRTOS mRTOS;
WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

String mqtt_prefix = "";
uint32_t publish_timeout = 0;
uint32_t reconnect_timeout = 0;
bool tls_configured = false;

const char* MQTTS_CA_CERT = R"EOF(
-----BEGIN CERTIFICATE-----
YOUR_CA_CERTIFICATE_HERE
-----END CERTIFICATE-----
)EOF";

const char* MQTTS_CLIENT_CERT = "";
const char* MQTTS_CLIENT_KEY = "";

void mqtt_callback(char* topic, byte* payload, unsigned int length){
  Serial.printf("<< %s ", topic);
  for(unsigned int i = 0; i < length; i++){
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void mqtt_subscribe_topics(){
  String topic = mqtt_prefix + "/#";
  mqttClient.subscribe(topic.c_str());
  Serial.println("subscribed to " + topic);
}

bool mqtt_connect(){
  String client_name = String(MQTTS_UID_PREFIX) + mRTOS.macAddress();
  String will_topic = mqtt_prefix + "/" + String(MQTTS_WILL_SUBTOPIC);

  bool connected = false;
  if(strlen(MQTTS_USER) == 0){
    connected = mqttClient.connect(
      client_name.c_str(),
      will_topic.c_str(),
      1,
      true,
      MQTTS_WILL_PAYLOAD
    );
  }else{
    connected = mqttClient.connect(
      client_name.c_str(),
      MQTTS_USER,
      MQTTS_PASSWORD,
      will_topic.c_str(),
      1,
      true,
      MQTTS_WILL_PAYLOAD
    );
  }

  if(!connected){
    Serial.printf("mqtt connection failed, rc=%d\n", mqttClient.state());
    reconnect_timeout = millis() + 5000;
    return false;
  }

  Serial.println("mqtts is connected - sending first message");
  mqtt_subscribe_topics();
  mqttClient.publish((mqtt_prefix + "/status").c_str(), "online", true);
  return true;
}

void configure_tls(){
  secureClient.setHandshakeTimeout(30);

#if defined(MQTTS_TLS_INSECURE) && MQTTS_TLS_INSECURE
  secureClient.setInsecure();
  Serial.println("using insecure tls mode");
#else
  secureClient.setCACert(MQTTS_CA_CERT);

  if(strlen(MQTTS_CLIENT_CERT) > 0 && strlen(MQTTS_CLIENT_KEY) > 0){
    secureClient.setCertificate(MQTTS_CLIENT_CERT);
    secureClient.setPrivateKey(MQTTS_CLIENT_KEY);
    Serial.println("using ca certificate and client certificate");
  }else{
    Serial.println("using ca certificate");
  }
#endif

  mqttClient.setServer(MQTTS_HOST, MQTTS_PORT);
  mqttClient.setCallback(mqtt_callback);
  tls_configured = true;
}

void setup() {
  Serial.begin(115200);

#ifdef ENABLE_LTE
  Serial.println("demo-mqtts currently supports WiFi mode only. Disable ENABLE_LTE to use this example.");
#else
  mRTOS.init(WIFI_SSID, WIFI_PASSWORD);
#endif
}

void loop() {
#ifdef ENABLE_LTE
  delay(1000);
  return;
#else
  mRTOS.loop();

  if(!mRTOS.isWifiConnected()){
    delay(100);
    return;
  }

  if(!tls_configured){
    mqtt_prefix = String(MQTTS_PROJECT) + "/" + String(MQTTS_UID_PREFIX) + mRTOS.macAddress();
    configure_tls();
  }

  if(!mqttClient.connected()){
    if(reconnect_timeout < millis()){
      mqtt_connect();
    }
    delay(100);
    return;
  }

  mqttClient.loop();

  if(publish_timeout < millis()){
    String heap_free = String(ESP.getFreeHeap() / 1024);
    String uptime = String(millis());
    mqttClient.publish((mqtt_prefix + "/heapFree").c_str(), heap_free.c_str(), true);
    mqttClient.publish((mqtt_prefix + "/uptime").c_str(), uptime.c_str(), true);
    publish_timeout = millis() + 1000;
  }
#endif
}
