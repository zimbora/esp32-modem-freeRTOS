
#include "credentials.h"
#include "modem-freeRTOS.hpp"
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#ifdef ENABLE_LTE
#error "demo-mqtts only supports WiFi mode. Disable ENABLE_LTE in editable_macros.h."
#endif

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

uint32_t last_publish_at = 0;
uint32_t last_reconnect_attempt_at = 0;
bool tls_configured = false;
bool tls_configuration_failed = false;

char mqtt_client_name[64] = {0};
char mqtt_prefix[128] = {0};
char mqtt_subscribe_topic[160] = {0};
char mqtt_will_topic[160] = {0};
char mqtt_status_topic[160] = {0};
char mqtt_heap_free_topic[160] = {0};
char mqtt_uptime_topic[160] = {0};

const char* MQTTS_CA_CERT = R"EOF(
-----BEGIN CERTIFICATE-----
YOUR_CA_CERTIFICATE_HERE
-----END CERTIFICATE-----
)EOF";

const char* MQTTS_CLIENT_CERT = "";
const char* MQTTS_CLIENT_KEY = "";

bool has_ca_certificate(){
  return strlen(MQTTS_CA_CERT) > 0 && strstr(MQTTS_CA_CERT, "YOUR_CA_CERTIFICATE_HERE") == NULL;
}

bool build_topic(char* topic, size_t topic_size, const char* subtopic){
  int written = snprintf(topic, topic_size, "%s/%s", mqtt_prefix, subtopic);
  return written > 0 && (size_t)written < topic_size;
}

bool configure_topics(){
  String mac_address = mRTOS.macAddress();

  int written = snprintf(mqtt_client_name, sizeof(mqtt_client_name), "%s%s", MQTTS_UID_PREFIX, mac_address.c_str());
  if(written <= 0 || (size_t)written >= sizeof(mqtt_client_name)){
    return false;
  }

  written = snprintf(mqtt_prefix, sizeof(mqtt_prefix), "%s/%s", MQTTS_PROJECT, mqtt_client_name);
  if(written <= 0 || (size_t)written >= sizeof(mqtt_prefix)){
    return false;
  }

  return
    build_topic(mqtt_subscribe_topic, sizeof(mqtt_subscribe_topic), "#") &&
    build_topic(mqtt_will_topic, sizeof(mqtt_will_topic), MQTTS_WILL_SUBTOPIC) &&
    build_topic(mqtt_status_topic, sizeof(mqtt_status_topic), "status") &&
    build_topic(mqtt_heap_free_topic, sizeof(mqtt_heap_free_topic), "heapFreeKiB") &&
    build_topic(mqtt_uptime_topic, sizeof(mqtt_uptime_topic), "uptime");
}

void mqtt_callback(char* topic, byte* payload, unsigned int length){
  Serial.printf("<< %s ", topic);
  for(unsigned int i = 0; i < length; i++){
    uint8_t current = payload[i];
    if(current >= 32 && current <= 126){
      Serial.print((char)current);
    }else{
      Serial.printf("\\x%02X", current);
    }
  }
  Serial.println();
}

bool mqtt_subscribe_topics(){
  if(!mqttClient.subscribe(mqtt_subscribe_topic)){
    Serial.printf("failed to subscribe to %s\n", mqtt_subscribe_topic);
    return false;
  }

  Serial.printf("subscribed to %s\n", mqtt_subscribe_topic);
  return true;
}

bool mqtt_connect(){
  bool connected = false;
  if(strlen(MQTTS_USER) == 0){
    connected = mqttClient.connect(
      mqtt_client_name,
      mqtt_will_topic,
      1,
      true,
      MQTTS_WILL_PAYLOAD
    );
  }else{
    connected = mqttClient.connect(
      mqtt_client_name,
      MQTTS_USER,
      MQTTS_PASSWORD,
      mqtt_will_topic,
      1,
      true,
      MQTTS_WILL_PAYLOAD
    );
  }

  if(!connected){
    Serial.printf("mqtt connection failed, rc=%d\n", mqttClient.state());
    return false;
  }

  Serial.println("mqtts is connected - sending first message");
  if(!mqtt_subscribe_topics() || !mqttClient.publish(mqtt_status_topic, "online", true)){
    Serial.println("failed to initialize mqtt topics");
    mqttClient.disconnect();
    return false;
  }

  last_reconnect_attempt_at = 0;
  return true;
}

bool configure_tls(){
#if defined(MQTTS_TLS_INSECURE) && MQTTS_TLS_INSECURE
  secureClient.setInsecure();
  Serial.println("using insecure tls mode");
#else
  if(!has_ca_certificate()){
    Serial.println("replace MQTTS_CA_CERT in demo-mqtts.ino or enable MQTTS_TLS_INSECURE for testing");
    tls_configuration_failed = true;
    return false;
  }

  bool has_client_cert = strlen(MQTTS_CLIENT_CERT) > 0;
  bool has_client_key = strlen(MQTTS_CLIENT_KEY) > 0;
  if(has_client_cert != has_client_key){
    Serial.println("set both MQTTS_CLIENT_CERT and MQTTS_CLIENT_KEY or leave both empty");
    tls_configuration_failed = true;
    return false;
  }

  secureClient.setCACert(MQTTS_CA_CERT);

  if(has_client_cert){
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
  return true;
}

void setup() {
  Serial.begin(115200);
  mRTOS.init(WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
  mRTOS.loop();

  if(!mRTOS.isWifiConnected()){
    delay(100);
    return;
  }

  if(tls_configuration_failed){
    delay(1000);
    return;
  }

  if(!tls_configured){
    if(!configure_topics()){
      Serial.println("failed to configure mqtt topic buffers");
      tls_configuration_failed = true;
      delay(1000);
      return;
    }

    if(!configure_tls()){
      delay(1000);
      return;
    }
  }

  if(!mqttClient.connected()){
    uint32_t now = millis();
    if(last_reconnect_attempt_at == 0 || (uint32_t)(now - last_reconnect_attempt_at) >= 5000){
      last_reconnect_attempt_at = now;
      mqtt_connect();
    }
    delay(100);
    return;
  }

  mqttClient.loop();

  uint32_t now = millis();
  if(last_publish_at == 0 || (uint32_t)(now - last_publish_at) >= 1000){
    char heap_free[16];
    char uptime[16];
    snprintf(heap_free, sizeof(heap_free), "%lu", (unsigned long)(ESP.getFreeHeap() / 1024));
    snprintf(uptime, sizeof(uptime), "%lu", (unsigned long)(now / 1000UL));

    bool heap_published = mqttClient.publish(mqtt_heap_free_topic, heap_free, true);
    bool uptime_published = mqttClient.publish(mqtt_uptime_topic, uptime, true);
    if(heap_published && uptime_published){
      last_publish_at = now;
    }else{
      Serial.println("failed to publish telemetry");
    }
  }
}
