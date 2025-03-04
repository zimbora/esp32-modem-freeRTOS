

#include "credentials.h"
#include "modem-freeRTOS.hpp"

/*
* Edit editable_macros file in src path to change between WiFi and LTE
* This example connects to 2 mqtt brokers, subscribe some topics and sends a status message
* MQTT_PROJECT macro defines where the topics will be written or subscribed
* Configurations relative to WiFi/LTE and hosts must be defined in an external file "credentials.h"
*/

#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

// HARDWARE LTE
#define PWKEY 4

// CONTEXT LTE
#define CLIENTID 0
#define CONTEXTID 1

// MQTT
String mqtt_subscribe_topics[] = {
  "/status/#",
  "/#"
};


MODEMfreeRTOS mRTOS;

MQTT_MSG_RX* msg;

// callback for LTE
void (*callback)(uint8_t clientID);
void mqttOnConnect(uint8_t clientID){
  Serial.printf("mqtt %d is connected - sending first message \n",clientID);
  mRTOS.mqtt_pushMessage(clientID,"/status","online",2,true);

  mRTOS.mqtt_subscribeTopics(clientID);

}

// This function is called once MQTT WiFi connection 1 is established
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished(){
  Serial.println("mqtt 1 is connected - sending first message");
  mRTOS.mqtt_pushMessage(0,"/status","online",2,true);

  mRTOS.mqtt_subscribeTopics(0);
}

// This function is called once MQTT WiFi connection 2 is established
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished2(){
  Serial.println("mqtt 2 is connected - sending first message");
  mRTOS.mqtt_pushMessage(1,"/status","online",2,true);

  mRTOS.mqtt_subscribeTopics(1);
}

#ifdef ENABLE_LTE
void network_lte_task(void *pvParameters);
void network_lte_task(void *pvParameters){
  (void) pvParameters;

  mRTOS.init(SETTINGS_NB_COPS,GPRS,PWKEY); // initialize modem
  mRTOS.set_context(1,SETTINGS_NB_APN,SETTINGS_NB_USERNAME,SETTINGS_NB_PASSWORD);

  /* The following methods can be called at any time.
  * For changes to have effect mRTOS.mqtt_setup have to be called
  */
  mRTOS.mqtt_configure_connection(CLIENTID,CONTEXTID,MQTT_PROJECT,MQTT_UID,MQTT_HOST_1,1883,MQTT_USER_1,MQTT_PASSWORD_1);
  mRTOS.mqtt_set_will_topic(CLIENTID,MQTT_WILL_SUBTOPIC,MQTT_WILL_PAYLOAD);
  uint8_t i = 0;
  while(i<NUMITEMS(mqtt_subscribe_topics)){
    mRTOS.mqtt_add_subscribe_topic(CLIENTID,i,mqtt_subscribe_topics[i]);
    i++;
  }

  mRTOS.mqtt_configure_connection(1,CONTEXTID,MQTT_PROJECT,MQTT_UID,MQTT_HOST_2,1883,MQTT_USER_2,MQTT_PASSWORD_2);
  mRTOS.mqtt_set_will_topic(1,MQTT_WILL_SUBTOPIC,MQTT_WILL_PAYLOAD);
  i = 0;
  while(i<NUMITEMS(mqtt_subscribe_topics)){
    mRTOS.mqtt_add_subscribe_topic(1,i,mqtt_subscribe_topics[i]);
    i++;
  }

  callback = &mqttOnConnect;
  mRTOS.mqtt_setup(callback);

  for(;;){
    mRTOS.loop();
  }
}
#endif

void core(void *pvParameters);
void core(void *pvParameters){
  (void) pvParameters;

#ifndef ENABLE_LTE

  mRTOS.init(WIFI_SSID,WIFI_PASSWORD);
  String uid = MQTT_UID_PREFIX+mRTOS.macAddress();

  // Setup MQTT connection 1
  //mRTOS.wifi_configure_ap();
  Serial.println("wifi interface configured");
  mRTOS.mqtt_configure_connection(0,MQTT_PROJECT,uid.c_str(),MQTT_HOST_1,MQTT_PORT_1,MQTT_USER_1,MQTT_PASSWORD_1);
  mRTOS.mqtt_set_will_topic(CLIENTID,MQTT_WILL_SUBTOPIC,MQTT_WILL_PAYLOAD);
  Serial.println("mqtt client configured");

  for(uint8_t i=0;i<NUMITEMS(mqtt_subscribe_topics);i++){
    mRTOS.mqtt_add_subscribe_topic(0,i,mqtt_subscribe_topics[i]);
  }

  mRTOS.mqtt_wifi_setup(0,onConnectionEstablished);

  // Setup MQTT connection 2
  /*
  mRTOS.mqtt_configure_connection(1,MQTT_PROJECT,uid.c_str(),MQTT_HOST_2,MQTT_PORT_2,MQTT_USER_2,MQTT_PASSWORD_2);
  mRTOS.mqtt_set_will_topic(1,MQTT_WILL_SUBTOPIC,MQTT_WILL_PAYLOAD);
  Serial.println("mqtt client configured");

  for(uint8_t i=0;i<NUMITEMS(mqtt_subscribe_topics);i++){
    mRTOS.mqtt_add_subscribe_topic(1,i,mqtt_subscribe_topics[i]);
  }

  mRTOS.mqtt_wifi_setup(1,onConnectionEstablished2);
  */
#endif

#ifndef ENABLE_LTE
  Serial.println("wait for wifi connection..");
  while(!mRTOS.isWifiConnected()) delay(100);
  Serial.println("wifi is connected");
#else
  Serial.println("waiting for modem to register on network..");
  while(!mRTOS.isLTERegistered()) delay(100);
  Serial.println("modem is registered");
#endif

  uint32_t timeout = 0;
  while(1){

#ifndef ENABLE_LTE
    mRTOS.loop();
#endif

    if(timeout < millis()){
      Serial.println("send info");

      String heapFree = String(ESP.getFreeHeap() / 1024);
      Serial.println("heap free: " + heapFree + " KiB");
      String topic = "/heapFree";
      mRTOS.mqtt_pushMessage(CLIENTID,topic,heapFree,0,0);

      topic = "/uptime";
      String payload = String(millis());
      mRTOS.mqtt_pushMessage(CLIENTID,topic,payload,0,0);

      timeout = millis()+1000;
    }

    msg = mRTOS.mqtt_getNextMessage(msg);
    if(msg != NULL){
      Serial.println("new mqtt message received:");
      Serial.printf("<< [%d] ",msg->clientID);
      Serial.printf("%s ",msg->topic);
      Serial.printf("%s \n",msg->data);
      /*
      String topic = String(msg->topic);
      if(topic.endsWith("/fw/get")){

      }
      */
    }
    //delay(100); // use delay to moderate concurrency access to queues
  }
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

#ifdef ENABLE_LTE
  // give highest priority to modem - otherwise data coming from modem can be lost
  xTaskCreatePinnedToCore(
      network_lte_task
      ,  "network_lte_task"   // A name just for humans
      ,  2048*4  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest. !! do not edit priority
      ,  NULL
      ,  1
  );
#endif

  xTaskCreatePinnedToCore(
      core
      ,  "core"   // A name just for humans
      ,  2048*4  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest. !! do not edit priority
      ,  NULL
      ,  1
  );


}

uint32_t timeout = 0;
void loop() {

}
