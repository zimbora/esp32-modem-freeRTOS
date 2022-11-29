

#include "credentials.h"
#include "modem-freeRTOS.hpp"


#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

// HARDWARE
#define PWKEY 4

// CONTEXT
#define CLIENTID 0
#define CONTEXTID 1

// MQTT
#define MQTT_PROJECT "esp32/freeRTOS"
String mqtt_subscribe_topics[] = {
  "/fw/#",
  "/sysfile/#",
  "/config/#",
};


MODEMfreeRTOS mRTOS;

MQTT_MSG* msg;

void (*callback)(uint8_t clientID);
void mqttOnConnect(uint8_t clientID){
  Serial.println("mqtt is connected - sending first message");
  mRTOS.mqtt_pushMessage(clientID,"/status","online",2,true);
  return;
}

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

  callback = &mqttOnConnect;
  mRTOS.mqtt_setup(callback);

  for(;;){
    mRTOS.loop();
  }
}

void core(void *pvParameters);
void core(void *pvParameters){
  (void) pvParameters;

  for(;;){

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
    delay(100); // use delay to moderate concurrency access to queues
  }
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  // give highest priority to modem - otherwise data coming from modem can be lost
  xTaskCreatePinnedToCore(
      network_lte_task
      ,  "network_lte_task"   // A name just for humans
      ,  2048*4  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest. !! do not edit priority
      ,  NULL
      ,  1);

    xTaskCreatePinnedToCore(
        core
        ,  "core"   // A name just for humans
        ,  2048*4  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest. !! do not edit priority
        ,  NULL
        ,  1);


}

void loop() {
  // put your main code here, to run repeatedly:

}
