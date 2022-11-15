

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
void (*callback)();
void mqttOnConnect(){
  //mqtt_enqueue_msg(clientID,topic,payload);
  Serial.println("push messages to queue");
  mRTOS.mqtt_pushMessage(CLIENTID,"/status","online",2,true);
  return;
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);


  mRTOS.init(SETTINGS_NB_COPS,AUTO,PWKEY); // initialize modem
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
}

MQTT_MSG* msg;

void loop() {
  // put your main code here, to run repeatedly:

  mRTOS.loop();

  msg = mRTOS.mqtt_getMessageNextMessage(msg);
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

}
