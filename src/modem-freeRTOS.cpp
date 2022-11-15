

#include "modem-freeRTOS.hpp"

#define MQTT_RX_QUEUE_SIZE 10
#define MQTT_TX_QUEUE_SIZE 10

#define HTTP_RX_QUEUE_SIZE 1
#define HTTP_TX_QUEUE_SIZE 1


// MQTT
QueueHandle_t mqttRxQueue;
QueueHandle_t mqttTxQueue;
MQTT_MSG rcv_mqtt_msg[MQTT_RX_QUEUE_SIZE];
MQTT_MSG tx_mqtt_msg[MQTT_TX_QUEUE_SIZE];
SemaphoreHandle_t mqttTxQueueMutex;
MQTT_SETUP mqtt[MAX_MQTT_CONNECTIONS];

// HTTP


MODEMBGXX modem;

void (*mqttOnConnect)();
bool (*mqtt_callback)(uint8_t,String,String);
bool mqtt_parse_msg(uint8_t clientID, String topic, String payload){
  mqtt_enqueue_msg(clientID,topic,payload);
  return true;
}


void MODEMfreeRTOS::init(uint16_t cops, uint8_t mode, uint8_t pwkey){

  mqttRxQueue = xQueueCreate( MQTT_RX_QUEUE_SIZE, sizeof( struct MQTT_MSG * ) );
  mqttTxQueue = xQueueCreate( MQTT_TX_QUEUE_SIZE, sizeof( struct MQTT_MSG * ) );

  /*
  httpTxQueue = xQueueCreate( HTTP_RX_QUEUE_SIZE, sizeof( struct HTTP_MSG * ) );
  httpRxQueue = xQueueCreate( HTTP_TX_QUEUE_SIZE, sizeof( struct HTTP_MSG * ) );
  */

  mqttTxQueueMutex = xSemaphoreCreateMutex();

  for(uint8_t i = 0; i<MAX_MQTT_CONNECTIONS; i++){
    mqtt[i].contextID = 0;
    mqtt[i].msg_id = 0;
  }

  modem.init_port(115200,SERIAL_8N1);
  modem.init(cops,mode,pwkey);

  // init contexts
}

bool MODEMfreeRTOS::set_context(uint8_t contextID, String apn, String user, String pwd){
  if(contextID == 0 || contextID > MAX_CONNECTIONS)
    return false;

  return modem.setup(contextID,apn,user,pwd);
}

/*
* changes mqtt connection parameters
*
* @clientID 0-5, limited to MAX_MQTT_CONNECTIONS defined in bgxx library
* @contextID 1-16, limited to MAX_CONNECTIONS defined in bgxx library
* @project - string to create prefix of topic - :project/:uid/...
* @host - IP or DNS of server
* @user - credential username
* @pwd - credential password
*/
void MODEMfreeRTOS::mqtt_configure_connection(uint8_t clientID, uint8_t contextID, String project, String uid, String host, uint16_t port, String user, String pwd){
  if(clientID > MAX_MQTT_CONNECTIONS)
    return;
  mqtt[clientID].contextID = contextID;
  mqtt[clientID].nick = uid;
  mqtt[clientID].prefix = project+"/"+uid;
  mqtt[clientID].host = host;
  mqtt[clientID].port = port;
  mqtt[clientID].user = user;
  mqtt[clientID].pwd = pwd;
}

/*
* mqtt set will
*
* @clientID 0-5, limited to MAX_MQTT_CONNECTIONS defined in bgxx library
* @topic - without prefix
* @payload - data to be written on topic
*/
void MODEMfreeRTOS::mqtt_set_will_topic(uint8_t clientID, String topic, String payload){

  mqtt[clientID].will_topic = topic;
  mqtt[clientID].will_payload = payload;
}

/*
* mqtt adds topic to be subscribed, limited to 10 topics
*
* @clientID 0-5, limited to MAX_MQTT_CONNECTIONS defined in bgxx library
* @index 0-9
* @topic - without prefix
*/
void MODEMfreeRTOS::mqtt_add_subscribe_topic(uint8_t clientID, uint8_t index, String topic){

  if(index >= 10)
    return;

  mqtt[clientID].subscribe_topics[index] = topic;
}

void MODEMfreeRTOS::mqtt_setup(void(*callback)()){

  mqttOnConnect = callback;
  mqtt_callback = &mqtt_parse_msg;
  modem.MQTT_init(mqtt_callback);

  for(uint8_t i = 0; i<MAX_MQTT_CONNECTIONS; i++){
    if(mqtt[i].contextID != 0)
      modem.MQTT_setup(i,mqtt[i].contextID,mqtt[i].prefix+mqtt[i].will_topic,mqtt[i].will_payload);
  }
}

void MODEMfreeRTOS::loop(){
  /*
  // if priority is wifi
  if(!mqtt_connected()){
      mqtt_connect();
  }else{
    #ifdef WIFI_MQTT_ENABLE
      if(mqtt_wifi_connected())
        mqtt_wifi_loop();
    #endif
  }
  */

  // LTE
  if(modem.loop(5000)){ // state was updated
    for(uint8_t i=0; i<MAX_MQTT_CONNECTIONS; i++){
      if(mqtt[i].contextID == 0)
        continue;

      if(modem.has_context(mqtt[i].contextID)){
        if(!modem.MQTT_connected(mqtt[i].clientID)){
          if(modem.MQTT_connect(mqtt[i].clientID,mqtt[i].nick.c_str(),mqtt[i].user.c_str(),mqtt[i].pwd.c_str(),mqtt[i].host.c_str(),mqtt[i].port)){
            if(mqttOnConnect != NULL)
              mqttOnConnect();
            for(uint8_t j=0; j<10; j++){
              if(mqtt[i].subscribe_topics[j] == "")
                continue;
              String topic = mqtt[i].prefix + mqtt[i].subscribe_topics[j];
              modem.MQTT_subscribeTopic(mqtt[i].clientID,++mqtt[i].msg_id,topic,2);
            }
          }
        }
      }else{
        modem.open_pdp_context(mqtt[i].contextID);
      }
    }

    modem.log_status();
  }

  mqtt_sendMessage();

  /*
  if(!state_connected && mqtt_connected()){
    state_connected = true;
    mqtt_pushMessage("/status","online",1,true);
    mqtt_pushMessage("/fw","{\"version\":\""+String(PACKAGE_VERSION)+"\",\"model\":"+String(PACKAGE_MODEL)+",\"hash\":\""+String(settings.fw.hash)+"\"}",1,false);
  }else if(state_connected && !mqtt_connected()){
    state_connected = false;
  }

  mqtt_sendMessage();
  */
}


void mqtt_enqueue_msg(uint8_t clientID, String topic, String payload){

  //#ifdef DEBUG_MQTT_GW
  Serial.println("[mqtt] << ["+String(clientID)+"] "+topic + " : " + payload);
  //#endif
  struct MQTT_MSG *pxMessage;

  if( mqttRxQueue != 0 && uxQueueSpacesAvailable(mqttRxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     pxMessage = &rcv_mqtt_msg[uxQueueMessagesWaiting(mqttRxQueue)];
     memset(pxMessage->topic,0,100);
     memset(pxMessage->data,0,255);

     memcpy(pxMessage->topic,topic.c_str(),topic.length());
     memcpy(pxMessage->data,payload.c_str(),payload.length());
     pxMessage->clientID = clientID;
     xQueueGenericSend( mqttRxQueue, ( void * ) &pxMessage, ( TickType_t ) 0, queueSEND_TO_BACK );
  }else{
    #ifdef WARNING_MQTT_GW
    log("mqtt rx no space available");
    #endif
  }
}

MQTT_MSG* MODEMfreeRTOS::mqtt_getMessageNextMessage(MQTT_MSG *pxRxedMessage){

  if( mqttRxQueue != 0 && uxQueueMessagesWaiting(mqttRxQueue) > 0){
     // get a message on the created queue.  Block for 10 ticks if a
     // message is not immediately available.
     //if( xQueueReceive( mqttRxQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
     if( xQueueReceive( mqttRxQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
         // pcRxedMessage now points to the struct AMessage variable posted
         // by vATask, but the item still remains on the queue.
         return pxRxedMessage;
     }
  }
  return NULL;
}

/*
void mqtt_connect(){

  #ifdef WIFI_MQTT_ENABLE
    // if priority is wifi
    if(!network_wifi_isConnected())
      return;

    Serial.println("[mqtt wifi] try to connect");
    void (*callback)(String, String, uint16_t);
    callback = &mqtt_enqueue_msg;
    mqtt_wifi_connect(callback);
  #endif

}

bool mqtt_connected(){

  bool wifi_mqtt_connected = false;
  bool modem_mqtt_connected = false;
  #ifdef WIFI_MQTT_ENABLE
    wifi_mqtt_connected = mqtt_wifi_connected();
  #endif
  #ifdef ENABLE_LTE
    modem_mqtt_connected = network_gprs_mqtt_connected();
  #endif
  return wifi_mqtt_connected|modem_mqtt_connected;
}

void mqtt_disconnect(){

  #ifdef WIFI_MQTT_ENABLE
    if(mqtt_wifi_connected())
      mqtt_wifi_disconnect();
  #endif

  if(network_gprs_mqtt_connected())
    network_gprs_mqtt_disconnect();

}

int8_t mqtt_get_subscriptions_size(){

  return (int8_t)NUMITEMS(mqtt_subscribe_topics);

}
*/

/*
* send received message to Queue
*/
bool MODEMfreeRTOS::mqtt_pushMessage(uint8_t clientID, const String& topic, const String& message, uint8_t qos, uint8_t retain) {

  if(clientID >= MAX_MQTT_CONNECTIONS)
    return false;
  /*
  if(qos == 0 && !mqtt_connected())
    return false;
  */
  struct MQTT_MSG *pxMessage;

  if( mqttTxQueue != 0 && uxQueueSpacesAvailable(mqttTxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     if(!xSemaphoreTake( mqttTxQueueMutex, 2000)){
       xSemaphoreGive(mqttTxQueueMutex);
       return false;
     }

     pxMessage = &tx_mqtt_msg[uxQueueMessagesWaiting(mqttTxQueue)];
     memset(pxMessage->topic,0,100);
     memset(pxMessage->data,0,255);

     String topic_ = mqtt[clientID].prefix+topic;
     memcpy(pxMessage->topic,topic_.c_str(),topic_.length());
     memcpy(pxMessage->data,message.c_str(),message.length());
     pxMessage->qos = qos;
     pxMessage->retain = retain;

     bool res = xQueueSendToBack( mqttTxQueue, ( void * ) &pxMessage, ( TickType_t ) 0 ) == true;
     xSemaphoreGive(mqttTxQueueMutex);
     return res;
  }

  return false;

}

/*
* call network interface to send mqtt message
*/

void MODEMfreeRTOS::mqtt_sendMessage(){

  struct MQTT_MSG *pxMessage;

  if(!xSemaphoreTake( mqttTxQueueMutex, 2000)){
    xSemaphoreGive(mqttTxQueueMutex);
    return;
  }

  while( mqttTxQueue != 0 && uxQueueMessagesWaiting(mqttTxQueue) > 0){
    // get a message on the created queue.  Block for 10 ticks if a
    // message is not immediately available.

    //if( xQueueReceive( mqttTxQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
    if( xQueueReceive( mqttTxQueue, &( pxMessage ), ( TickType_t ) 10 ) ){
      // pcRxedMessage now points to the struct AMessage variable posted
      // by vATask, but the item still remains on the queue.
      String topic = String(pxMessage->topic);
      String data = String(pxMessage->data);
      uint8_t qos = pxMessage->qos;
      uint8_t retain = pxMessage->retain;
      uint8_t clientID = pxMessage->clientID;
      if(clientID >= MAX_MQTT_CONNECTIONS){
        Serial.println("invalid mqtt clientID");
        return;
      }


      if(!modem.MQTT_connected(clientID)){
        mqtt_pushMessage(clientID,topic,data,qos,retain);
        return;
      }


      //log("[mqtt] >> "+topic + " : " + data);
      modem.MQTT_publish(clientID,++mqtt[clientID].msg_id,qos,retain,topic,data);
      /*
      #ifdef WIFI_MQTT_ENABLE
        if(mqtt_wifi_connected()){
          if(!mqtt_wifi_publish(topic,data,retain)){
            xSemaphoreGive(mqttTxQueueMutex);
            return;
          }
        }else{
          #ifdef ENABLE_LTE
            if(!mqtt_wifi_connected() && network_gprs_mqtt_connected()){
              if(!network_gprs_mqtt_sendMessage(pxMessage->topic,pxMessage->data,pxMessage->retain)){
                xSemaphoreGive(mqttTxQueueMutex);
                return;
              }
            }
          #endif
        }
      #elif defined ENABLE_LTE

        if(network_gprs_mqtt_connected()){
          if(!network_gprs_mqtt_sendMessage(pxMessage->topic,pxMessage->data,pxMessage->retain)){
            xSemaphoreGive(mqttTxQueueMutex);
            return;
          }
        }
      #endif
      */
    }

  }
  xSemaphoreGive(mqttTxQueueMutex);
}
