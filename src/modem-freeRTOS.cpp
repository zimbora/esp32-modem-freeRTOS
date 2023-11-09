

#include "modem-freeRTOS.hpp"

#define MQTT_RX_QUEUE_SIZE 5
#define MQTT_TX_QUEUE_SIZE 10

#define TCP_RX_QUEUE_SIZE 2
#define TCP_TX_QUEUE_SIZE 2

#define HTTP_RX_QUEUE_SIZE 1 // !! do not change it
#define HTTP_TX_QUEUE_SIZE 1 // !! do not change it

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;

String topic1 = "";
String topic2 = "";

// TCP
QueueHandle_t tcpRxQueue;
QueueHandle_t tcpTxQueue;
TCP_MSG rcv_tcp_msg[TCP_RX_QUEUE_SIZE];
TCP_MSG tx_tcp_msg[TCP_TX_QUEUE_SIZE];
SemaphoreHandle_t tcpTxQueueMutex;
TCP_SETUP tcp[MAX_TCP_CONNECTIONS];

// HTTP
QueueHandle_t httpHeaderRxQueue;
QueueHandle_t httpBodyRxQueue;
QueueHandle_t httpTxQueue;
HTTP_HEADER_MSG rcv_http_header_msg[HTTP_RX_QUEUE_SIZE];
HTTP_BODY_MSG rcv_http_body_msg[HTTP_RX_QUEUE_SIZE];
HTTP_REQUEST_ tx_HTTP_REQUEST_[HTTP_TX_QUEUE_SIZE];
SemaphoreHandle_t httpTxQueueMutex;

// MQTT
QueueHandle_t mqttRxQueue;
QueueHandle_t mqttTxQueue;
MQTT_MSG_RX rcv_mqtt_msg[MQTT_RX_QUEUE_SIZE];
MQTT_MSG_TX tx_mqtt_msg[MQTT_TX_QUEUE_SIZE];
SemaphoreHandle_t mqttTxQueueMutex;
MQTT_SETUP mqtt[MAX_MQTT_CONNECTIONS];

#ifdef ENABLE_LTE
MODEMBGXX modem;
#else
#include <WiFi.h>

EspMQTTClient mqtt1;
/*
EspMQTTClient mqtt1(
  "",
  1883,
  "",
  "",
  "replaceItasSoonAsPosibble"
);
*/
EspMQTTClient mqtt2;
/*
EspMQTTClient mqtt2(
  "",
  1883,
  "",
  "",
  "replaceItasSoonAsPosibble"
);
*/
#endif

// private vars
uint32_t timeout_loop = 0;
bool wifi_connected = false;
bool update_clock = false;
String ip = "";
int16_t rssi = -1;
String mac_address = "";
uint32_t retry_counter = 1;
uint32_t connect_retry = 0;

void (*tcpOnConnect)(uint8_t clientID);
void (*mqttOnConnect)(uint8_t clientID);
bool (*mqtt_callback)(uint8_t,String,String);
bool mqtt_parse_msg(uint8_t clientID, String topic, String payload){
  mqtt_enqueue_msg(clientID,topic,payload);
  return true;
}

/*
* init class for WiFi
*/
void MODEMfreeRTOS::init(const char* ssid, const char* password){

  mqttRxQueue = xQueueCreate( MQTT_RX_QUEUE_SIZE, sizeof( struct MQTT_MSG_RX * ) );
  mqttTxQueue = xQueueCreate( MQTT_TX_QUEUE_SIZE, sizeof( struct MQTT_MSG_TX * ) );

  tcpRxQueue = xQueueCreate( TCP_RX_QUEUE_SIZE, sizeof( struct TCP_MSG * ) );
  tcpTxQueue = xQueueCreate( TCP_TX_QUEUE_SIZE, sizeof( struct TCP_MSG * ) );

  httpTxQueue = xQueueCreate( HTTP_RX_QUEUE_SIZE, sizeof( struct HTTP_MSG * ) );
  httpHeaderRxQueue = xQueueCreate( HTTP_RX_QUEUE_SIZE, sizeof( struct HTTP_HEADER_MSG * ) );
  httpBodyRxQueue = xQueueCreate( HTTP_TX_QUEUE_SIZE, sizeof( struct HTTP_BODY_MSG * ) );

  mqttTxQueueMutex = xSemaphoreCreateMutex();
  tcpTxQueueMutex = xSemaphoreCreateMutex();
  httpTxQueueMutex = xSemaphoreCreateMutex();

  for(uint8_t i = 0; i<MAX_MQTT_CONNECTIONS; i++){
    mqtt[i].contextID = 0;
    mqtt[i].active = false;
    mqtt[i].msg_id = 0;
  }

  for(uint8_t i = 0; i<MAX_TCP_CONNECTIONS; i++){
    tcp[i].contextID = 0;
    tcp[i].active = false;
  }

  #ifndef ENABLE_LTE
  #ifdef DEBUG_MQTT_MSG
  mqtt1.enableDebuggingMessages();
  mqtt2.enableDebuggingMessages();
  #endif
  WiFi.disconnect(true);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
  #endif

}

/*
* init hw interface and class for LTE
*
* @cops - operator selection, if 0 let modem search automatically
* @mode - radio technology GSM:1/GPRS:2/NB:3/CATM1:4/AUTO:5 (auto for any available technology)
* @pwkey - gpio to switchOn module
*/
void MODEMfreeRTOS::init(uint16_t cops, uint8_t mode, uint8_t pwkey){

  mqttRxQueue = xQueueCreate( MQTT_RX_QUEUE_SIZE, sizeof( struct MQTT_MSG_RX * ) );
  mqttTxQueue = xQueueCreate( MQTT_TX_QUEUE_SIZE, sizeof( struct MQTT_MSG_TX * ) );

  tcpRxQueue = xQueueCreate( TCP_RX_QUEUE_SIZE, sizeof( struct TCP_MSG * ) );
  tcpTxQueue = xQueueCreate( TCP_TX_QUEUE_SIZE, sizeof( struct TCP_MSG * ) );

  httpTxQueue = xQueueCreate( HTTP_RX_QUEUE_SIZE, sizeof( struct HTTP_MSG * ) );
  httpHeaderRxQueue = xQueueCreate( HTTP_RX_QUEUE_SIZE, sizeof( struct HTTP_HEADER_MSG * ) );
  httpBodyRxQueue = xQueueCreate( HTTP_TX_QUEUE_SIZE, sizeof( struct HTTP_BODY_MSG * ) );

  mqttTxQueueMutex = xSemaphoreCreateMutex();
  tcpTxQueueMutex = xSemaphoreCreateMutex();
  httpTxQueueMutex = xSemaphoreCreateMutex();

  for(uint8_t i = 0; i<MAX_MQTT_CONNECTIONS; i++){
    mqtt[i].contextID = 0;
    mqtt[i].msg_id = 0;
  }

  for(uint8_t i = 0; i<MAX_TCP_CONNECTIONS; i++){
    tcp[i].contextID = 0;
    tcp[i].active = false;
  }

  #ifdef ENABLE_LTE
  modem.init_port(115200,SERIAL_8N1);
  modem.init(mode,cops,pwkey);
  Serial.println("firmware version");
  Serial.println(get_firmware_version());
  #endif
  // init contexts
}

/*
* Executes whatever is needed
* call it always that is possible
*/
void MODEMfreeRTOS::loop(){

  #ifndef ENABLE_LTE
  if(!isWifiConnected()){
    if(connect_retry < millis() && connect_retry != 0){
      // device is offline for too long, restart it
      WiFi.reconnect();
    }
  }
  #endif

  // WIFI
  #ifndef ENABLE_LTE
  if(mqtt1.getMqttServerIp() != "")
    mqtt1.loop();
  if(mqtt2.getMqttServerIp() != "")
    mqtt2.loop();
  #endif
  // tcp

  if(timeout_loop < millis()){

    timeout_loop = millis()+100;

    //LTE
    #ifdef ENABLE_LTE
    lte_loop();
    #else
    if(update_clock){
      sync_clock();
      update_clock = false;
    }
    #endif

    // TCP
    tcp_sendMessage();
    tcp_checkMessages();

    // mqtt
    mqtt_sendMessage();

    // http
    http_execute_requests();

  }


}

#ifdef ENABLE_LTE
void MODEMfreeRTOS::lte_loop(){
  if(modem.loop(5000)){ // state was updated

    for(uint8_t i=0; i<MAX_MQTT_CONNECTIONS; i++){
      if(mqtt[i].contextID == 0)
        continue;
      if(modem.has_context(mqtt[i].contextID)){
        if(!modem.MQTT_connected(i)){
          if(modem.MQTT_connect(i,mqtt[i].nick.c_str(),mqtt[i].user.c_str(),mqtt[i].pwd.c_str(),mqtt[i].host.c_str(),mqtt[i].port)){
            if(mqttOnConnect != NULL)
              mqttOnConnect(i);
            for(uint8_t j=0; j<10; j++){
              if(mqtt[i].subscribe_topics[j] == "")
                continue;
              String topic = mqtt[i].prefix + mqtt[i].subscribe_topics[j];
              modem.MQTT_subscribeTopic(i,++mqtt[i].msg_id,topic,2);
            }
          }
        }
      }else{
        modem.open_pdp_context(mqtt[i].contextID);
        return; // open a context at a time
      }
    }

    for(uint8_t i=0; i<MAX_TCP_CONNECTIONS; i++){
      if(tcp[i].contextID == 0)
        continue;

      if(modem.has_context(tcp[i].contextID)){
        if(!modem.tcp_connected(i)){
          if(modem.tcp_connect(tcp[i].contextID,i,tcp[i].host,tcp[i].port)){
            if(tcpOnConnect != NULL)
              tcpOnConnect(i);
          }
        }
      }else{
        modem.open_pdp_context(tcp[i].contextID);
      }
    }

    for(uint8_t i=0; i<HTTP_TX_QUEUE_SIZE; i++){
      if(tx_HTTP_REQUEST_[i].contextID == 0)
        continue;

      if(!modem.has_context(tx_HTTP_REQUEST_[i].contextID))
        modem.open_pdp_context(tx_HTTP_REQUEST_[i].contextID);
    }

  }

  if(update_clock){
    modem.update_sys_clock();
    update_clock = false;
  }

}
#endif

#ifndef ENABLE_LTE
void MODEMfreeRTOS::sync_clock(){

  Serial.println("NTP config time");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }else{
    Serial.println(&timeinfo, "NTP %A, %B %d %Y %H:%M:%S");

    int y = timeinfo.tm_year + 1900;
    int mo = timeinfo.tm_mon + 1;
    int d = timeinfo.tm_mday;
    int h = timeinfo.tm_hour;
    int m = timeinfo.tm_min;
    int s = timeinfo.tm_sec;

    setTime(h, m, s, d, mo, y);
  }

}
#endif

// Don't use it for now. It will force mqtt lib to handle wifi
void MODEMfreeRTOS::wifi_configure_ap(const char* wifi_ssid, const char* wifi_pwd){
  /*
  #ifndef ENABLE_LTE
  Serial.printf("ssid: %s \n",wifi_ssid);
  Serial.printf("wifi_pwd: %s \n",wifi_pwd);
  mqtt1.setWifiCredentials(wifi_ssid,wifi_pwd);
  #endif
  */
}

bool MODEMfreeRTOS::isWifiConnected(){
  return wifi_connected;
}

/*
* sets contextID to use ssl
* call it before set_context
*
* @contextID 1-16, limited to MAX_CONNECTIONS defined in bgxx library
*/
bool MODEMfreeRTOS::set_ssl(uint8_t contextID){
  if(contextID == 0 || contextID > MAX_CONNECTIONS)
    return false;

  #ifdef ENABLE_LTE
  return modem.set_ssl(contextID);
  #else
  return false;
  #endif
}

/*
* activates and sets contextID
*
* @contextID 1-16, limited to MAX_CONNECTIONS defined in bgxx library
* @apn - operator intranetwork
* @user - apn credentials
* @pwd - apn credentials
*/
bool MODEMfreeRTOS::set_context(uint8_t contextID, String apn, String user, String pwd){
  if(contextID == 0 || contextID > MAX_CONNECTIONS)
    return false;

  #ifdef ENABLE_LTE
  return modem.setup(contextID,apn,user,pwd);
  #else
  return false;
  #endif
}

void MODEMfreeRTOS::log_modem_status(){
  #ifdef ENABLE_LTE
  modem.log_status();
  #else
  Serial.println("wifi connected?: "+String(isWifiConnected()));
  Serial.println("mqtt client 1 connected?: "+String(mqtt_isConnected(0)));
  if(mqtt[1].active)
    Serial.println("mqttc client 2 connected?: "+String(mqtt_isConnected(1)));
  #endif
}

String MODEMfreeRTOS::get_manufacturer_identification(){
  #ifdef ENABLE_LTE
  return modem.get_manufacturer_identification();
  #else
  return "";
  #endif
}

String MODEMfreeRTOS::get_model_identification(){
  #ifdef ENABLE_LTE
  return modem.get_model_identification();
  #else
  return "";
  #endif
}

String MODEMfreeRTOS::get_firmware_version(){
  #ifdef ENABLE_LTE
  return modem.get_firmware_version();
  #else
  return "";
  #endif
}

int16_t MODEMfreeRTOS::get_rssi(){
  #ifdef ENABLE_LTE
  return modem.rssi();
  #else
  return WiFi.RSSI();
  #endif
}

String MODEMfreeRTOS::get_technology(){
  #ifdef ENABLE_LTE
  return modem.technology();
  #else
  return "WiFi";
  #endif
}

String MODEMfreeRTOS::macAddress() {
  if(mac_address == ""){
    String mac = WiFi.macAddress();
    // drop ':''
    mac.replace(":", "");
    // lower letters
    mac.toLowerCase();
    mac_address = mac;
  }

  return mac_address;
}

bool MODEMfreeRTOS::isLTERegistered(){
  #ifdef ENABLE_LTE
  return modem.get_actual_mode() != 0;
  #else
  return false;
  #endif
}

/*
* return timezone difference in seconds
*/
uint32_t MODEMfreeRTOS::get_tz(){
  #ifdef ENABLE_LTE
  return modem.get_tz();
  #endif

  return 0;
}

/*
* set flag to update sys clock asap
*/
void MODEMfreeRTOS::update_clock_sys(){
  update_clock = true;
}

/*
* call it before mqtt_setup
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
  if(clientID >= MAX_MQTT_CONNECTIONS)
    return;
  mqtt[clientID].contextID = contextID;
  mqtt[clientID].active = true;
  mqtt[clientID].nick = uid;
  mqtt[clientID].prefix = project+"/"+uid;
  mqtt[clientID].host = host;
  mqtt[clientID].port = port;
  mqtt[clientID].user = user;
  mqtt[clientID].pwd = pwd;
}

/*
* call it before mqtt_setup
* changes mqtt connection parameters - use it with wifi
*
* @clientID 0-5, limited to MAX_MQTT_CONNECTIONS defined in bgxx library
* @contextID 1-16, limited to MAX_CONNECTIONS defined in bgxx library
* @project - string to create prefix of topic - :project/:uid/...
* @host - IP or DNS of server
* @user - credential username
* @pwd - credential password
*/
void MODEMfreeRTOS::mqtt_configure_connection(uint8_t clientID, const char* project, const char* uid, const char* host, uint16_t port, const char* user, const char* pwd){
  if(clientID >= MAX_MQTT_CONNECTIONS)
    return;

  mqtt[clientID].active = true;
  mqtt[clientID].nick = String(uid);
  mqtt[clientID].prefix = String(project)+"/"+String(uid);
  mqtt[clientID].host = String(host);
  mqtt[clientID].port = port;
  mqtt[clientID].user = String(user);
  mqtt[clientID].pwd = String(pwd);

  #ifndef ENABLE_LTE

  Serial.println("mqtt nick: "+mqtt[clientID].nick);
  Serial.println("will topic: "+mqtt[clientID].will_topic);
  if(clientID == 0){

    topic1 = mqtt[clientID].prefix+mqtt[clientID].will_topic;
    Serial.println("will 1: "+topic1);
    mqtt1.enableLastWillMessage(topic1.c_str(),"offline",true);

    mqtt1.setMqttClientName(mqtt[clientID].nick .c_str());
    mqtt1.setMqttServer(host,user,pwd,port);
  }else if(clientID == 1){

    topic2 = mqtt[clientID].prefix+mqtt[clientID].will_topic;
    Serial.println("will 2: "+topic2);
    mqtt2.enableLastWillMessage(topic2.c_str(),"offline",true);

    mqtt2.setMqttClientName(uid);
    mqtt2.setMqttServer(host,user,pwd,port);
  }

  Serial.println(mqtt1.getMqttClientName());
  #endif
}

// --- TCP ---

/*
* call it before tcp_setup
* changes tcp connection parameters
* while clientID has contextID != 0, loop function will try to keep connection activated
*
* @clientID 0-5, limited to MAX_TCP_CONNECTIONS defined in bgxx library
* @contextID 1-16, limited to MAX_CONNECTIONS defined in bgxx library
* @host - IP or DNS of server
* @port
*/
void MODEMfreeRTOS::tcp_configure_connection(uint8_t clientID, uint8_t contextID, String host, uint16_t port){
  if(clientID > MAX_TCP_CONNECTIONS)
    return;
  tcp[clientID].contextID = contextID;
  tcp[clientID].host = host;
  tcp[clientID].port = port;
}

/*
* configures callbacks to be called when connection is established and closed
*/
void MODEMfreeRTOS::tcp_setup(void(*callback1)(uint8_t clientID),void(*callback2)(uint8_t clientID)){

  tcpOnConnect = callback1;
  #ifdef ENABLE_LTE
  modem.tcp_set_callback_on_close(callback2);
  #else
  #endif
}

/*
* private method
* checks if there are data on buffers. Buffers are filled automatically and must be read with high frequency
* if a long response is being received
* Data is then sent to queue. Use tcp_getNextMessage to read data
*/
void MODEMfreeRTOS::tcp_checkMessages(){

  #ifdef ENABLE_LTE
  for(uint8_t i=0; i<MAX_TCP_CONNECTIONS; i++){
    uint16_t len = modem.tcp_has_data(i);
    if(len > 0){
      char* data = (char*)malloc(len);
      if(data != nullptr){
        len = modem.tcp_recv(i,data,len);
        /*
        for(uint16_t i = 0; i<len; i++){
          Serial.print((char)data[i]);
        }
        */
        tcp_enqueue_msg(i,(const char*)data,len);
        free(data);
      }
    }
  }
  #endif

}

/*
* private method
* called on received message. It adds message to queue
*/
void MODEMfreeRTOS::tcp_enqueue_msg(uint8_t clientID, const char* data, uint16_t data_len){

  //#ifdef DEBUG_tcp_GW
  Serial.println("[tcp] << ["+String(clientID)+"] size:" + String(data_len));
  //#endif
  struct TCP_MSG *pxMessage;

  if( tcpRxQueue != 0 && uxQueueSpacesAvailable(tcpRxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     pxMessage = &rcv_tcp_msg[uxQueueMessagesWaiting(tcpRxQueue)];
     memset(pxMessage->data,0,sizeof(pxMessage->data));
     memcpy(pxMessage->data,data,data_len);
     pxMessage->data_len = data_len;
     pxMessage->clientID = clientID;
     xQueueGenericSend( tcpRxQueue, ( void * ) &pxMessage, ( TickType_t ) 0, queueSEND_TO_BACK );
  }else{
    #ifdef WARNING_TCP_GW
    Serial.println("tcp rx no space available");
    #endif
  }
}

/*
* use it to get received messages.
*
* returns a pointer to TCP_MSG struct containing the received message
* if no message is available it returns NULL
*/
TCP_MSG* MODEMfreeRTOS::tcp_getNextMessage(TCP_MSG *pxRxedMessage){

  if( tcpRxQueue != 0 && uxQueueMessagesWaiting(tcpRxQueue) > 0){
     // get a message on the created queue.  Block for 10 ticks if a
     // message is not immediately available.
     //if( xQueueReceive( tcpRxQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
     if( xQueueReceive( tcpRxQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
         // pcRxedMessage now points to the struct AMessage variable posted
         // by vATask, but the item still remains on the queue.
         return pxRxedMessage;
     }
  }
  return NULL;
}


/*
* use it to send tcp messages
*
* @clientID 0-5, tcp index client
* @data - payload to be sent
* @len - payload len
* @retain true|false
*/
bool MODEMfreeRTOS::tcp_pushMessage(uint8_t clientID, const char* data, uint16_t len) {

  if(clientID >= MAX_TCP_CONNECTIONS)
    return false;
  /*
  if(qos == 0 && !tcp_connected())
    return false;
  */
  struct TCP_MSG *pxMessage;

  if( tcpTxQueue != 0 && uxQueueSpacesAvailable(tcpTxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     if(!xSemaphoreTake( tcpTxQueueMutex, 2000)){
       xSemaphoreGive(tcpTxQueueMutex);
       return false;
     }

     pxMessage = &tx_tcp_msg[uxQueueMessagesWaiting(tcpTxQueue)];
     memset(pxMessage->data,0,sizeof(pxMessage->data));

     memcpy(pxMessage->data,data,len);
     pxMessage->data_len = len;
     pxMessage->clientID = clientID;

     bool res = xQueueSendToBack( tcpTxQueue, ( void * ) &pxMessage, ( TickType_t ) 0 ) == true;
     xSemaphoreGive(tcpTxQueueMutex);
     return res;
  }

  return false;

}

/*
* private method
* checks if queue has messages to be sent.
* If there is it will send using an available network interface
*/
void MODEMfreeRTOS::tcp_sendMessage(){

  struct TCP_MSG *pxMessage;

  if(!xSemaphoreTake( tcpTxQueueMutex, 2000)){
    xSemaphoreGive(tcpTxQueueMutex);
    return;
  }

  while( tcpTxQueue != 0 && uxQueueMessagesWaiting(tcpTxQueue) > 0){
    // get a message on the created queue.  Block for 10 ticks if a
    // message is not immediately available.

    //if( xQueueReceive( tcpTxQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
    if( xQueueReceive( tcpTxQueue, &( pxMessage ), ( TickType_t ) 10 ) ){
      // pcRxedMessage now points to the struct AMessage variable posted
      // by vATask, but the item still remains on the queue.
      uint8_t clientID = pxMessage->clientID;
      if(clientID >= MAX_TCP_CONNECTIONS){
        Serial.println("invalid tcp clientID");
        return;
      }

      #ifdef ENABLE_LTE
      if(!modem.tcp_connected(clientID)){
        tcp_pushMessage(clientID,pxMessage->data,pxMessage->data_len);
        return;
      }else{
        //Serial.println("[tcp] >> "+topic + " : " + data);
        modem.tcp_send(clientID,pxMessage->data,pxMessage->data_len);
      }
      #else
      tcp_pushMessage(clientID,pxMessage->data,pxMessage->data_len); // for now
      return;
      #endif
    }

  }
  xSemaphoreGive(tcpTxQueueMutex);
}


// --- HTTP

/*
* use it to get http header messages.
*
* returns a pointer to HTTP_HEADER_MSG struct containing the received message
* if no message is available it returns NULL
*/
HTTP_HEADER_MSG* MODEMfreeRTOS::http_header_getNextMessage(HTTP_HEADER_MSG *pxRxedMessage){

  if( httpHeaderRxQueue != 0 && uxQueueMessagesWaiting(httpHeaderRxQueue) > 0){
     // get a message on the created queue.  Block for 10 ticks if a
     // message is not immediately available.
     //if( xQueueReceive( httpHeaderRxQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
     if( xQueueReceive( httpHeaderRxQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
         // pcRxedMessage now points to the struct AMessage variable posted
         // by vATask, but the item still remains on the queue.
         return pxRxedMessage;
     }
  }
  return NULL;
}

/*
* use it to get http body messages.
*
* returns a pointer to HTTP_BODY_MSG struct containing the received message
* if no message is available it returns NULL
*/
HTTP_BODY_MSG* MODEMfreeRTOS::http_body_getNextMessage(HTTP_BODY_MSG *pxRxedMessage){

  if( httpBodyRxQueue != 0 && uxQueueMessagesWaiting(httpBodyRxQueue) > 0){
     // get a message on the created queue.  Block for 10 ticks if a
     // message is not immediately available.
     //if( xQueueReceive( httpBodyRxQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
     if( xQueueReceive( httpBodyRxQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
         // pcRxedMessage now points to the struct AMessage variable posted
         // by vATask, but the item still remains on the queue.
         return pxRxedMessage;
     }
  }
  return NULL;
}

/*
* use it to do http requests - LTE
*
* @contextID 1-11, context id
* @clientID 0-5, tcp index client
* @host
* @path
* @method
* @header_key - use it to add a parameter to header (key)
* @header_value - use it to add a parameter to header (value)
*/
bool MODEMfreeRTOS::http_pushMessage(uint8_t contextID, uint8_t clientID, String host, String path, String method, String header_key, String header_value, String body, bool json){

  if(clientID >= MAX_TCP_CONNECTIONS)
    return false;
  /*
  if(qos == 0 && !tcp_connected())
    return false;
  */
  struct HTTP_REQUEST_ *pxMessage;

  if( httpTxQueue != 0 && uxQueueSpacesAvailable(httpTxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     if(!xSemaphoreTake( httpTxQueueMutex, 2000)){
       xSemaphoreGive(httpTxQueueMutex);
       return false;
     }

     pxMessage = &tx_HTTP_REQUEST_[uxQueueMessagesWaiting(httpTxQueue)];

     pxMessage->protocol = "HTTP";
     pxMessage->contextID = contextID;
     pxMessage->clientID = clientID;
     pxMessage->host = host;
     pxMessage->path = path;
     pxMessage->method = method;
     pxMessage->header_key = header_key;
     pxMessage->header_value = header_value;
     pxMessage->body = body;
     pxMessage->json = json;

     bool res = xQueueSendToBack( httpTxQueue, ( void * ) &pxMessage, ( TickType_t ) 0 ) == true;
     xSemaphoreGive(httpTxQueueMutex);
     return res;
  }

  return false;

}

/*
* use it to do http requests - WIFI
*
* @host
* @path
* @method
* @header_key - use it to add a parameter to header (key)
* @header_value - use it to add a parameter to header (value)
*/
bool MODEMfreeRTOS::http_pushMessage(String host, String path, String method, String header_key, String header_value, String body, bool json){

  /*
  if(qos == 0 && !tcp_connected())
    return false;
  */
  struct HTTP_REQUEST_ *pxMessage;

  if( httpTxQueue != 0 && uxQueueSpacesAvailable(httpTxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     if(!xSemaphoreTake( httpTxQueueMutex, 2000)){
       xSemaphoreGive(httpTxQueueMutex);
       return false;
     }

     pxMessage = &tx_HTTP_REQUEST_[uxQueueMessagesWaiting(httpTxQueue)];

     pxMessage->protocol = "HTTP";
     pxMessage->host = host;
     pxMessage->path = path;
     pxMessage->method = method;
     pxMessage->header_key = header_key;
     pxMessage->header_value = header_value;
     pxMessage->body = body;
     pxMessage->json = json;

     bool res = xQueueSendToBack( httpTxQueue, ( void * ) &pxMessage, ( TickType_t ) 0 ) == true;
     xSemaphoreGive(httpTxQueueMutex);
     return res;
  }

  return false;

}

/*
* use it to do http requests - LTE
*
* @contextID 1-11, context id
* @clientID 0-5, tcp index client
* @sslClientID 0-5, ssl index client
* @host
* @path
* @method - GET|POST
* @header_key - to be added to header ex: "token"
* @header_value - to be added to header ex: "asd"
* @body
* @json - true|false (body data format)
*/
bool MODEMfreeRTOS::https_pushMessage(uint8_t contextID, uint8_t clientID, uint8_t sslClientID, String host, String path, String method, String header_key, String header_value, String body, bool json)  {

  if(clientID >= MAX_TCP_CONNECTIONS)
    return false;
  /*
  if(qos == 0 && !tcp_connected())
    return false;
  */
  struct HTTP_REQUEST_ *pxMessage;

  if( httpTxQueue != 0 && uxQueueSpacesAvailable(httpTxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     if(!xSemaphoreTake( httpTxQueueMutex, 2000)){
       xSemaphoreGive(httpTxQueueMutex);
       return false;
     }

     pxMessage = &tx_HTTP_REQUEST_[uxQueueMessagesWaiting(httpTxQueue)];

     pxMessage->protocol = "HTTPS";
     pxMessage->contextID = contextID;
     pxMessage->clientID = clientID;
     pxMessage->sslClientID = sslClientID;
     pxMessage->host = host;
     pxMessage->path = path;
     pxMessage->method = method;
     pxMessage->header_key = header_key;
     pxMessage->header_value = header_value;
     pxMessage->body = body;
     pxMessage->json = json;

     bool res = xQueueSendToBack( httpTxQueue, ( void * ) &pxMessage, ( TickType_t ) 0 ) == true;
     xSemaphoreGive(httpTxQueueMutex);
     return res;
  }

  return false;

}

/*
* use it to do http requests - WIFI
*
* @host
* @path
* @method - GET|POST
* @header_key - to be added to header ex: "token"
* @header_value - to be added to header ex: "asd"
* @body
* @json - true|false (body data format)
*/
bool MODEMfreeRTOS::https_pushMessage(String host, String path, String method, String header_key, String header_value, String body, bool json)  {

  struct HTTP_REQUEST_ *pxMessage;

  if( httpTxQueue != 0 && uxQueueSpacesAvailable(httpTxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     if(!xSemaphoreTake( httpTxQueueMutex, 2000)){
       xSemaphoreGive(httpTxQueueMutex);
       return false;
     }

     pxMessage = &tx_HTTP_REQUEST_[uxQueueMessagesWaiting(httpTxQueue)];

     pxMessage->protocol = "HTTPS";
     pxMessage->host = host;
     pxMessage->path = path;
     pxMessage->method = method;
     pxMessage->header_key = header_key;
     pxMessage->header_value = header_value;
     pxMessage->body = body;
     pxMessage->json = json;

     bool res = xQueueSendToBack( httpTxQueue, ( void * ) &pxMessage, ( TickType_t ) 0 ) == true;
     xSemaphoreGive(httpTxQueueMutex);
     return res;
  }

  return false;

}


/*
* private method
* checks if queue has requests to be done.
* If there is it will send using an available network interface
*/
void MODEMfreeRTOS::http_execute_requests(){

  struct HTTP_REQUEST_ *pxMessage;

  if(!xSemaphoreTake( httpTxQueueMutex, 2000)){
    xSemaphoreGive(httpTxQueueMutex);
    return;
  }

  while( httpTxQueue != 0 && uxQueueMessagesWaiting(httpTxQueue) > 0){
    // get a message on the created queue.  Block for 10 ticks if a
    // message is not immediately available.
    //if( xQueueReceive( httpTxQueue, &( pxRxedMessage ), ( TickType_t ) 10 ) ){
    if( xQueueReceive( httpTxQueue, &( pxMessage ), ( TickType_t ) 10 ) ){
      // pcRxedMessage now points to the struct AMessage variable posted
      // by vATask, but the item still remains on the queue.
      uint8_t clientID = pxMessage->clientID;
      uint8_t contextID = pxMessage->contextID;
      uint8_t sslClientID = pxMessage->sslClientID;
      if(clientID >= MAX_TCP_CONNECTIONS){
        Serial.println("invalid tcp clientID");
        xSemaphoreGive(httpTxQueueMutex);
        return;
      }

      xSemaphoreGive(httpTxQueueMutex);

      #ifdef ENABLE_LTE
      if(!modem.has_context(contextID)){
        if(pxMessage->protocol == "HTTP")
          http_pushMessage(contextID,clientID,pxMessage->host,pxMessage->path,pxMessage->method,pxMessage->header_key,pxMessage->header_value,pxMessage->body,pxMessage->json);
        else if(pxMessage->protocol == "HTTPS"){
          https_pushMessage(contextID,clientID,sslClientID,pxMessage->host,pxMessage->path,pxMessage->method,pxMessage->header_key,pxMessage->header_value,pxMessage->body,pxMessage->json);
        }else
          Serial.println("http protocol not known");
        return;
      }else{
        if(pxMessage->protocol == "HTTP")
          http_get_request(contextID,clientID,pxMessage->host,pxMessage->path);
        else if(pxMessage->protocol == "HTTPS"){
          String token = "";
          if(pxMessage->header_key != "")
            token = pxMessage->header_key + ":" + pxMessage->header_value;
          https_request(contextID,clientID,sslClientID,pxMessage->host,pxMessage->path,token,pxMessage->body,pxMessage->method,pxMessage->json);
        }else
          Serial.println("http protocol not known");
        return;
      }
      #else
        if(!isWifiConnected()){
          if(pxMessage->protocol == "HTTP")
            http_pushMessage(pxMessage->host,pxMessage->path,pxMessage->method,pxMessage->header_key,pxMessage->header_value,pxMessage->body,pxMessage->json);
          else if(pxMessage->protocol == "HTTPS")
            https_pushMessage(pxMessage->host,pxMessage->path,pxMessage->method,pxMessage->header_key,pxMessage->header_value,pxMessage->body,pxMessage->json);
          else
            Serial.println("http protocol not known");
          return;
        }else{
          if(pxMessage->protocol == "HTTP")
            wifi_HTTP_REQUEST_(pxMessage->host,pxMessage->path,pxMessage->method,pxMessage->header_key,pxMessage->header_value,pxMessage->body,pxMessage->json);
          else if(pxMessage->protocol == "HTTPS")
            wifi_https_request(pxMessage->host,pxMessage->path,pxMessage->method,pxMessage->header_key,pxMessage->header_value,pxMessage->body,pxMessage->json);
          else
            Serial.println("http protocol not known");
          return;
        }

      #endif
    }

  }

  xSemaphoreGive(httpTxQueueMutex);
}

/*
* private method
*/
bool MODEMfreeRTOS::http_get_request(uint8_t contextID, uint8_t clientID, String host, String path){

  #ifdef ENABLE_LTE
  if(!modem.http_get(host,path,"",clientID,contextID)){
    #ifdef DEBUG_HTTP_ERROR
    Serial.printf("http request to: %s%s has failed..\n",host.c_str(),path.c_str());
    #endif
    return false;
  }
  if(!modem.http_wait_response(clientID)){
    #ifdef DEBUG_HTTP_ERROR
    Serial.println("http request: no response received");
    #endif
    return false;
  }

  if(modem.http_response_status().indexOf("200") > -1){
    uint16_t len = modem.http_get_body_size();
    #ifdef DEBUG_HTTP
    Serial.println("http request: OK");
    Serial.printf("http body size: %d \n",len);
    #endif
    http_enqueue_header_msg(clientID, len, modem.http_md5().c_str(), modem.http_response_status());

    uint16_t len_read = 0, total_len_read = 0;
    char* data = (char*)malloc(CONNECTION_BUFFER);
    if(data != nullptr){
      memset(data,0,CONNECTION_BUFFER);
      while(true){

        // if queue has space
        if(http_queue_body_has_space()){
          len_read = modem.http_get_body(clientID,data,CONNECTION_BUFFER);

          if(len_read > 0){
            #ifdef DEBUG_HTTP
            Serial.printf("send %d bytes to queue \n",len_read);
            #endif
            http_enqueue_body_msg(clientID,data,len_read);
            memset(data,0,len_read);
            total_len_read += len_read;
            if(len == total_len_read){
              #ifdef DEBUG_HTTP
              Serial.println("all data was read");
              #endif
              break;
            }
          }
        }
        delay(100);
      }
      /*
      for(uint16_t i = 0; i<len_read; i++)
        Serial.print((char)data[i]);
      */

      free(data);
    }
  }else{
    #ifdef DEBUG_HTTP_ERROR
    Serial.println("response: "+modem.http_response_status());
    #endif
    http_enqueue_header_msg(clientID, 0, modem.http_md5().c_str(), modem.http_response_status());
  }
  #else
  Serial.println("WiFi - implement http get request");
  #endif

}

bool MODEMfreeRTOS::https_request(uint8_t contextID, uint8_t clientID, uint8_t sslClientID, String host, String path, String token, String body, String method, bool json){

  #ifdef ENABLE_LTE

  #ifdef DEBUG_HTTP
  Serial.println("method: "+method);
  #endif

  if(method == "GET"){
    if(!modem.https_get(host,path,token,clientID,sslClientID,contextID)){
      #ifdef DEBUG_HTTP_ERROR
      Serial.printf("http request to: %s%s has failed..\n",host.c_str(),path.c_str());
      #endif
      return false;
    }
  }else if(method == "POST"){
    if(json){
      if(!modem.https_post_json(host,path,body,token,clientID,sslClientID,contextID)){
        #ifdef DEBUG_HTTP_ERROR
        Serial.printf("https json post to: %s%s has failed..\n",host.c_str(),path.c_str());
        #endif
        return false;
      }
    }else{
      if(!modem.https_post(host,path,body,token,clientID,sslClientID,contextID)){
        #ifdef DEBUG_HTTP_ERROR
        Serial.printf("https post to: %s%s has failed..\n",host.c_str(),path.c_str());
        #endif
        return false;
      }
    }
  }

  if(!modem.http_wait_response(clientID)){
    #ifdef DEBUG_HTTP_ERROR
    Serial.println("http request: no response received");
    #endif
    return false;
  }

  if(modem.http_response_status().indexOf("200") > -1){
    uint16_t len = modem.http_get_body_size();
    #ifdef DEBUG_HTTP
    Serial.println("http request: OK");
    Serial.printf("http body size: %d \n",len);
    #endif
    http_enqueue_header_msg(clientID, len, modem.http_md5().c_str(), modem.http_response_status());

    uint16_t len_read = 0, total_len_read = 0;
    char* data = (char*)malloc(CONNECTION_BUFFER);
    if(data != nullptr){
      memset(data,0,CONNECTION_BUFFER);
      while(true){

        // if queue has space
        if(http_queue_body_has_space()){
          len_read = modem.http_get_body(clientID,data,CONNECTION_BUFFER);

          if(len_read > 0){
            #ifdef DEBUG_HTTP
            Serial.printf("send %d bytes to queue \n",len_read);
            #endif
            http_enqueue_body_msg(clientID,data,len_read);
            memset(data,0,len_read);
            total_len_read += len_read;
            if(len == total_len_read)
              break;
          }
        }
        delay(100);
      }
      /*
      for(uint16_t i = 0; i<len_read; i++)
        Serial.print((char)data[i]);
      */

      free(data);
    }
  }else{
    #ifdef DEBUG_HTTP_ERROR
    Serial.println("response: "+modem.http_response_status());
    #endif
    http_enqueue_header_msg(clientID, 0, modem.http_md5().c_str(), modem.http_response_status());
  }

  #endif
}

#ifndef ENABLE_LTE
bool MODEMfreeRTOS::wifi_HTTP_REQUEST_(String host, String path, String method, String header_token, String header_value, String body, bool json){

  WiFiClient *client = new WiFiClient;
  HTTPClient http;

  #ifdef DEBUG_HTTP
  Serial.print("[HTTP] begin...\n");
  Serial.println("host: "+host);
  Serial.println("path: "+path);
  Serial.println("method: "+method);
  #endif

  // only port 80 works
  uint16_t port = 80;
  int index = host.indexOf(":");
  if( index > 0){
    Serial.printf("Port 80 must be used for http, instead of: %lu \n",port);
    return false;
  }
  http.begin(*client,host,port,path); //HTTP

  const char *keys[3] = {
    "Content-Length",
    "Content-Type",
    "Content-MD5"
  };

  http.collectHeaders(keys,3);
  // start connection and send HTTP header

  if(method != "GET" && json)
    http.addHeader("Content-Type","application/json");
  else if(method != "GET")
    http.addHeader("Content-Type","application/x-www-form-urlencoded");

  if(header_token != "")
    http.addHeader(header_token,header_value);

  int httpCode = 0;
  if(method == "GET"){
    #ifdef DEBUG_HTTP
    Serial.print("[HTTP] GET...\n");
    #endif
    httpCode = http.GET();
  }else if(method == "POST"){
    #ifdef DEBUG_HTTP
    Serial.print("[HTTP] POST...\n");
    #endif
    httpCode = http.POST(body);
  }

  // httpCode will be negative on error
  if(httpCode > 0) {
    // HTTP header has been send and Server response header has been handleda

    String body_len = http.header("Content-Length");
    uint32_t len = body_len.toInt();
    String md5 = http.header("Content-MD5");

    #ifdef DEBUG_HTTP
    Serial.printf("[HTTP] ... code: %d\n", httpCode);
    Serial.println("content length: "+body_len);
    Serial.println("content md5: "+md5);
    Serial.println("content type: "+http.header("Content-Type"));
    #endif

    // file found at server
    if(httpCode == HTTP_CODE_OK) {

      http_enqueue_header_msg(0,body_len.toInt(),md5.c_str(),"200");

      uint16_t len_read = 0, total_len_read = 0;
      char* data = (char*)malloc(CONNECTION_BUFFER);
      if(data != nullptr){
        memset(data,0,CONNECTION_BUFFER);
        while(true){

          // if queue has space
          if(http_queue_body_has_space()){

            uint16_t i = 0;
            while(client->available() && i<CONNECTION_BUFFER){
              data[i++] = (char)client->read();
            }

            len_read = i;
            if(len_read > 0){
              #ifdef DEBUG_HTTP
              Serial.printf("send %d bytes to queue \n",len_read);
              #endif
              http_enqueue_body_msg(0,data,len_read);
              memset(data,0,len_read);
              total_len_read += len_read;
              if(len == total_len_read){
                break;
              }
            }
          }
          delay(100);
        }
        free(data);
      }

    }else{
        #ifdef DEBUG_HTTP_ERROR
        Serial.printf("[HTTP] $s ... failed, error: %s\n", method.c_str(),http.errorToString(httpCode).c_str());
        Serial.printf("https request to: %s%s has failed..\n",host.c_str(),path.c_str());
        #endif
        http_enqueue_header_msg(0,0,md5.c_str(),http.errorToString(httpCode));
    }
  }else Serial.println("Unable to do http request, http code: "+String(httpCode));

  http.end();
}
#endif

#ifndef ENABLE_LTE
bool MODEMfreeRTOS::wifi_https_request(String host, String path, String method, String header_token, String header_value, String body, bool json){

  WiFiClientSecure *client = new WiFiClientSecure;
  HTTPClient http;

  #ifdef DEBUG_HTTP
  Serial.print("[HTTP] begin...\n");
  Serial.println("host: "+host);
  Serial.println("path: "+path);
  Serial.println("method: "+method);
  #endif

  // only port 80 works
  uint16_t port = 443;
  int index = host.indexOf(":");
  if( index > 0){
    Serial.printf("Port 443 must be used for https instead of: %lu \n",port);
    return false;
  }
  http.begin(*client,host,port,path); //HTTP

  const char *keys[3] = {
    "Content-Length",
    "Content-Type",
    "Content-MD5"
  };

  http.collectHeaders(keys,3);
  // start connection and send HTTP header

  if(method != "GET" && json)
    http.addHeader("Content-Type","application/json");
  else if(method != "GET")
    http.addHeader("Content-Type","application/x-www-form-urlencoded");

  if(header_token != "")
    http.addHeader(header_token,header_value);

  int httpCode = 0;
  if(method == "GET"){
    #ifdef DEBUG_HTTP
    Serial.print("[HTTP] GET...\n");
    #endif
    httpCode = http.GET();
  }else if(method == "POST"){
    #ifdef DEBUG_HTTP
    Serial.print("[HTTP] POST...\n");
    #endif
    httpCode = http.POST(body);
  }

  // httpCode will be negative on error
  if(httpCode > 0) {
    // HTTP header has been send and Server response header has been handleda

    String body_len = http.header("Content-Length");
    uint32_t len = body_len.toInt();
    String md5 = http.header("Content-MD5");

    #ifdef DEBUG_HTTP
    Serial.printf("[HTTP] ... code: %d\n", httpCode);
    Serial.println("content length: "+body_len);
    Serial.println("md5: "+md5);
    Serial.println("content type: "+http.header("Content-Type"));
    #endif

    // file found at server
    if(httpCode == HTTP_CODE_OK) {

      http_enqueue_header_msg(0,body_len.toInt(),md5.c_str(),"200");

      uint16_t len_read = 0, total_len_read = 0;
      char* data = (char*)malloc(CONNECTION_BUFFER);
      if(data != nullptr){
        memset(data,0,CONNECTION_BUFFER);
        while(true){

          // if queue has space
          if(http_queue_body_has_space()){

            uint16_t i = 0;
            while(client->available() && i<CONNECTION_BUFFER){
              data[i++] = (char)client->read();
            }

            len_read = i;
            if(len_read > 0){
              #ifdef DEBUG_HTTP
              Serial.printf("send %d bytes to queue \n",len_read);
              #endif
              http_enqueue_body_msg(0,data,len_read);
              memset(data,0,len_read);
              total_len_read += len_read;
              if(len == total_len_read){
                break;
              }
            }
          }
          delay(100);
        }
        free(data);
      }

    }else{
        #ifdef DEBUG_HTTP_ERROR
        Serial.printf("[HTTP] $s ... failed, error: %s\n", method.c_str(),http.errorToString(httpCode).c_str());
        Serial.printf("https request to: %s%s has failed..\n",host.c_str(),path.c_str());
        #endif
        http_enqueue_header_msg(0,0,md5.c_str(),http.errorToString(httpCode));
    }
  }else Serial.println("Unable to do https request");

  http.end();
}
#endif

bool MODEMfreeRTOS::http_enqueue_header_msg(uint8_t clientID, uint32_t body_len, const char* md5, String http_response){

  #ifdef DEBUG_HTTP
  Serial.println("[http header] << ["+String(clientID)+"] size:" + String(body_len));
  #endif
  struct HTTP_HEADER_MSG *pxMessage;

  if( httpHeaderRxQueue != 0 && uxQueueSpacesAvailable(httpHeaderRxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     pxMessage = &rcv_http_header_msg[uxQueueMessagesWaiting(httpHeaderRxQueue)];

     pxMessage->clientID = clientID;
     pxMessage->body_len = body_len;
     pxMessage->md5 = md5;
     pxMessage->http_response = http_response;
     xQueueGenericSend( httpHeaderRxQueue, ( void * ) &pxMessage, ( TickType_t ) 0, queueSEND_TO_BACK );
     #ifdef DEBUG_HTTP
     Serial.println("header msg added to queue");
     #endif
  }else{
    #ifdef WARNING_HTTP_GW
    Serial.println("http rx no space available");
    #endif
    return false;
  }
  return true;
}

bool MODEMfreeRTOS::http_enqueue_body_msg(uint8_t clientID, char* data, uint16_t data_len){

  #ifdef DEBUG_HTTP
  Serial.println("[http body] << ["+String(clientID)+"] size:" + String(data_len));
  #endif
  struct HTTP_BODY_MSG *pxMessage;

  if( httpBodyRxQueue != 0 && uxQueueSpacesAvailable(httpBodyRxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     pxMessage = &rcv_http_body_msg[uxQueueMessagesWaiting(httpBodyRxQueue)];

     pxMessage->clientID = clientID;
     pxMessage->data_len = data_len;
     memset(pxMessage->data,0,sizeof(pxMessage->data));
     memcpy(pxMessage->data,data,data_len);
     xQueueGenericSend( httpBodyRxQueue, ( void * ) &pxMessage, ( TickType_t ) 0, queueSEND_TO_BACK );
  }else{
    #ifdef WARNING_HTTP_GW
    Serial.println("http rx no space available");
    #endif
    return false;
  }
  return true;
}

bool MODEMfreeRTOS::http_queue_body_has_space(){

  if( httpBodyRxQueue != 0 && uxQueueSpacesAvailable(httpBodyRxQueue) > 0)
    return true;

  return false;
}

// --- MQTT ---

/*
* call it before mqtt_setup
* mqtt set will
*
* @clientID 0-5, limited to MAX_MQTT_CONNECTIONS defined in bgxx library
* @topic - without prefix
* @payload - data to be written on topic
*/
void MODEMfreeRTOS::mqtt_set_will_topic(uint8_t clientID, String topic, String payload){
  if(clientID >= MAX_MQTT_CONNECTIONS)
    return;
  mqtt[clientID].will_topic = topic;
  mqtt[clientID].will_payload = payload;
}

/*
* call it before mqtt_setup
* mqtt configuration adds topic to be subscribed, limited to 10 topics
*
* @clientID 0-5, limited to MAX_MQTT_CONNECTIONS defined in bgxx library
* @index 0-9
* @topic - without prefix
*/
void MODEMfreeRTOS::mqtt_add_subscribe_topic(uint8_t clientID, uint8_t index, String topic){
  if(clientID >= MAX_MQTT_CONNECTIONS)
    return;

  if(index >= 10)
    return;

  mqtt[clientID].subscribe_topics[index] = topic;
}

/*
* Init mqtt and configures callback to be called when connection is established
*/
void MODEMfreeRTOS::mqtt_setup(void(*callback)(uint8_t)){

  mqttOnConnect = callback;
  mqtt_callback = &mqtt_parse_msg;

  xSemaphoreGive(mqttTxQueueMutex);

  #ifdef ENABLE_LTE
  modem.MQTT_init(mqtt_callback);

  for(uint8_t i = 0; i<MAX_MQTT_CONNECTIONS; i++){
    if(mqtt[i].contextID != 0)
      modem.MQTT_setup(i,mqtt[i].contextID,mqtt[i].prefix+mqtt[i].will_topic,mqtt[i].will_payload);
  }
  #endif
}

void MODEMfreeRTOS::mqtt_wifi_setup(void(*callback)()){

  xSemaphoreGive(mqttTxQueueMutex);

  #ifndef ENABLE_LTE

  mqtt2.setOnConnectionEstablishedCallback(callback);

  #endif
}

/*
* call it to subscribe topics
*/
void MODEMfreeRTOS::mqtt_subscribeTopics(uint8_t clientID){

  if(clientID >= MAX_MQTT_CONNECTIONS)
    return;

  for(uint8_t j=0; j<10; j++){
    if(mqtt[clientID].subscribe_topics[j] == "")
      continue;
    String topic = mqtt[clientID].prefix + mqtt[clientID].subscribe_topics[j];
    #ifdef ENABLE_LTE
    modem.MQTT_subscribeTopic(clientID,++mqtt[clientID].msg_id,topic,2);
    #else
    if(clientID == 0){
      mqtt1.subscribe(topic, [](const String & topic, const String & payload) {
        mqtt_enqueue_msg(0,topic,payload);
      });
    }else if(clientID == 1){
      mqtt2.subscribe(topic, [](const String & topic, const String & payload) {
        mqtt_enqueue_msg(1,topic,payload);
      });
    }
    #endif
  }
}

/*
* check if mqtt client is connected
*/
bool MODEMfreeRTOS::mqtt_isConnected(uint8_t clientID){

  #ifdef ENABLE_LTE
  return modem.MQTT_connected(clientID);
  #else
  if(clientID == 0)
    return mqtt1.isMqttConnected();
  else if(clientID == 1)
    return mqtt2.isMqttConnected();
  else return false;
  #endif
}

/*
* use it to get received messages.
*
* returns a pointer to MQTT_MSG struct containing the received message
* if no message is available it returns NULL
*/
MQTT_MSG_RX* MODEMfreeRTOS::mqtt_getNextMessage(MQTT_MSG_RX *pxRxedMessage){

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
* use it to send mqtt messages
* If qos == 0 and mqtt is disconnected messages will be discarded
*
* @clientID 0-5, mqtt index client
* @topic - topic without prefix, must start with '/', topic is added thereafter
* @message - payload
* @qos 0-2
* @retain true|false
*/
bool MODEMfreeRTOS::mqtt_pushMessage(uint8_t clientID, const String& topic, const String& message, uint8_t qos, uint8_t retain) {

  if(clientID >= MAX_MQTT_CONNECTIONS)
    return false;
  /*
  if(qos == 0 && !mqtt_connected())
    return false;
  */
  struct MQTT_MSG_TX *pxMessage;

  if( mqttTxQueue != 0 && uxQueueSpacesAvailable(mqttTxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.

     if(!xSemaphoreTake( mqttTxQueueMutex, 2000)){
       xSemaphoreGive(mqttTxQueueMutex);
       Serial.println("Couldn't get mqttTxQueueMutex");
       return false;
     }

     pxMessage = &tx_mqtt_msg[uxQueueMessagesWaiting(mqttTxQueue)];
     memset(pxMessage->topic,0,sizeof(pxMessage->topic));
     memset(pxMessage->data,0,sizeof(pxMessage->data));

     String topic_ = mqtt[clientID].prefix+topic;
     memcpy(pxMessage->topic,topic_.c_str(),topic_.length());
     memcpy(pxMessage->data,message.c_str(),message.length());
     pxMessage->qos = qos;
     pxMessage->retain = retain;
     pxMessage->clientID = clientID;

     bool res = xQueueSendToBack( mqttTxQueue, ( void * ) &pxMessage, ( TickType_t ) 0 ) == true;
     xSemaphoreGive(mqttTxQueueMutex);
     return res;
  }

  return false;

}

/*
* private method
* checks if queue has messages to be sent.
* If there is it will send using an available network interface
*/
void MODEMfreeRTOS::mqtt_sendMessage(){

  struct MQTT_MSG_TX *pxMessage;

  if(!xSemaphoreTake( mqttTxQueueMutex, 200)){
    return;
  }

  if( mqttTxQueue != 0 && uxQueueMessagesWaiting(mqttTxQueue) > 0){
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

      xSemaphoreGive(mqttTxQueueMutex);

      if(clientID >= MAX_MQTT_CONNECTIONS){
        Serial.println("invalid mqtt clientID");
        return;
      }

      #ifdef ENABLE_LTE
        if(!modem.MQTT_connected(clientID)){
          if(qos != 0)
            mqtt_pushMessage(clientID,topic,data,qos,retain);
        }else{
          #ifdef DEBUG_MQTT_MSG
          Serial.println("[mqtt] >> "+topic + " : " + data);
          #endif
          modem.MQTT_publish(clientID,++mqtt[clientID].msg_id,qos,retain,topic,data);
        }
      #else
        if(clientID == 0){
          if(!mqtt1.isMqttConnected()){
            if(qos != 0)
              mqtt_pushMessage(clientID,topic,data,qos,retain);
          }else{
            #ifdef DEBUG_MQTT_MSG
            Serial.println("[mqtt][1] >> "+topic + " : " + data);
            #endif
            mqtt1.publish(topic,data,retain);
          }
        }if(clientID == 1){
          if(!mqtt2.isMqttConnected()){
            if(qos != 0)
              mqtt_pushMessage(clientID,topic,data,qos,retain);
          }else{
            #ifdef DEBUG_MQTT_MSG
            Serial.println("[mqtt][2] >> "+topic + " : " + data);
            #endif
            mqtt2.publish(topic,data,retain);
          }
        }
      #endif
    }

  }
  xSemaphoreGive(mqttTxQueueMutex);
}

/*
* private method, called on received message. It adds message to queue
*/
void mqtt_enqueue_msg(uint8_t clientID, String topic, String payload){

  #ifdef DEBUG_MQTT_MSG
  Serial.println("[mqtt] << ["+String(clientID)+"] "+topic + " : " + payload);
  #endif
  struct MQTT_MSG_RX *pxMessage;

  if( mqttRxQueue != 0 && uxQueueSpacesAvailable(mqttRxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     pxMessage = &rcv_mqtt_msg[uxQueueMessagesWaiting(mqttRxQueue)];
     memset(pxMessage->topic,0,sizeof(pxMessage->topic));
     memset(pxMessage->data,0,sizeof(pxMessage->data));

     memcpy(pxMessage->topic,topic.c_str(),topic.length());
     memcpy(pxMessage->data,payload.c_str(),payload.length());
     pxMessage->clientID = clientID;
     xQueueGenericSend( mqttRxQueue, ( void * ) &pxMessage, ( TickType_t ) 0, queueSEND_TO_BACK );
  }else{
    #ifdef WARNING_MQTT_GW
    Serial.println("mqtt rx no space available");
    #endif
  }
}

#ifndef ENABLE_LTE
void MODEMfreeRTOS::WiFiEvent(WiFiEvent_t event){

  switch (event) {
    case SYSTEM_EVENT_WIFI_READY:
        #ifdef DEBUG_WIFI
        Serial.println("WiFi interface ready");
        String mac = WiFi.macAddress();
        // drop ':''
        mac.replace(":", "");
        // lower letters
        mac.toLowerCase();
        mac_address = mac;
        #endif
        break;
    case SYSTEM_EVENT_SCAN_DONE:
        #ifdef DEBUG_WIFI
        Serial.println("Completed scan for access points");
        #endif
        break;
    case SYSTEM_EVENT_STA_START:
        #ifdef DEBUG_WIFI
        Serial.println("WiFi client started");
        #endif
        break;
    case SYSTEM_EVENT_STA_STOP:
        #ifdef DEBUG_WIFI
        Serial.println("WiFi clients stopped");
        #endif
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        #ifdef DEBUG_WIFI
        Serial.println("Connected to access point");
        #endif
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        #ifdef DEBUG_WIFI
        Serial.println("Disconnected from WiFi access point");
        #endif
        wifi_connected = false;
        if(retry_counter > 900){
          retry_counter *= 3;
        }else retry_counter = 1;
        connect_retry = retry_counter*60*1000;
        break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
        #ifdef DEBUG_WIFI
        Serial.println("Authentication mode of access point has changed");
        #endif
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("Obtained IP address: ");
        #ifdef DEBUG_WIFI
        Serial.println(WiFi.localIP());
        #endif
        ip = WiFi.localIP().toString();
        wifi_connected = true;
        break;
    case SYSTEM_EVENT_STA_LOST_IP:
        #ifdef DEBUG_WIFI
        Serial.println("Lost IP address and IP address is reset to 0");
        #endif
        break;
    case SYSTEM_EVENT_AP_START:
        #ifdef DEBUG_WIFI
        Serial.println("WiFi access point started");
        #endif
        break;
    case SYSTEM_EVENT_AP_STOP:
        #ifdef DEBUG_WIFI
        Serial.println("WiFi access point  stopped");
        #endif
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        #ifdef DEBUG_WIFI
        Serial.println("Client connected");
        #endif
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        #ifdef DEBUG_WIFI
        Serial.println("Client disconnected");
        #endif
        break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
        #ifdef DEBUG_WIFI
        Serial.println("Assigned IP address to client");
        #endif
        break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
        #ifdef DEBUG_WIFI
        Serial.println("Received probe request");
        #endif
        break;
    case SYSTEM_EVENT_GOT_IP6:
        #ifdef DEBUG_WIFI
        Serial.println("IPv6 is preferred");
        #endif
        break;
  }
}
#endif
