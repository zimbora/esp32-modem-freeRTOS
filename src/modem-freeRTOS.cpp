

#include "modem-freeRTOS.hpp"

#define MQTT_RX_QUEUE_SIZE 10
#define MQTT_TX_QUEUE_SIZE 10

#define TCP_RX_QUEUE_SIZE 2
#define TCP_TX_QUEUE_SIZE 2

#define HTTP_RX_QUEUE_SIZE 1 // !! do not change it
#define HTTP_TX_QUEUE_SIZE 1 // !! do not change it


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
HTTP_REQUEST tx_http_request[HTTP_TX_QUEUE_SIZE];
SemaphoreHandle_t httpTxQueueMutex;

// MQTT
QueueHandle_t mqttRxQueue;
QueueHandle_t mqttTxQueue;
MQTT_MSG rcv_mqtt_msg[MQTT_RX_QUEUE_SIZE];
MQTT_MSG tx_mqtt_msg[MQTT_TX_QUEUE_SIZE];
SemaphoreHandle_t mqttTxQueueMutex;
MQTT_SETUP mqtt[MAX_MQTT_CONNECTIONS];


MODEMBGXX modem;

void (*tcpOnConnect)(uint8_t clientID);
void (*mqttOnConnect)();
bool (*mqtt_callback)(uint8_t,String,String);
bool mqtt_parse_msg(uint8_t clientID, String topic, String payload){
  mqtt_enqueue_msg(clientID,topic,payload);
  return true;
}

/*
* init hw interface and class
*
* @cops - operator selection, if 0 let modem search automatically
* @mode - radio technology GSM:1/GPRS:2/NB:3/CATM1:4/AUTO:5 (auto for any available technology)
* @pwkey - gpio to switchOn module
*/
void MODEMfreeRTOS::init(uint16_t cops, uint8_t mode, uint8_t pwkey){

  mqttRxQueue = xQueueCreate( MQTT_RX_QUEUE_SIZE, sizeof( struct MQTT_MSG * ) );
  mqttTxQueue = xQueueCreate( MQTT_TX_QUEUE_SIZE, sizeof( struct MQTT_MSG * ) );

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

  modem.init_port(115200,SERIAL_8N1);
  modem.init(mode,cops,pwkey);

  // init contexts
}

/*
* Executes whatever is needed
* call it always that is possible
*/
void MODEMfreeRTOS::loop(){


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
      if(tx_http_request[i].contextID == 0)
        continue;

      if(!modem.has_context(tx_http_request[i].contextID))
        modem.open_pdp_context(tx_http_request[i].contextID);
    }

    modem.log_status();
  }

  // tcp
  tcp_sendMessage();
  tcp_checkMessages();

  // mqtt
  mqtt_sendMessage();

  // http
  http_execute_requests();

  delay(100);
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

  return modem.setup(contextID,apn,user,pwd);
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
  modem.tcp_set_callback_on_close(callback2);
}

/*
* private method
* checks if there are data on buffers. Buffers are filled automatically and must be read with high frequency
* if a long response is being received
* Data is then sent to queue. Use tcp_getNextMessage to read data
*/
bool MODEMfreeRTOS::tcp_checkMessages(){

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
    log("tcp rx no space available");
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

      if(!modem.tcp_connected(clientID)){
        tcp_pushMessage(clientID,pxMessage->data,pxMessage->data_len);
        return;
      }else{
        //log("[tcp] >> "+topic + " : " + data);
        modem.tcp_send(clientID,pxMessage->data,pxMessage->data_len);
      }
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
* use it to do http requests
*
* @contextID 1-11, context id
* @clientID 0-5, tcp index client
* @host
* @port
* @path
* @method
*/
bool MODEMfreeRTOS::http_pushMessage(uint8_t contextID, uint8_t clientID, String host, String path, String method)  {

  if(clientID >= MAX_TCP_CONNECTIONS)
    return false;
  /*
  if(qos == 0 && !tcp_connected())
    return false;
  */
  struct HTTP_REQUEST *pxMessage;

  if( httpTxQueue != 0 && uxQueueSpacesAvailable(httpTxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     if(!xSemaphoreTake( httpTxQueueMutex, 2000)){
       xSemaphoreGive(httpTxQueueMutex);
       return false;
     }

     pxMessage = &tx_http_request[uxQueueMessagesWaiting(httpTxQueue)];

     pxMessage->contextID = contextID;
     pxMessage->clientID = clientID;
     pxMessage->host = host;
     pxMessage->path = path;
     pxMessage->method = method;

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

  struct HTTP_REQUEST *pxMessage;

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
      if(clientID >= MAX_TCP_CONNECTIONS){
        Serial.println("invalid tcp clientID");
        return;
      }

      if(!modem.has_context(contextID)){
        xSemaphoreGive(httpTxQueueMutex);
        http_pushMessage(contextID,clientID,pxMessage->host,pxMessage->path,pxMessage->method);
        return;
      }else{
        //log("[tcp] >> "+topic + " : " + data);
        xSemaphoreGive(httpTxQueueMutex);
        http_get_request(contextID,clientID,pxMessage->host,pxMessage->path);
        return;
      }
    }

  }
  xSemaphoreGive(httpTxQueueMutex);
}

/*
* private method
*/
bool MODEMfreeRTOS::http_get_request(uint8_t contextID, uint8_t clientID, String host, String path){


  if(!modem.http_do_request(host,path,clientID,contextID)){
    Serial.printf("http request to: %s%s has failed..\n",host.c_str(),path.c_str());
    return false;
  }
  if(!modem.http_wait_response(clientID)){
    Serial.println("http request: no response received");
    return false;
  }

  if(modem.http_response_status().indexOf("200") > -1){
    Serial.println("http request: OK");
    uint16_t len = modem.http_get_body_size();
    Serial.printf("http body size: %d \n",len);

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
            Serial.printf("send %d bytes to queue \n",len_read);
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
    Serial.println("response: "+modem.http_response_status());
    http_enqueue_header_msg(clientID, 0, modem.http_md5().c_str(), modem.http_response_status());
  }

}


bool MODEMfreeRTOS::http_enqueue_header_msg(uint8_t clientID, uint16_t body_len, const char* md5, String http_response){

  //#ifdef DEBUG_tcp_GW
  Serial.println("[http header] << ["+String(clientID)+"] size:" + String(body_len));
  //#endif
  struct HTTP_HEADER_MSG *pxMessage;

  if( httpHeaderRxQueue != 0 && uxQueueSpacesAvailable(httpHeaderRxQueue) > 0){
     // Send a pointer to a struct AMessage object.  Don't block if the
     // queue is already full.
     pxMessage = &rcv_http_header_msg[uxQueueMessagesWaiting(httpHeaderRxQueue)];

     pxMessage->clientID = clientID;
     pxMessage->body_len = body_len;
     memcpy(pxMessage->md5,md5,16);
     pxMessage->http_response = http_response;
     xQueueGenericSend( httpHeaderRxQueue, ( void * ) &pxMessage, ( TickType_t ) 0, queueSEND_TO_BACK );
  }else{
    #ifdef WARNING_HTTP_GW
    log("tcp rx no space available");
    #endif
    return false;
  }
  return true;
}

bool MODEMfreeRTOS::http_enqueue_body_msg(uint8_t clientID, char* data, uint16_t data_len){

  //#ifdef DEBUG_tcp_GW
  Serial.println("[http body] << ["+String(clientID)+"] size:" + String(data_len));
  //#endif
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
    //#ifdef WARNING_HTTP_GW
    Serial.println("http rx no space available");
    //#endif
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

  if(index >= 10)
    return;

  mqtt[clientID].subscribe_topics[index] = topic;
}

/*
* Init mqtt and configures callback to be called when connection is established
*/
void MODEMfreeRTOS::mqtt_setup(void(*callback)()){

  mqttOnConnect = callback;
  mqtt_callback = &mqtt_parse_msg;
  modem.MQTT_init(mqtt_callback);

  for(uint8_t i = 0; i<MAX_MQTT_CONNECTIONS; i++){
    if(mqtt[i].contextID != 0)
      modem.MQTT_setup(i,mqtt[i].contextID,mqtt[i].prefix+mqtt[i].will_topic,mqtt[i].will_payload);
  }
}


/*
* use it to get received messages.
*
* returns a pointer to MQTT_MSG struct containing the received message
* if no message is available it returns NULL
*/
MQTT_MSG* MODEMfreeRTOS::mqtt_getNextMessage(MQTT_MSG *pxRxedMessage){

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
* private method
* checks if queue has messages to be sent.
* If there is it will send using an available network interface
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
      }else{
        //log("[mqtt] >> "+topic + " : " + data);
        modem.MQTT_publish(clientID,++mqtt[clientID].msg_id,qos,retain,topic,data);
      }
    }

  }
  xSemaphoreGive(mqttTxQueueMutex);
}

/*
* private method, called on received message. It adds message to queue
*/
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
