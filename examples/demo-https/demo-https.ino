

#include "credentials.h"
#include "modem-freeRTOS.hpp"
#include <ArduinoJson.h>

// HARDWARE
#define PWKEY 4

// CONTEXT
#define CLIENTID 0
#define SSLCLIENTID 0
#define CONTEXTID 1


DynamicJsonDocument doc(512);


MODEMfreeRTOS mRTOS;

HTTP_HEADER_MSG* msg_header;
HTTP_BODY_MSG* msg_body;

void network_lte_task(void *pvParameters);
void network_lte_task(void *pvParameters){
  (void) pvParameters;

  mRTOS.init(SETTINGS_NB_COPS,AUTO,PWKEY); // initialize modem

  if(!mRTOS.set_ssl(CONTEXTID))
    Serial.println("failing configuring ssl");
  mRTOS.set_context(1,SETTINGS_NB_APN,SETTINGS_NB_USERNAME,SETTINGS_NB_PASSWORD);

  for(;;){
    mRTOS.loop();
  }
}

String do_request(String host, String path, String method, String token, String body, bool json){

  mRTOS.https_pushMessage(CONTEXTID,CLIENTID,SSLCLIENTID,host,path,method,token,body,json);

  uint32_t timeout = millis() + 60000; // 30 seconds timeout
  while(timeout > millis()){
    msg_header = mRTOS.http_header_getNextMessage(msg_header);
    if(msg_header != NULL){
      Serial.printf("client [%d] %s \n",msg_header->clientID,msg_header->http_response.c_str());
      if(msg_header->http_response.indexOf("200") > 0){
        Serial.printf("http body len %d \n",msg_header->body_len);
        uint32_t len = 0;
        char* data = (char*)malloc(msg_header->body_len);
        while(msg_header->body_len != len){
          msg_body = mRTOS.http_body_getNextMessage(msg_body);

          if(msg_body != NULL){

            if(data != nullptr){
              for(uint16_t i=0;i<msg_body->data_len;i++){
                data[len+i] = msg_body->data[i];
              }
            }

            len += msg_body->data_len;
            Serial.printf("http total bytes read of body data: %d \n",len);

          }
          delay(100); // use delay to moderate concurrency access to queues
        }
        Serial.println("http all data was read");

        String body = "";
        for(uint16_t i=0;i<len;i++){
          //Serial.print(data[i]);
          body += data[i];
        }

        //String body = String(data);
        free(data);
        return body;
      }
    }
    delay(100); // use delay to moderate concurrency access to queues
  }
  return "";
}

void core(void *pvParameters);
void core(void *pvParameters){
  (void) pvParameters;

  delay(60000);

  String host = HTTPS_HOST;
  String path = "/api/auth"; // must start with '/'
  String method = "POST";
  String token = "";
  String body = HTTPS_BODY;
  bool json = false;
  String json_str = do_request(host,path,method,token,body,json);

  DeserializationError error = deserializeJson(doc, json_str.c_str());

  if(error){
    Serial.println("Error parsing JSON: "+String(error.c_str()));
  }

  if(doc.containsKey("status")){
    if(doc["status"] == "ok"){
      method = "GET";
      if(doc.containsKey("data") && doc["data"].containsKey("session")){
        const char* key = doc["data"]["session"];
        Serial.println(String(key));
        token = "X-Session:"+String(key);
        String entities = do_request(host,"/api/entities",method,token,"",false);
        Serial.println(entities);
      }
    }else if(doc["status"] != "ok")
      Serial.println("response came with an error");
  }else{
    Serial.println("status key not found");
  }

  for(;;){



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
