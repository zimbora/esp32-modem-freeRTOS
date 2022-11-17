

#include "credentials.h"
#include "modem-freeRTOS.hpp"

// HARDWARE
#define PWKEY 4

// CONTEXT
#define CLIENTID 0
#define CONTEXTID 1

// freeRTOS
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//#define CORE_STACK_SIZE 2048*8
//StackType_t xStack_core[ CORE_STACK_SIZE ];
//StaticTask_t xTaskBuffer_core;

MODEMfreeRTOS mRTOS;

HTTP_HEADER_MSG* msg_header;
HTTP_BODY_MSG* msg_body;

void network_lte_task(void *pvParameters);
void network_lte_task(void *pvParameters){
  (void) pvParameters;

  mRTOS.init(SETTINGS_NB_COPS,AUTO,PWKEY); // initialize modem
  mRTOS.set_context(1,SETTINGS_NB_APN,SETTINGS_NB_USERNAME,SETTINGS_NB_PASSWORD);

  for(;;){
    mRTOS.loop();
  }
}

void core(void *pvParameters);
void core(void *pvParameters){
  (void) pvParameters;

  delay(5000);

  String host = HTTP_HOST;
  String path = HTTP_PATH; // must start with '/'
  mRTOS.http_pushMessage(CONTEXTID,CLIENTID,host,path,"GET");

  for(;;){

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
        for(uint16_t i=0;i<len;i++){
          Serial.print(data[i]);
        }
        free(data);
      }
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
