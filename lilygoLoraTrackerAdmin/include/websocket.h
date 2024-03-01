#include <ArduinoJson.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"




void init_server();
void initWebSocket();
void initSPIFFS();
void initWiFi();
void notifyClients(String sensorReadings);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void initWebSocket();
uint8_t get_state();
void set_state(uint8_t s);