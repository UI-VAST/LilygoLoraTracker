#include <Arduino.h>
#include <websocket.h>
#include <main.h>
//ground staiton

#define wifi_station

// Replace with your network credentials
const char* ssid = "your wifi ssid";
const char* password = "your wifi password";

uint8_t s_state = close_servo;
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi() {

  #ifdef wifi_station
    WiFi.mode(WIFI_AP);
  
    uint8_t mac[8];
    esp_efuse_mac_get_default(mac);
    uint8_t id = (mac[7]+mac[6]+mac[5]+mac[4]+mac[3]+mac[2]+mac[1]+mac[0]);

    String ssid ="vast_tracker" + String(id);
    WiFi.softAP(ssid, "vast1234");
    Serial.println(WiFi.softAPBroadcastIP().toString());
  #else
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
    }
    Serial.println(WiFi.localIP());
  #endif
  

}

void notifyClients(String data) {
  if(ws.availableForWriteAll()){
    ws.textAll(data);
    //ws.binaryAll(data, len);
  }
  
}
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String message = (char*)data;
    if (strcmp((char*)data, "cd") == 0) {
      //if it is, send current sensor readings
      Serial.println("Cutdown recieved!");
      if(s_state == open_servo){
        s_state = close_servo;
        return;
      }
      if(s_state == close_servo){
        s_state = open_servo;
        return;
      }
      
      //notifyClients(sensorReadings);
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}


void init_server() {
  initWiFi();
  initSPIFFS();
  initWebSocket();

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  // Start server
  server.begin();

}

uint8_t get_state(){
  return s_state;
}

void set_state(uint8_t s){
  s_state = s;
}