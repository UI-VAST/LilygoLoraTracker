#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>         
#include <LoRa.h>
#include <main.h>
#include <websocket.h>

long lastSendTime = 0;
int count = 1000;        // last send time
#define interval 1000    // interval between sends


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

JsonDocument data;
String jsonString;

uint8_t badPacket = 0;

packet RXp;
packet TXp;

#define spreadFactor 8
#define txPower 19

void setup() {
  Serial.begin(115200);             
  while (!Serial);
  Serial.println("Hello World!");

  init_lora();
  LoRa.dumpRegisters(Serial);

  
  init_server();
  init_oled();
  
  TXp.pcount = 0;
  TXp.state = 0x00;
  update_clients(&TXp);
}


void loop() {
  if (millis() - lastSendTime > count) {
    slow_loop(); 
    count = interval;    
  }
  fast_loop();
}

void Send_packet(){
    uint8_t payload[sizeof(TXp)];
    memcpy(&payload[0], &TXp, sizeof(TXp));
    long start = millis();
    LoRa.beginPacket();
    LoRa.write(payload, sizeof(payload));
    LoRa.endPacket();
    Serial.println((millis()- start));
    Serial.println("Packet Sent!");
}

void Recieve_packet(int len)
{
    if(len == sizeof(RXp)){
      uint8_t payload[len];
      LoRa.readBytes(payload, len);
      memcpy(&RXp, &payload[0], sizeof(TXp));

      Serial.println("Packet Recieved!");
      
      /*
      display.clearDisplay();
      display.setCursor(0, 0); 
      display.println("Packet Count: " + String(RXp.pcount));
      display.println("Packet Len: " + String(len));
      display.println("Spread Factor: " + String(spreadFactor));
      display.println("RSSI: " + String(LoRa.packetRssi()));
      display.println("Snr: " + String(LoRa.packetSnr()));
      display.println("Bad Packet: " + String(badPacket));
      display.display();
      */
      
    }
    else if(len > 0){
      badPacket++;
    }

}

void init_lora(){
  SPI.begin(CONFIG_CLK, CONFIG_MISO, CONFIG_MOSI, CONFIG_NSS);
  LoRa.setPins(CONFIG_NSS, CONFIG_RST, CONFIG_DIO0);

  
  if (!LoRa.begin(915E6)) {         // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                   // if failed, do nothing
  }

  LoRa.enableCrc();
  LoRa.setCodingRate4(5);
  LoRa.setSignalBandwidth(125E3);
  //LoRa.setPreambleLength(6);
  LoRa.setTxPower(txPower,0);
  LoRa.setSpreadingFactor(spreadFactor);

}

void init_oled(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font


  display.println("Hello World!");
  display.println("IP: " + WiFi.localIP().toString());
  display.display();
  display.clearDisplay();
}

void update_clients(packet* p){
  data["lat"] = String(p->lat, 8);
  data["lon"] =  String(p->lon, 8);
  data["alt"] = String(p->alt);
  data["sats"] = String(p->sats);
  data["pcount"] = String(p->pcount);
  data["rssi"] = String(LoRa.packetRssi());
  data["snr"] = String(LoRa.packetSnr());
  
  serializeJson(data, jsonString);
  notifyClients(jsonString);
}

//1sec
void slow_loop(){
    update_clients(&RXp);

    TXp.pcount++;

    //Send_packet();
    //display.clearDisplay();
    //display.setCursor(0, 0);
    //display.println("RSSI: " + String(LoRa.rssi()));
    //display.println("Packet RSSI: " + String(LoRa.packetRssi()));
    //display.println("Packet Count: " + String(RXp.pcount));
    //display.println("IP: " + WiFi.localIP().toString());
    //display.display();


    lastSendTime = millis();
}


void fast_loop(){
  Recieve_packet(LoRa.parsePacket());
}



