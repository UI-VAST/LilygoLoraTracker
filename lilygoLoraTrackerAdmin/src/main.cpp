#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>         
#include <LoRa.h>
#include <main.h>
#include <websocket.h>
#include <HardwareSerial.h>
#include "LoRa_E32.h"

//#include <common/mavlink.h>
//#include <AsyncUDP.h>



long lastSendTime = 0;
int count = 1000;        // last send time
#define interval 1000    // interval between sends


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

HardwareSerial ebyteSerial(1);
LoRa_E32 e32(ebyte_tx, ebyte_rx, &ebyteSerial, ebyte_aux, ebyte_m0, ebyte_m1, UART_BPS_RATE_9600, SERIAL_8N1);


JsonDocument data;
String jsonString;

const int port = 14550;


//SimpleMavlinkDrone drone(&udp);


uint8_t badPacket = 0;
bool is_synced = 0;

packet RXp;
packet RXp2;

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
  init_ebyte();

  update_clients(&RXp, &RXp2);
}
 

void loop() {
  if (millis() - lastSendTime > count) {
    slow_loop(); 
    count = interval;    
  }
  fast_loop();
}

void Send_packet(){
    //uint8_t payload[sizeof(TXp)];
    //memcpy(&payload[0], &TXp, sizeof(TXp));
    
    long start = millis();
    LoRa.beginPacket();
    //LoRa.write(payload, sizeof(payload));
    LoRa.write(get_state());
    LoRa.endPacket();
    Serial.println((millis()- start));
    Serial.println("Packet Sent!");
    
    ebyteSerial.write(0xff);
    ebyteSerial.write(0xff);
    ebyteSerial.write(0x04);
    ebyteSerial.write(get_state());


}

void Recieve_packet(int len)
{
    if(len == sizeof(RXp)){
      uint8_t payload[len];
      LoRa.readBytes(payload, len);
      memcpy(&RXp, &payload[0], sizeof(RXp));

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
  LoRa.setTxPower(txPower,1);
  LoRa.setSpreadingFactor(spreadFactor);

}

void init_oled(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
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

void update_clients(packet* p, packet* p2){
  data["lat"] = String(p->lat, 8);
  data["lon"] =  String(p->lon, 8);
  data["alt"] = String(p->alt);
  data["sats"] = String(p->sats);
  data["pcount"] = String(p->pcount);

  data["lat2"] = String(p2->lat, 8);
  data["lon2"] =  String(p2->lon, 8);
  data["alt2"] = String(p2->alt);
  data["sats2"] = String(p2->sats);
  data["pcount2"] = String(p2->pcount);

  data["rssi"] = String(LoRa.packetRssi());
  data["snr"] = String(LoRa.packetSnr());
  data["sync"] = String(is_synced);
  serializeJson(data, jsonString);
  notifyClients(jsonString);
}

void Recieve_ebyte(){
  if (e32.available() > 0){
		ResponseStructContainer rsc = e32.receiveMessage(sizeof(RXp));
		RXp2 = *(packet*) rsc.data;
    Serial.println(rsc.status.getResponseDescription());
		rsc.close();

	}
}

//1sec
void slow_loop(){   
     
    if(RXp.state != get_state()){
      //sync states between boards
      is_synced = 0;
      Serial.println("Syncing states");
      Send_packet();
    }
    else{
      is_synced = 1;
      //Serial.println("boards are synced!");
    }

    
    update_clients(&RXp, &RXp2);

  
    
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
  Recieve_ebyte();
 
}

void init_ebyte(){
    e32.begin();
  	ResponseStructContainer c;
    c = e32.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    configuration.ADDL = 0x0;
    configuration.ADDH = 0x0;
    configuration.CHAN = 0x04;
    configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
	  configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;
    configuration.OPTION.transmissionPower = POWER_30;
    configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
    configuration.OPTION.fec = FEC_1_ON;

    e32.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    printParameters(configuration);
    c.close();
}

void printParameters(struct Configuration configuration) {
	Serial.println("----------------------------------------");

	Serial.print(F("HEAD : "));  Serial.print(configuration.HEAD, BIN);Serial.print(" ");Serial.print(configuration.HEAD, DEC);Serial.print(" ");Serial.println(configuration.HEAD, HEX);
	Serial.println(F(" "));
	Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, DEC);
	Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, DEC);
	Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
	Serial.print(F("SpeedUARTDatte  : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRate());
	Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRate());

	Serial.print(F("OptionTrans        : "));  Serial.print(configuration.OPTION.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFixedTransmissionDescription());
	Serial.print(F("OptionPullup       : "));  Serial.print(configuration.OPTION.ioDriveMode, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getIODroveModeDescription());
	Serial.print(F("OptionWakeup       : "));  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
	Serial.print(F("OptionFEC          : "));  Serial.print(configuration.OPTION.fec, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFECDescription());
	Serial.print(F("OptionPower        : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());

	Serial.println("----------------------------------------");

}

void printModuleInformation(struct ModuleInformation moduleInformation) {
	Serial.println("----------------------------------------");
	Serial.print(F("HEAD BIN: "));  Serial.print(moduleInformation.HEAD, BIN);Serial.print(" ");Serial.print(moduleInformation.HEAD, DEC);Serial.print(" ");Serial.println(moduleInformation.HEAD, HEX);

	Serial.print(F("Freq.: "));  Serial.println(moduleInformation.frequency, HEX);
	Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
	Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
	Serial.println("----------------------------------------");

}



