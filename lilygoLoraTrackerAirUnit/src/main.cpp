#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>         
#include <LoRa.h>
#include <main.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "LoRa_E32.h"


long lastSendTime = 0;
int count = 1000;        // last send time
#define interval 1000    // interval between sends

HardwareSerial gpsSerial(1);
HardwareSerial ebyteSerial(2);
LoRa_E32 e32(ebyte_tx, ebyte_rx, &ebyteSerial, ebyte_aux, ebyte_m0, ebyte_m1, UART_BPS_RATE_9600, SERIAL_8N1);
TinyGPSPlus gps;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
Servo myservo;
uint8_t badPacket = 0;

packet TXp;

#define spreadFactor 8
#define txPower 19

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello World!");

  gpsSerial.begin(GPSBaud, SERIAL_8N1, gps_rx, gps_tx);

  init_lora();
  LoRa.dumpRegisters(Serial);

  init_oled();
  init_servo();
  init_ebyte();
  TXp.lat = 23.324234;
  TXp.lng = 43.42564;
  TXp.alt = 23432;
  TXp.sats = 0;
  TXp.pcount = 0;
  TXp.state = 0x00;
}


void loop() {
  if (millis() - lastSendTime > count) {
    slow_loop(); 
    count = interval;    
  }
  fast_loop();
}


void Send_packet(){
    //load the packet into a byte array
    uint8_t payload[sizeof(TXp)];
    memcpy(&payload[0], &TXp, sizeof(TXp));
    
    //TX on the 915MHz lora
    long start = millis();
    LoRa.beginPacket();
    LoRa.write(payload, sizeof(payload));
    LoRa.endPacket();
    Serial.print((millis()- start));
    Serial.println(" mS 915!");
    
    //TX on the 433MHz lora

    start = millis();
    ResponseStatus rs = e32.sendBroadcastFixedMessage(4, &TXp, sizeof(TXp));
    Serial.println(rs.getResponseDescription());
    Serial.print((millis() - start));
    Serial.println(" mS 433!");

    
}

void Recieve_packet(int len)
{
  if(len > 0){
    TXp.state = LoRa.read();
    Serial.print("Packet Recieved: ");
    Serial.println(TXp.state);

    if(TXp.state == open_servo){
      myservo.write(180);
      Serial.println("Opening Servo");
    }
    if(TXp.state == close_servo){
      myservo.write(0);
      Serial.println("Closing Servo");
    }
  }


}

void Recieve_ebyte(){
  if (e32.available() > 0){

		ResponseStructContainer rsc = e32.receiveMessage(1);
		TXp.state = *(uint8_t*)rsc.data;
		rsc.close();

    if(TXp.state == open_servo){
      myservo.write(180);
      Serial.println("Opening Servo");
    }
    if(TXp.state == close_servo){
      myservo.write(0);
      Serial.println("Closing Servo");
    }

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
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font


  display.println("Hello World!");
  display.display();
  display.clearDisplay();
}

//1sec
void slow_loop(){

    TXp.pcount++;

    Send_packet();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("RSSI: " + String(LoRa.rssi()));
    display.println("Packet RSSI: " + String(LoRa.packetRssi()));
    display.display();


    lastSendTime = millis();
}

void init_ebyte(){
    e32.begin();
  	ResponseStructContainer c;
    c = e32.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    configuration.ADDL = 2;
    configuration.ADDH = 0;
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

void fast_loop(){
  Recieve_packet(LoRa.parsePacket());
  read_gps();
  Recieve_ebyte(); 
}

void read_gps(){
  if(gpsSerial.available() > 0){
    //Serial.println("gps avalible");
    if(gps.encode(gpsSerial.read())){
      //Serial.println("gps decoded");
      if(gps.location.isValid()){
        digitalWrite(LED_BUILTIN, HIGH);
        TXp.lat = gps.location.lat();
        TXp.lng = gps.location.lng();
        //Serial.println("location valid");
      }
      if(gps.altitude.isValid()){
        TXp.alt = gps.altitude.meters();
      }
      if(gps.satellites.isValid()){
        TXp.sats = gps.satellites.value();
      }
    }
  }
}

void init_servo(){
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(cutdown_servo_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object 
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




