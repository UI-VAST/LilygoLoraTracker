#define OLED_ADDRESS    0x3C
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_RST    -1

#define CONFIG_MOSI 27
#define CONFIG_MISO 19
#define CONFIG_CLK  5
#define CONFIG_NSS  18
#define CONFIG_RST  23
#define CONFIG_DIO0 26

#define SDCARD_MOSI 15
#define SDCARD_MISO 2
#define SDCARD_SCLK 14
#define SDCARD_CS   13

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels



/*
34
35
12
13
14
15
0
4
*/
#define GPSBaud 9600
#define ebyteBaud 9600

#define cutdown_servo_pin 4

#define gps_rx 13
#define gps_tx 15

#define ebyte_rx 12
#define ebyte_tx 14
#define ebyte_m1 25
#define ebyte_m0 0
#define ebyte_aux 35

#define E32_TTL_1W

#define default_state 0x00
#define open_servo 0x01
#define close_servo 0x02

struct packet{
  float lat;
  float lng;
  float alt;
  uint8_t sats;
  uint8_t pcount;
  uint8_t state;
};


void Send_packet();
void Recieve_packet(int len);
void init_lora();
void init_oled();
void slow_loop();
void fast_loop();
void read_gps();
void init_servo();

void init_ebyte();
void printModuleInformation(struct ModuleInformation moduleInformation);
void printParameters(struct Configuration configuration);
