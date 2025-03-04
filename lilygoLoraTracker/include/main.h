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

#define led 25

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define default_state 0x00
#define open_servo 0x01
#define close_servo 0x02

struct packet{
  float lat;
  float lon;
  float alt;
  uint8_t sats;
  uint8_t pcount;
  uint8_t state;
  
};




void Recieve_packet(int len);
void init_lora();
void init_oled();
void update_clients(packet* p);
void slow_loop();
void fast_loop();
void Recieve_ebyte();


