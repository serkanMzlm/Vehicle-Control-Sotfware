#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h>

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "joy"
#define REMOTEXY_WIFI_PASSWORD "135798642"
#define REMOTEXY_SERVER_PORT 6377

#define HEADER_BYTE   27
#define FOOTHER_BYTE  71
#define DELAY  10

typedef enum {
    S_HEADER, S_LX, S_LY, S_RX, S_RY, 
    S_PWR, UNDEFINED, S_FOOTHER, S_ALL
}Serial_e;

// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 51 bytes
  { 255,5,0,3,0,44,0,16,24,0,5,37,0,33,30,30,2,27,31,5,
  52,70,33,30,30,2,27,31,65,121,1,1,9,9,2,1,78,1,21,5,
  132,26,31,1,79,78,0,79,70,70,0 };
  
struct {

    // input variables
  int8_t left_joy_x; // from -100 to 100  
  int8_t left_joy_y; // from -100 to 100  
  int8_t right_joy_x; // from -100 to 100  
  int8_t right_joy_y; // from -100 to 100  
  uint8_t power; // =1 if switch ON and =0 if OFF 

    // output variables
  uint8_t RGB_r; // =0..255 LED Red brightness 
  uint8_t RGB_g; // =0..255 LED Green brightness 
  uint8_t RGB_b; // =0..255 LED Blue brightness 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

uint8_t  buffer[S_ALL];
uint64_t prev_time;

void transmitBuffer();
void updateData();

void setup(){
  RemoteXY_Init ();
  Serial.begin(9600);
  buffer[S_HEADER] = HEADER_BYTE;
  buffer[S_FOOTHER] = FOOTHER_BYTE;
  RemoteXY.RGB_r = 255;
  RemoteXY.RGB_g = 0;
  RemoteXY.RGB_b = 255;
  prev_time = millis();
}

void loop(){ 
  RemoteXY_Handler ();
  updateData();
  transmitBuffer();
}

void transmitBuffer(){
  if((millis() - prev_time) > DELAY){
    prev_time = millis();
    Serial.write(buffer, S_ALL);
  }
}

void updateData(){
  buffer[S_LX] = (uint8_t)map(RemoteXY.left_joy_x ,-100, 100, 0, 200);
  buffer[S_LY] = (uint8_t)map(RemoteXY.left_joy_y ,-100, 100, 0, 200);
  buffer[S_RX] = (uint8_t)map(RemoteXY.right_joy_x ,-100, 100, 0, 200);
  buffer[S_RY] = (uint8_t)map(RemoteXY.right_joy_y ,-100, 100, 0, 200);
  buffer[S_PWR] = RemoteXY.power;
  buffer[UNDEFINED] = 0;
}
