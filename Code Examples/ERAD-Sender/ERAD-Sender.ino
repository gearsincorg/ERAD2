/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

#include <esp_now.h>
#include <WiFi.h>

#define axialPin   35
#define yawPin     34
#define CAL_SAMPLES 64


// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
// uint8_t broadcastAddress1[] = {0x88, 0x57, 0x21, 0xA0, 0x07, 0x98};  // MakerBase
uint8_t broadcastAddress1[] = {0x10, 0x51, 0xDB, 0x57, 0x46, 0x58};  // RoboticWorx

typedef struct struct_message {
  float cmd;
} struct_message;


struct_message fl_drive;
struct_message fr_drive;
struct_message bl_drive;
struct_message br_drive;

int ax_offset = 0;
int yaw_offset = 0;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void  calibrateJS(){
  long int ax_sum  = 0;
  long int yaw_sum = 0;

  for (int i = 0; i < CAL_SAMPLES; i++){
    ax_sum  += analogRead(axialPin);
    yaw_sum += analogRead(yawPin);
  }

  ax_offset  = (int)(ax_sum  / CAL_SAMPLES);
  yaw_offset = (int)(yaw_sum / CAL_SAMPLES);
} 

void setup() {
  Serial.begin(115200);
  calibrateJS();
 
  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // register peers   
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
float deadband(float js){
  if (js > 0.1) {
    js = (js - 0.1) * 1.11;
  } else if (js < -0.1) {
    js = (js + 0.1) * 1.11;
  } else {
    js = 0;
  }

  return js;
}

void loop() {

  float ax  = (float)(constrain(analogRead(axialPin) - ax_offset, -2000, 2000))  / 2000;
  float yaw = (float)(constrain(analogRead(yawPin) - yaw_offset, -2000, 2000)) / 2000;

  ax = deadband(ax);
  yaw = deadband(yaw);
  
  fl_drive.cmd = ax - yaw ;
  fr_drive.cmd = ax + yaw ;

  esp_err_t result = esp_now_send(0, (uint8_t *) &fl_drive, sizeof(struct_message));

  delay(50);
}
