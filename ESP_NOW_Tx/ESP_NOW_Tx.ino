/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x94, 0xb9, 0x7e, 0x5f, 0x2c, 0x20};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int voltage;
  int cmd;
  String data;
} struct_message;

// Create a struct_message called myData
struct_message myData;

String readString, str_cmd, str_volt;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial1.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {

  while (Serial1.available()) {
  //delay to allow buffer to fill 
    delay(3);
    if (Serial1.available() >0) {
      char c = Serial1.read();  //gets one byte from serial buffer
      readString += c; //makes the string readString
    } 
  }

  if (readString.length() >0) {
      Serial.println(readString); //see what was received
      // expect a string like 07002100 containing the two servo positions      
      //servo1 = readString.substring(0, 4); //get the first four characters
      //servo2 = readString.substring(4, 8); //get the next four characters 
      // Set values to send
      //strcpy(myData.data, "THIS IS A CHAR");
      str_volt = readString.substring(0, 2);
      str_cmd  = readString.substring(3, 4);
    
      myData.voltage = str_volt.toInt();
      myData.cmd = str_cmd.toInt();
      myData.data = readString.substring(5,19);
      readString="";

      // Send message via ESP-NOW
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
       
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
  }


  


}
