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

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int voltage;
  int cmd;
  String msg;
} struct_message;

// Create a struct_message called myData
struct_message myData;

uint8_t msgRcvd = 0;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  msgRcvd = 1;
}

void displayMsg() {
  display.clearDisplay();
  display.setCursor(0,10);

  display.print("V: ");
  display.println(myData.voltage);
  display.print("CMD: ");
  display.println(myData.cmd);
  display.print("A: ");
  display.println(myData.msg);
  display.display(); // actually display all of the above
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  display.begin(0x3C, true); // Address 0x3C default
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();
  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setRotation(1);
  display.display(); // actually display all of the above

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    display.println("Error initializing ESP-NOW");
    display.display();
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  if(msgRcvd == 1) {
    displayMsg();
    msgRcvd = 0;
  }


}
