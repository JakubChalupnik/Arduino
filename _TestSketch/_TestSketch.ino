#include <ESP8266WiFi.h>
// #include <WiFi.h>


#ifndef STASSID
#define STASSID "kubikovo"
#define STAPSK  ""
#endif

const char * ssid = STASSID; // your network SSID (name)
const char * pass = STAPSK;  // your network password


void setup() {
  Serial.begin(115200);

  WiFi.scanNetworks();
//  WiFi.connectToNetwork();

// We start by connecting to a WiFi network
//  Serial.println("Disconnecting...\n");
//  WiFi.disconnect();
//  delay(3000);
//  ESP.reset();

}



void loop() {

}
