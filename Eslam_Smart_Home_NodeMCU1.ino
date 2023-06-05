

#if !(defined(ESP8266) /*|| defined(ESP32)*/  )
  #error This code is intended to run on the ESP32/ESP8266 boards ! Please check your Tools->Board setting.
#endif

// Uncomment the following line to enable serial debug output
//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
  #define DEBUG_ESP_PORT Serial
  #define NODEBUG_WEBSOCKETS
  #define NDEBUG
#endif

#if (ESP8266)
  #include <ESP8266WiFi.h>
#elif (ESP32)
  #include <WiFi.h>
#endif

#include "SinricPro_Generic.h"
#include "SinricProSwitch.h"
#include "SinricProMotionsensor.h"

#include <map>

#define WIFI_SSID         "Smart-X"
#define WIFI_PASS         "11a22b33c"
#define APP_KEY           "896e4ffd-9a2d-4249-a077-4ab3fc0df3a0"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        "d16f1ed4-3a6b-4ea7-a95d-7e8573025ccb-9c8d6c95-6592-481b-a7f0-438e79b9e708"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define BAUD_RATE         115200              // Change baudrate to your need

// comment the following line if you use a toggle switches instead of tactile buttons
#define TACTILE_BUTTON 1

#define DEBOUNCE_TIME 250
#define MOTIONSENSOR_ID   "6321b7cf36b44d06d4bb460d"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define MOTIONSENSOR_PIN  D1                 // PIN where motionsensor is connected to

bool myPowerState = true;                     // assume device is turned on
bool lastMotionState = false;
unsigned long lastChange = 0;


typedef struct 
{      
  // struct for the std::map below
  int relayPIN;
  int flipSwitchPIN;
} deviceConfig_t;




// this is the main configuration
// please put in your deviceId, the PIN for Relay and PIN for flipSwitch
// this can be up to N devices...depending on how much pin's available on your device ;)
// right now we have 4 devicesIds going to 4 relays and 4 flip switches to switch the relay manually
std::map<String, deviceConfig_t> devices = 
{
  //{deviceId, {relayPIN,  flipSwitchPIN}}
  {"6321b455fa69c39e7cc9e004", {   D0,  D8 }}, //First floor
  {"6321b5b736b44d06d4bb43d5", {   D2,  D7 }}, //second floor
  {"6321b6a636b44d06d4bb44c4", {   D3,  D6 }}, // 3rd floor
  {"6321b6e136b44d06d4bb450e", {   D4,  D5 }} // Garden
};

typedef struct 
{      
  // struct for the std::map below
  String deviceId;
  bool lastFlipSwitchState;
  unsigned long lastFlipSwitchChange;
} flipSwitchConfig_t;

std::map<int, flipSwitchConfig_t> flipSwitches;    // this map is used to map flipSwitch PINs to deviceId and handling debounce and last flipSwitch state checks
// it will be setup in "setupFlipSwitches" function, using informations from devices map

void setupRelays() 
{
  for (auto &device : devices) 
  {           
    // for each device (relay, flipSwitch combination)
    int relayPIN = device.second.relayPIN; // get the relay pin
    pinMode(relayPIN, OUTPUT);             // set relay pin to OUTPUT
  }
}

void setupFlipSwitches() 
{
  for (auto &device : devices)  
  {                     
    // for each device (relay / flipSwitch combination)
    flipSwitchConfig_t flipSwitchConfig;              // create a new flipSwitch configuration

    flipSwitchConfig.deviceId = device.first;         // set the deviceId
    flipSwitchConfig.lastFlipSwitchChange = 0;        // set debounce time
    flipSwitchConfig.lastFlipSwitchState = false;     // set lastFlipSwitchState to false (LOW)

    int flipSwitchPIN = device.second.flipSwitchPIN;  // get the flipSwitchPIN

    flipSwitches[flipSwitchPIN] = flipSwitchConfig;   // save the flipSwitch config to flipSwitches map
    pinMode(flipSwitchPIN, OUTPUT);                   // set the flipSwitch pin to OUTPUT
  }
}

bool onPowerState(String deviceId, bool &state)
{
  Serial.printf("%s: %s\r\n", deviceId.c_str(), state ? "on" : "off");
  int relayPIN = devices[deviceId].relayPIN; // get the relay pin for corresponding device
  digitalWrite(relayPIN, state);             // set the new relay state
  
  return true;
}

void handleFlipSwitches() 
{
  unsigned long actualMillis = millis();                                          // get actual millis
  
  for (auto &flipSwitch : flipSwitches) 
  {                                         
    // for each flipSwitch in flipSwitches map
    unsigned long lastFlipSwitchChange = flipSwitch.second.lastFlipSwitchChange;  // get the timestamp when flipSwitch was pressed last time (used to debounce / limit events)

    if (actualMillis - lastFlipSwitchChange > DEBOUNCE_TIME) 
    {                    
      // if time is > debounce time...

      int flipSwitchPIN = flipSwitch.first;                                       // get the flipSwitch pin from configuration
      bool lastFlipSwitchState = flipSwitch.second.lastFlipSwitchState;           // get the lastFlipSwitchState
      bool flipSwitchState = digitalRead(flipSwitchPIN);                          // read the current flipSwitch state
      
      if (flipSwitchState != lastFlipSwitchState) 
      {                               
        // if the flipSwitchState has changed...
        
#ifdef TACTILE_BUTTON
        // if the tactile button is pressed
        if (flipSwitchState) 
        {                                                          
#endif
          flipSwitch.second.lastFlipSwitchChange = actualMillis;                  // update lastFlipSwitchChange time
          String deviceId = flipSwitch.second.deviceId;                           // get the deviceId from config
          int relayPIN = devices[deviceId].relayPIN;                              // get the relayPIN from config
          bool newRelayState = !digitalRead(relayPIN);                            // set the new relay State
          digitalWrite(relayPIN, newRelayState);                                  // set the trelay to the new state

          SinricProSwitch &mySwitch = SinricPro[deviceId];                        // get Switch device from SinricPro
          mySwitch.sendPowerStateEvent(newRelayState);                            // send the event
#ifdef TACTILE_BUTTON
        }
#endif
        flipSwitch.second.lastFlipSwitchState = flipSwitchState;                  // update lastFlipSwitchState
      }
    }
  }
}

// setup function for WiFi connection
void setupWiFi() 
{
  Serial.print("\n[Wifi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    delay(250);
  }
  
  Serial.print("\n[WiFi]: IP-Address is ");
  Serial.println(WiFi.localIP());
}
void handleMotionsensor() 
{
  if (!myPowerState) 
    return;                            // if device switched off...do nothing

  unsigned long actualMillis = millis();
  
  if (actualMillis - lastChange < 250) 
    return;          // debounce motionsensor state transitions (same as debouncing a pushbutton)

  bool actualMotionState = digitalRead(MOTIONSENSOR_PIN);   // read actual state of motion sensor

  if (actualMotionState != lastMotionState) 
  {         
    // if state has changed
    Serial.printf("Motion %s\r\n", actualMotionState ? "detected" : "not detected");
    
    lastMotionState = actualMotionState;              // update last known state
    lastChange = actualMillis;                        // update debounce time
    SinricProMotionsensor &myMotionsensor = SinricPro[MOTIONSENSOR_ID]; // get motion sensor device
    myMotionsensor.sendMotionEvent(actualMotionState);
  }
}



void setupSinricPro()
{
   // add device to SinricPro
  SinricProMotionsensor& myMotionsensor = SinricPro[MOTIONSENSOR_ID];

  // set callback function to device
  

  for (auto &device : devices)
  {
    const char *deviceId = device.first.c_str();
    SinricProSwitch &mySwitch = SinricPro[deviceId];
    mySwitch.onPowerState(onPowerState);
    myMotionsensor.onPowerState(onPowerState);
  }

  SinricPro.begin(APP_KEY, APP_SECRET);
  SinricPro.restoreDeviceStates(true);
}

// main setup function
void setup() 
{
  
  Serial.begin(BAUD_RATE); 
  while (!Serial);
  
  Serial.println("\nStarting MultiSwitch_advance on " + String(ARDUINO_BOARD));
  Serial.println("Version : " + String(SINRICPRO_VERSION_STR));
  
  pinMode(MOTIONSENSOR_PIN, INPUT);
  pinMode(D1, OUTPUT);
  setupRelays();
  setupFlipSwitches();
  setupWiFi();
  setupSinricPro();
}

void loop()
{
  pinMode(MOTIONSENSOR_PIN, INPUT);
  SinricPro.handle();
  handleFlipSwitches();
}
