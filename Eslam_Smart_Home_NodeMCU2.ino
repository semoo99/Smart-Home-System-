
#if !(defined(ESP8266) || defined(ESP32))
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
#include "SinricProMotionsensor.h"

#include "SinricProTemperaturesensor.h"
#include "SinricProDoorbell.h"

#include "DHT.h"                              // https://github.com/adafruit/DHT-sensor-library

#define WIFI_SSID         "Smart-X"
#define WIFI_PASS         "11a22b33c"
#define APP_KEY           "896e4ffd-9a2d-4249-a077-4ab3fc0df3a0"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        "d16f1ed4-3a6b-4ea7-a95d-7e8573025ccb-9c8d6c95-6592-481b-a7f0-438e79b9e708"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"

#define MOTIONSENSOR_ID   "6321b7cf36b44d06d4bb460d"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define DOORBELL_ID       "632863d8fa69c39e7cce3df5"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define TEMP_SENSOR_ID    "6321b9ee36b44d06d4bb4914"    // Should look like "5dc1564130xxxxxxxxxxxxxx"
#define BAUD_RATE         115200              // Change baudrate to your need (used for serial monitor)

#define EVENT_WAIT_TIME   60000               // send event every 60 seconds

#define fan_PIN   D6
// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic, connect pin 1 to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHT_PIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
#if (ESP8266)
  #define DHT_PIN    D5
#elif (ESP32)
  #define DHT_PIN    D5
#endif

#define BUTTON_PIN  D2

#define MOTIONSENSOR_PIN  D3                   // PIN where motionsensor is connected to
                                              // LOW  = motion is not detected
                                              // HIGH = motion is detected


// Uncomment whatever type you're using!
#define DHT_TYPE      DHT11   // DHT 11
//#define DHT_TYPE        DHT22   // DHT 22  (AM2302), AM2321
//#define DHT_TYPE      DHT21   // DHT 21 (AM2301)

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHT_PIN, DHT_TYPE);

bool deviceIsOn;                              // Temeprature sensor on/off state
float temperature;                            // actual temperature
float humidity;                               // actual humidity
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)
unsigned long lastEvent = (-EVENT_WAIT_TIME); // last time event has been sent



bool myPowerState = true;                     // assume device is turned on
bool lastMotionState = false;
unsigned long lastChange = 0;


/* bool onPowerState(String deviceId, bool &state)

   Callback for setPowerState request
   parameters
    String deviceId (r)
      contains deviceId (useful if this callback used by multiple devices)
    bool &state (r/w)
      contains the requested state (true:on / false:off)
      must return the new state

   return
    true if request should be marked as handled correctly / false if not
*/
bool onPowerState(const String &deviceId, bool &state) 
{
  (void) deviceId;
  
  Serial.println("TemperatureSensor turned " + String(state ? "on" : "off"));
  deviceIsOn = state; // turn on / off temperature sensor
  return true; // request handled properly
}

/* handleTemperatatureSensor()
   - Checks if Temperaturesensor is turned on
   - Checks if time since last event > EVENT_WAIT_TIME to prevent sending too much events
   - Get actual temperature and humidity and check if these values are valid
   - Compares actual temperature and humidity to last known temperature and humidity
   - Send event to SinricPro Server if temperature or humidity changed
*/
void handleTemperaturesensor() 
{
  // device is off...do nothing
  if (deviceIsOn == false) 
    return; 

  unsigned long actualMillis = millis();
  
  if (actualMillis - lastEvent < EVENT_WAIT_TIME) 
    return; //only check every EVENT_WAIT_TIME milliseconds

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  temperature = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //  temperature = dht.readTemperature(true);
  humidity = dht.readHumidity();                          // get actual humidity

  if (isnan(temperature) || isnan(humidity)) 
  { 
    // reading failed...
    Serial.println("DHT reading failed");       // print error message
    return;                                     // try again next time
  }

  // Check if any reads failed and exit early (to try again).
  if (temperature == lastTemperature || humidity == lastHumidity) 
    return; 

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];     // get temperaturesensor device
  bool success = mySensor.sendTemperatureEvent(temperature, humidity);  // send event
  
  if (success) 
  {  
    // if event was sent successfuly, print temperature and humidity to serial
    Serial.println("Temperature: " + String(temperature, 1) + " Celsius\tHumidity: " + String(humidity, 1) + " %");
  } 
  else 
  {  
    // if sending event failed, print error message
    Serial.println("Something went wrong...Could not send Event to server!");
  }

  lastTemperature = temperature;  // save actual temperature for next compare
  lastHumidity = humidity;        // save actual humidity for next compare
  lastEvent = actualMillis;       // save actual time for next compare
}


void checkButtonPress() 
{
  static unsigned long lastBtnPress;
  unsigned long actualMillis = millis();

  if (actualMillis - lastBtnPress > 500) 
  {
    if (digitalRead(BUTTON_PIN) == LOW) 
    {
      Serial.println("Ding dong...");
      lastBtnPress = actualMillis;

      // get Doorbell device back
      SinricProDoorbell& myDoorbell = SinricPro[DOORBELL_ID];

      // send doorbell event
      myDoorbell.sendDoorbellEvent();
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

void handleMotionsensor() {
  if (!myPowerState) return;                            // if device switched off...do nothing

  unsigned long actualMillis = millis();
  if (actualMillis - lastChange < 250) return;          // debounce motionsensor state transitions (same as debouncing a pushbutton)

  bool actualMotionState = digitalRead(MOTIONSENSOR_PIN);   // read actual state of motion sensor

  if (actualMotionState != lastMotionState) {         // if state has changed
    Serial.printf("Motion %s\r\n", actualMotionState?"detected":"not detected");
    lastMotionState = actualMotionState;              // update last known state
    lastChange = actualMillis;                        // update debounce time
    SinricProMotionsensor &myMotionsensor = SinricPro[MOTIONSENSOR_ID]; // get motion sensor device
    myMotionsensor.sendMotionEvent(actualMotionState);
  }
}


// setup function for SinricPro
void setupSinricPro() 
{
  // add device to SinricPro
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  SinricPro.add<SinricProDoorbell>(DOORBELL_ID);
  mySensor.onPowerState(onPowerState);
  SinricProMotionsensor& myMotionsensor = SinricPro[MOTIONSENSOR_ID];

  // set callback function to device
  myMotionsensor.onPowerState(onPowerState);

  // setup SinricPro
  SinricPro.onConnected([]() 
  {
    Serial.println("Connected to SinricPro");
  });
  
  SinricPro.onDisconnected([]() 
  {
    Serial.println("Disconnected from SinricPro");
  });
  
  SinricPro.begin(APP_KEY, APP_SECRET);
}

// main setup function
void setup() 
{
  Serial.begin(BAUD_RATE); 
  while (!Serial);
  
  Serial.println("\nStarting temperaturesensor on " + String(ARDUINO_BOARD));
  Serial.println("Version : " + String(SINRICPRO_VERSION_STR));
  pinMode(MOTIONSENSOR_PIN, INPUT);
   pinMode(BUTTON_PIN, INPUT_PULLUP); // BUTTIN_PIN as INPUT
  dht.begin();

 Serial.println("\nStarting doorbell on " + String(ARDUINO_BOARD));
  Serial.println("Version : " + String(SINRICPRO_VERSION_STR));
  
  setupWiFi();
  setupSinricPro();

  pinMode(fan_PIN , OUTPUT);
}

void loop() 
{
  handleMotionsensor();
  checkButtonPress();
  SinricPro.handle();
  handleTemperaturesensor();
   if ( temperature >= 25){
    digitalWrite(fan_PIN , HIGH);
    
   }
   else {
    digitalWrite(fan_PIN , LOW);
   }
}
