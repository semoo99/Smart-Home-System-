int pirled = 12;                // LED 
int pirPin = 4;                 // PIR Out pin 
int pirStat = 0;                   // PIR status
float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;
int isFlamePin = 5;  // This is our input pin
int isFlame = HIGH;  // HIGH MEANS NO FLAME

int Buzzer= 6;

int ldrState = 0;     
#include "DHT.h"

#define DHTPIN 7     // Digital pin connected to the DHT sensor

#define DHTTYPE DHT11   // DHT 

DHT dht(DHTPIN, DHTTYPE);


int ldr = 22;
#define rainfall A0


int value;
int nodepir =23;
int set=10;

int smokeA0 = A1;
int nodeGas =24;
int nodeflame =25;
// Your threshold value
int sensorThres = 150;

//sketch created by Akshay Joseph
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

#include <Servo.h>

Servo servo1;  // create servo object to control a servo
Servo servo2;  // create servo object to control a servo
Servo servo3;  // create servo object to control a servo

int val;    // variable to read the value from the analog pin

#include <Adafruit_Fingerprint.h>

#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
SoftwareSerial mySerial(2, 3);
#else
#define mySerial Serial1
#endif

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

const int ledPin =  13;      // the number of the LED pin


void setup() {
 pinMode(ldr, INPUT);
 pinMode(Buzzer, OUTPUT);
 pinMode(pirled, OUTPUT);     
 pinMode(pirPin, INPUT);  
 pinMode(nodepir, OUTPUT); 
 pinMode(nodeGas, OUTPUT);    
 pinMode(nodeflame, OUTPUT); 
 Serial.begin(9600);
 pinMode(isFlamePin, INPUT);


  
 pinMode(rainfall,INPUT);
 pinMode(Buzzer, OUTPUT);
 pinMode(smokeA0, INPUT);
 
 dht.begin();
 lcd.init();    
 lcd.backlight();
 lcd.clear();
 lcd.setCursor(5,0);
 lcd.print("Smart Home");

 servo1.attach(8);  // attaches the servo on pin 9 to the servo object
 servo2.attach(9);  // attaches the servo on pin 9 to the servo object
 servo3.attach(10);  // attaches the servo on pin 9 to the servo object



 pinMode(ledPin, OUTPUT);
 
  while (!Serial);  
  delay(100);
  finger.begin(57600);
  delay(5);
//  if (finger.verifyPassword()) {
//    Serial.println("Found fingerprint sensor!");
//  } 
//  else {
//    Serial.println("Did not find fingerprint sensor :(");
//    while (1) { delay(1); }
//  }
  
}
void loop(){

  
  
  
  Amp();
 // PIR();
  vldr();
  flame();
  //temp();
  rain();
  gas();
 // fingerp();
  
} 


void PIR(){
  
 pirStat = digitalRead(pirPin);
 if (pirStat == HIGH) {            // if motion detected
  digitalWrite(pirled, HIGH);  // turn LED ON
  digitalWrite(nodepir, HIGH);  // turn LED ON
  Serial.println("Hey I got you!!!");
  lcd.setCursor(5,0);
  lcd.print("Smart Home");
  lcd.setCursor(4,2);
  lcd.print("Hey I got you!!!");
  delay(2000);
  lcd.clear();
 } 
 else {
    Serial.println("no move");  
    digitalWrite(nodepir, LOW);  // turn LED ON
   digitalWrite(pirled, LOW); // turn LED OFF if we have no motion
 }
}


void flame() {
  isFlame = digitalRead(isFlamePin);
  if (isFlame== LOW)
  {
  servo2.write(80);
  servo3.write(80);
  Serial.println("FLAME, FLAME, FLAME");
  digitalWrite(Buzzer, HIGH);
  lcd.setCursor(5,0);
  lcd.print("Smart Home");
  lcd.setCursor(7,2);
  lcd.print("Fire !!!!!!");
  digitalWrite(nodeflame, HIGH);  // turn LED ON
  delay(500);
  lcd.clear();

  }
  else
  {
    Serial.println("no flame");
     digitalWrite(nodeflame, LOW);  // turn LED ON
    digitalWrite(Buzzer, LOW);
  }
  }




void temp() {

 

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F(" Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("C "));
  Serial.print(f);
  Serial.print(F("F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("C "));
  Serial.print(hif);
  Serial.println(F("F"));
  lcd.setCursor(5,0);
  lcd.print("Smart Home");
  lcd.setCursor(2,3);
  lcd.print("T= ");
  lcd.setCursor(5,3);
  lcd.print(t);
  lcd.setCursor(9,3);
  lcd.print("C ");
  
  lcd.setCursor(11,3);
  lcd.print("H= ");
  lcd.setCursor(13,3);
  lcd.print(hic);
  lcd.setCursor(16,3);
  lcd.print("%");
   

  delay(500);

  
}






void rain() {

  

 value = analogRead(rainfall);

 Serial.println("LOL");


// Serial.println(value);

 value = map(value,0,1023,225,0);

 Serial.println(value);

 if(value>=100){
  servo2.write(0);
  servo3.write(0);
  Serial.println("rain detected");
  lcd.setCursor(5,0);
  lcd.print("Smart Home");
  lcd.setCursor(4,2);
  lcd.print("it's Rrainning");
  digitalWrite(Buzzer,HIGH);
  servo2.write(0);
  servo3.write(0);
  delay(500);
  

 }

 else{

  digitalWrite(Buzzer,LOW);
  servo2.write(40);
  servo3.write(40);

 }



}



void vldr() {
ldrState = digitalRead(ldr);

  if (ldrState == HIGH) {
    Serial.println("Good Night");
    digitalWrite(ledPin, HIGH);
  } else {
    Serial.println("Good day");
    digitalWrite(ledPin, LOW);
  }
}

void gas() {
  int analogSensor = analogRead(smokeA0);

  Serial.print("Pin Gas: ");
  Serial.println(analogSensor);
  // Checks if it has reached the threshold value
  if (analogSensor > sensorThres)
  {
    servo2.write(80);
    servo3.write(80);
    lcd.clear();
    lcd.setCursor(5,0);
    lcd.print("Smart Home");
    lcd.setCursor(6,1);
    lcd.print("Gas leek");
    digitalWrite(nodeGas, HIGH);
    tone(Buzzer, 1000, 200);
    delay(1000);
    lcd.clear();
  }
  else
  {
    digitalWrite(nodeGas, LOW);
    noTone(Buzzer);
  }

}

void fingerp(){
  uint8_t p = finger.getImage();
if (p == FINGERPRINT_NOFINGER)  goto NoFinger; 
else if (p == FINGERPRINT_OK) {
  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  goto NoMatch;
  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  goto NoMatch;
  Serial.print("Welcome! Your ID is ");
  Serial.print(finger.fingerID);
  Serial.println(". You are granted access.");
  
  digitalWrite(Buzzer, LOW);
  servo1.write(90);
  delay(1500);
  servo1.write(0);
  return; 
}
NoMatch: 
{
Serial.println("Access  Denied"); 

digitalWrite(Buzzer, HIGH);
delay(1500); 
digitalWrite(Buzzer, LOW);
return;
} 
NoFinger: 
{
Serial.println("No finger detected");
delay(1500);
} 
}

void Amp() {
unsigned int x=0;
float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;

  for (int x = 0; x < 150; x++){ //Get 150 samples
  AcsValue = analogRead(A2);     //Read current sensor values   
  Samples = Samples + AcsValue;  //Add samples together
  delay (3); // let ADC settle before next sample 3ms
}
AvgAcs=Samples/150.0;//Taking Average of Samples

//((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
//2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
//out to be 2.5 which is out offset. If your arduino is working on different voltage than 
//you must change the offset according to the input voltage)
//0.066v(66mV) is rise in output voltage when 1A current flows at input
AcsValueF = (2.5 - (AvgAcs * (12.0 / 1024.0)) )/0.066;
Serial.print("current");
Serial.println(AcsValueF);//Print the read current on Serial monitor
float wat = AcsValueF  ;
  lcd.setCursor(5,0);
  lcd.print("Smart Home");
  lcd.setCursor(2,1);
  lcd.print("All is goood >_<");
  lcd.setCursor(2,3);
  lcd.print("power used = ");
  lcd.setCursor(13,3);
  lcd.print(AcsValueF *-0.4);
  lcd.setCursor(17,3);
  lcd.print("W/h");
delay(50);
}
