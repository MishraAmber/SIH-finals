#include <Adafruit_Fingerprint.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include<DHT.h>
#include <MHZ.h> // Co2 Sensor
//#include <WiFi.h>
//#include <NTPClient.h>
//#include <WiFiUdp.h>
//====================TFT Checks code=================
#include <TFT_eSPI.h>
#include <SPI.h>
#include "times.h"
#include "alcoholtestpic.h"
#include "thumb.h"
#include "temperature.h"
#include "heart_rate.h"
#include "check.h"
//=====================TFT Check code end======================

//======================Ubisots ==========================================
//#include "UbidotsEsp32Mqtt.h"
////Ubidots details
//const char*token = "BBFF-Z36fdiGtPiw9j0kBJK0gAbCMe0M23b";
//const char*variable_label = "temperature";
//const char*variable_label2 = "humidity";
//const char*variable_label3 = "alcohol";
//const char*variable_label4 = "body-temperature";
//const char*device_label = "vechile-monitoring";
////Wifi details 
//const char*ssid = "Real Me Narzo 30 Pro";
//const char*pass = "12345678";
//Ubidots ubidots(token);
//==================================Ubidots ends================

//=======================Time Code Starts==================
////wifi credentials
//const char *ssid     = "Redmi Note 11";
//const char *password = "12345678";
//
////dht variables
////float h;
////float t;
//
////flag for dht delay
//int timeFlag=0;
//
////date and time library
//unsigned long epochTime;
//int monthDay;
//int currentMonth;
//int currentYear;
//
//WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP, "pool.ntp.org");
//===================Time code Ends===========================

//================== AFTER CHECKS CODE ===============
#define DHTTYPE DHT22
#define DHTPIN 4
DHT dht(DHTPIN,DHTTYPE);

#define A 12
#define B 14
int outputA = 0;
int outputB = 0;

int isBlacked = 0;


//#define co2pwmPin 13 // GPIO13
//MHZ co2(co2pwmPin, MHZ19C);


//================ AFTER CHECKS END ===============

//================TFT code starts=========================================

TFT_eSPI tft=TFT_eSPI();

//===================TFT code ends========================================

//=====================Relay=====================================
#define relayPin 15
//=====================Relay=====================================

//===========================HEART CODE START ========================================


MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100]; 
uint16_t redBuffer[100];  
#else
uint32_t irBuffer[100]; 
uint32_t redBuffer[100];  
#endif

int32_t bufferLength; 
int32_t spo2; 
int8_t validSPO2; 
int32_t heartRate; 
int8_t validHeartRate;

byte pulseLED = A5; 
byte readLED = A4; 

//==================== HEART CODE END==================================

//=============================TEMP CODE START ==============================

const int oneWireBus = 2;     

OneWire oneWire(oneWireBus);

DallasTemperature sensors(&oneWire);

//============================TEMP CODE END================================

//============================ALCOHOL CODE STARS===========================
#define Sober 300

#define MQ3 27

float sensorValue;

//==========================ALCOHOL CODE END================================


int isUserFound = 0;
int heartRateChecked = 0;
int temperatureChecked = 0;
int alcoholChecked = 0;

//============================Fingerprint code starts====================
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&Serial2);
//============================Fingerprint code end=======================


//============================Global Variables ===========================
int t;
int h;
int ppm_pwm;
String airQuality; 

void setup()
{
  Serial.begin(115200);
//  //Ubidots credentials
//  ubidots.connectToWifi(ssid,pass);
//  ubidots.setup();
//  ubidots.reconnect();

  sensors.begin();   // for bodytemperature
  dht.begin(); // for DST
  pinMode(A,INPUT);// MP503
  pinMode(B,INPUT);
//===========================Relay==============================
  pinMode(relayPin,OUTPUT);
//==========================Time Code Starts===========================
//WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//
//timeClient.begin();
//timeClient.setTimeOffset(19800);

//============================Time Code Ends==============================
  
//============================TFT Setup===========================
tft.init(); //initialise tft display
  
tft.setRotation(1); //landscape mode
//============================TFT Setup end=================================

  // co2 sensor
// pinMode(co2pwmPin, INPUT);
//    delay(100);
//    if (co2.isPreHeating()) {
//        Serial.println("Preheating");
//        while (co2.isPreHeating()) {
//            Serial.print(".");
//            delay(5000);
//        }
//    }

    
  
  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  delay(100);
  Serial.println("\n\nAdafruit finger detect test");
  

  // set the data rate for the sensor serial port
  finger.begin(57600);
  
  delay(5);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    while (1) { delay(1); }
  }

  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
  }
  else {
    Serial.println("Waiting for valid finger...");
      Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  }



////  ========================= HEART CODE START ==============================
//
//  pinMode(pulseLED, OUTPUT);
//  pinMode(readLED, OUTPUT);
//
//  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
//  {
//    Serial.println(F("MAX30102 was not found. Please check wiring/power."));
//    while (1);
//  }
//
//  Serial.println(F("Attach sensor to finger with rubber band."));
//  Serial.read();
//
//  byte ledBrightness = 60; 
//  byte sampleAverage = 4; 
//  byte ledMode = 3; 
//  byte sampleRate = 200; 
//  int pulseWidth = 411; 
//  int adcRange = 4096; 
//
//  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
////===================HEART CODE END ===============================

//====================TEMP CODE START=======================

//======================TEMP CODE END====================
//=====================ALCOHOL CODE STARTS=================
  Serial.println("MQ3 Heating Up!");
//=======================ALCOHOL CODE END=================


}

void loop()                     
{
//  if (!ubidots.connected())
//  {
//    ubidots.reconnect();
//  }

  if(isUserFound == 0){
    //Fingerprint Sign
    showThumbPage();
    getFingerprintID();
  }
  if(isUserFound == 1 && temperatureChecked == 0){
    
   showTemperaturePage();
   delay(5000);
   getTemperature();
  }
//  if(heartRateChecked == 1 &&  temperatureChecked == 0){
//    //Temperature sign
//    
//    Serial.println("Heart Rate Checked Successfully!");
//    getTemperature();
//  }
//  if(temperatureChecked == 1 && alcoholChecked == 0 ){
//    //Alcohol Sign
// 
//   
//    getAlcohol();
//  }
  if(temperatureChecked == 1 && alcoholChecked == 0){
    showAlcoholPage();
    delay(5000);
    getAlcohol();
    showCheckPage();
    delay(2000);
//    digitalWrite(relayPin,LOW);
//    delay(1);
  }
  if(alcoholChecked == 1){
    
//    delay(1);
     pinMode(relayPin,LOW);
    showInfo();
    DHT();
  }
}

void showCheckPage(){
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  tft.pushImage(40,0,240,240,check);
}

void showThumbPage(){
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.drawNumber(1,150,10,7);
  tft.setFreeFont(&times18pt7b);
  tft.drawString("Place your thumb for",5,65);
  tft.drawString("Fingerprint",5,100);
  tft.setSwapBytes(true);
  tft.pushImage(120,134,92,90,thumb);
}
//
//void showHeartPage(){
//  tft.fillScreen(TFT_BLACK);
//  tft.setTextColor(TFT_WHITE,TFT_BLACK);
//  tft.drawNumber(2,150,10,7);
//  tft.setFreeFont(&times18pt7b);
//  tft.drawString("Place your finger to",5,65);
//  tft.drawString("check Heart Rate",5,100);
//  tft.setSwapBytes(true);
//  tft.pushImage(0,134,320,90,heart_rate);
//}
//
void showTemperaturePage(){
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.drawNumber(3,150,10,7);
  tft.setFreeFont(&times18pt7b);
  tft.drawString("Grab stick to check",5,65);
  tft.drawString("Body Temperature",5,100);
  tft.setSwapBytes(true);
  tft.pushImage(121,134,90,90,tempcopy2);
}
//
void showAlcoholPage(){
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.drawNumber(4,150,10,7);
  tft.setFreeFont(&times18pt7b);
  tft.drawString("Blow air into Alcohol",5,65);
  tft.drawString("Sensor",5,100);
  tft.setSwapBytes(true);
  tft.pushImage(130,134,58,90,alcoholtestpic);
}

void showInfo()
{
 tft.setSwapBytes(true);
 tft.fillScreen(TFT_BLACK);

 tft.setTextColor(TFT_WHITE);

 // headings
 tft.drawRoundRect(0, 0, 160, 20, 3, TFT_WHITE);
 tft.drawString("TEMPERATURE(C)", 25, 1, 2);
 tft.drawRoundRect(0, 20,160, 100, 3, TFT_WHITE);
 tft.setTextColor(TFT_WHITE,TFT_BLACK);
 tft.drawString(String(t),30,45,7);

 tft.drawRoundRect(0, 120, 160, 20, 3, TFT_WHITE);
 tft.drawString("HUMIDITY(%)", 50, 121, 2);
 tft.drawRoundRect(0, 140, 160, 100, 3, TFT_WHITE);
 tft.setTextColor(TFT_WHITE,TFT_BLACK);
 tft.drawString(String(h),30,170,7);

 tft.drawRoundRect(160, 0, 160, 20, 3, TFT_WHITE);
 tft.drawString("AIR QUALITY", 190, 1, 2);
 tft.drawRoundRect(160, 20, 160, 100, 3, TFT_WHITE);
 tft.setTextColor(TFT_WHITE,TFT_BLACK);
 tft.drawString(airQuality,180,30,4);
 tft.drawString("Pollution",180,57,4);


 tft.drawRoundRect(160, 120, 160, 20, 3, TFT_WHITE);
 tft.drawString("CARBON DIOXIDE(ppm)",175,121,2);
 tft.drawRoundRect(160, 140, 160, 100, 3, TFT_WHITE);
 tft.setTextColor(TFT_WHITE,TFT_BLACK);
 tft.drawString("NIL",200,170,4);
}
void DHT(){
  t = dht.readTemperature();
  h = dht.readHumidity();

  Serial.println(t);
  Serial.println(h);
//  ubidots.add(variable_label,t);
//  ubidots.add(variable_label2,h);
//  ubidots.publish(device_label);

outputA = analogRead(A);
  outputB = analogRead(B);

  int voltageA = outputA * (3.3 / 4095);
  int voltageB = outputB * (3.3 / 4095);

 // digitalWrite(relayPin,HIGH);
  Serial.print("value of A = ");
  Serial.println(voltageA);
  
  Serial.print("value of B = ");
  Serial.println(voltageB);

  if (voltageA == 0 && voltageB == 0)
    {
      Serial.print(" CLEAN AIR ");
      airQuality = "No"; 
    }  
else if (voltageA == 0 && voltageB == 3)
    {
      Serial.print(" SLIGHT POLLUTION ");
      airQuality = "Moderate";
    }
else if (voltageA == 3 && voltageB == 0)
    {
      Serial.print(" MIDDLE AIR POLLUTION ");
      airQuality = "Moderate";
    }
else if (voltageA == 3 && voltageB == 3)
    {
      Serial.print(" HEAVY POLLUTION ");
      airQuality = "Heavy";
    } 

  // co2 sensor code
//    ppm_pwm = co2.readCO2PWM();
//    Serial.println("PPM = " + String(ppm_pwm));

  // Time code ===========================================================
//  timeClient.update();
//  epochTime = timeClient.getEpochTime();
//  //Get a time structure
//  struct tm *ptm = gmtime ((time_t *)&epochTime);
//  monthDay = ptm->tm_mday;
//  currentMonth = ptm->tm_mon+1;
//  currentYear = ptm->tm_year+1900;
//  // formattedDate = timeClient.getDay();
//
//  // // Extract date
//  // int splitT = formattedDate.indexOf("T");
//  // dayStamp = formattedDate.substring(0, splitT);
//  
//  tft.setTextColor(TFT_WHITE,TFT_BLACK);
//  tft.setFreeFont(&times18pt7b);
//  tft.drawString(String(timeClient.getHours())+":"+String(timeClient.getMinutes()),170,45);
//  tft.drawString(String(monthDay)+"/"+String(currentMonth)+"/"+String(currentYear),170,100,2); 
//    digitalWrite(relayPin,HIGH);   
    delay(2000);
}

void getAlcohol(){
  sensorValue = analogRead(MQ3)*500/4095; 

  Serial.println("Sensor Value: ");

  Serial.println(sensorValue);
  if(sensorValue < 300){
    alcoholChecked = 1;
 
    Serial.println("Alcohol test passed successfully!");
//    ubidots.add(variable_label3,sensorValue);
//    ubidots.publish(device_label);
  }

  delay(3000);
}

void getTemperature(){
  Serial.println("i am called");
   sensors.requestTemperatures(); 
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.print(temperatureF);
  Serial.println("ºF");
  if(temperatureF > 90 && temperatureF <99){
    temperatureChecked = 1;

    Serial.print("Temperature Checked Successfully!");
//    ubidots.add(variable_label4,temperatureF);
//    ubidots.publish(device_label);
  }
  delay(1000);
}

void getHeartData(){
   bufferLength = 100; 

  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); 

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  while (1)
  {
    if(heartRateChecked == 1){
      return;
    }
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    for (byte i = 75; i < 100; i++)
    {
      
      while (particleSensor.available() == false) 
        particleSensor.check(); 

      digitalWrite(readLED, !digitalRead(readLED)); 

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); 


      Serial.print(F(", HR="));
      Serial.println(heartRate, DEC); // 60 - 100

 
     

      Serial.print(F(", SPO2="));
      Serial.println(spo2, DEC); // x>94

      if(heartRate > 40 && heartRate <150 && spo2 >50){
        heartRateChecked = 1;
       
        Serial.println("Heart Rate Checked Complete");
      }

      

    
     
    }

    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}

uint8_t getFingerprintID() {

  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
    isUserFound = 1;

    Serial.println("called =====================================");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  return finger.fingerID;
}

int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;

  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID;
}
