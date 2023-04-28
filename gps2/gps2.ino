#include "UbidotsEsp32Mqtt.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>



//GPS details
static const int RXPin = 33;
static const int TXPin = 32;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

SoftwareSerial GPS(RXPin,TXPin);

//Ubidots details
const char*token = "BBFF-Z36fdiGtPiw9j0kBJK0gAbCMe0M23b";
const char*variable_label = "position";
const char*variable_label2 = "speed";
//const char*variable_label3 = "distance";
const char*device_label = "vechile-monitoring";

//Wifi details 
const char*ssid = "Real Me Narzo 30 Pro";
const char*pass = "12345678";


//Setting ubidots as a client
Ubidots ubidots(token);

void setup()
{
  Serial.begin(115200);
  ubidots.connectToWifi(ssid,pass);
  ubidots.setup();
  ubidots.reconnect();
  GPS.begin(GPSBaud);
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
//
//  if(!accel.begin())
//
//  {
//
//    Serial.println("No valid sensor found");
//
//    while(1);
//
//  }
}

void loop()
{
  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  
  getPos();
 // delay(1000);
  getSpeed();
  //delay(1000);
 // getAcceleration();
//  delay(1000);
}

void getSpeed(){
  while(GPS.available() > 0){
    gps.encode(GPS.read());
    float s = gps.speed.kmph();
    Serial.println(s);
    Serial.println("I am called");
    ubidots.add(variable_label2,s);
    ubidots.publish(device_label);
  }
}

void getPos()
{
  float value = random(0,9)*10;
 
  while(GPS.available() > 0)
  {
    gps.encode(GPS.read());
    if (gps.location.isUpdated())
    {
      float latitude = gps.location.lat();
      float longitude = gps.location.lng();
      
      Serial.println(latitude);
      Serial.println(longitude);
      
      char*str_lat = (char*)malloc(sizeof(char)*10);
      char*str_lng = (char*)malloc(sizeof(char)*10);
      

      sprintf(str_lat,"%f",latitude);
      sprintf(str_lng,"%f",longitude);
      

      char*context = (char*)malloc(sizeof(char)*30);

      ubidots.addContext("lat",str_lat);
      ubidots.addContext("lng",str_lng);
     
      ubidots.getContext(context);
      

      ubidots.add(variable_label,value,context);
      ubidots.publish(device_label);

     
      Serial.println(str_lat);
      Serial.println(str_lng);
      
      

      free(str_lat);
      free(str_lng);
      delay(2000);
    } 
  }
}
