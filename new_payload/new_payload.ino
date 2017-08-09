#include <Wire.h>
#include <Arduino.h>
#include "TinyGPS.h"
#include <SoftwareSerial.h>
#include <stdint.h>

#include "GY_85.h"
#include "SparkFunBME280.h"
#include "CircularBuffer.h"
#include "Wire.h"
#include "SPI.h"

//Global sensor object
BME280 mySensor;
uint8_t buttonGroundPin = A0;
uint8_t buttonSensePin = A2;

//This is a fixed-size buffer used for averaging data, a software LP filter
//Depending on the settings, the response can be quite slow.
CircularBuffer myBuffer(50);

float reference = 0;

GY_85 GY85;     
TinyGPS gps;
SoftwareSerial ss(4,3);


void setup()
{
    setIMU();
    setGPS();
    setBME();
   
}

  
 /* Serial.println();
  Serial.print("Temperature: ");
  Serial.print(mySensor.readTempC(), 2);
  Serial.println(" degrees C");
  Serial.print("Last sample: ");
  if(lastAlt >= 0)
  {
    Serial.print(" ");
  }
  Serial.print(lastAlt, 2);
  Serial.println(" ft");
  Serial.print (altmeter);
  Serial.println ("meter"); 
  Serial.print("Last 15 samples averaged: ");
  if(tempAlt >= 0)
  {
    Serial.print(" ");
  }
  Serial.print(tempAlt);
  Serial.println(" ft");
  Serial.print (altaverage);
  Serial.println ("meter");  
  delay(500);

}*/

void setIMU() {
    Wire.begin();           //sda ==> A4 scl a5
    delay(10);
    Serial.begin(9600);
    delay(10);
    GY85.init();
    delay(10);
}

void setBME(){
  mySensor.settings.commInterface = I2C_MODE;
  mySensor.settings.I2CAddress = 0x77;
  
   pinMode( buttonSensePin, INPUT_PULLUP );
   pinMode( buttonGroundPin, OUTPUT );
   digitalWrite( buttonGroundPin, 0 );
   mySensor.settings.runMode = 3;
   mySensor.settings.tStandby = 0;
   mySensor.settings.filter = 4;
   mySensor.settings.tempOverSample = 5;
   mySensor.settings.pressOverSample = 5;
   Serial.begin(9600);
}

void setGPS() {
    ss.begin(9600);       
    delay(10);
}

void getIMUData() {
    int ax = GY85.accelerometer_x( GY85.readFromAccelerometer() );
    int ay = GY85.accelerometer_y( GY85.readFromAccelerometer() );
    int az = GY85.accelerometer_z( GY85.readFromAccelerometer() );

    int cx = GY85.compass_x( GY85.readFromCompass() );
    int cy = GY85.compass_y( GY85.readFromCompass() );
    int cz = GY85.compass_z( GY85.readFromCompass() );

    float gx = GY85.gyro_x( GY85.readGyro() );
    float gy = GY85.gyro_y( GY85.readGyro() );
    float gz = GY85.gyro_z( GY85.readGyro() );
    float gt = GY85.temp  ( GY85.readGyro() );
    
   //ax
   Serial.print(ax);
   Serial.print(",");
   //ay
   Serial.print(ay);
   Serial.print(",");
   //az
   Serial.print(az);
   Serial.print(",");
   //gx
   Serial.print(gx);
   Serial.print(",");
   //gy
   Serial.print(gy);
   Serial.print(",");
   //gz
   Serial.print(gz);
   Serial.print(",");
  
   
}

void loop()
{
    getIMUData();
    bool newData = false;
  
  //for (unsigned long start = millis(); millis() - start < 10;)
  {
    while (ss.available())
    {
      char c = ss.read();
      
      if (gps.encode(c)) 
        newData = true;
       //Get the local temperature!  Do this for calibration
        mySensor.readTempC();
  
  //Check button.  No debounce, so hold for operation
        if( digitalRead( buttonSensePin ) == 0 )
          {
            //Set reference altitude.
            reference = mySensor.readFloatAltitudeMeters();
          }
  
        /* float lastAlt = mySensor.readFloatAltitudeFeet() - reference;
         myBuffer.pushElement(lastAlt);
         float tempAlt = myBuffer.averageLast(15); //Do an average of the latest samples
         float altmeter = lastAlt * 0.3048 ;
         float altaverage = tempAlt * 0.3048 ;*/
    }
  }

  if (newData)
  {
         float lastAlt = mySensor.readFloatAltitudeMeters() - reference;
         myBuffer.pushElement(lastAlt);
         float tempAlt = myBuffer.averageLast(15); //Do an average of the latest samples
    float flat, flon, alt, velocity, satelit ;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    //alt = gps.f_altitude();
    velocity = gps.f_speed_kmph();
    satelit = gps.satellites();
    
    
    flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
    
    flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
    /*Serial.print(flat);
    Serial.print(",");
    Serial.print(flon);
    Serial.print(","); */
    Serial.print(tempAlt);
    Serial.println(",");
    /*Serial.print(velocity);
    Serial.print(",");
    Serial.println(satelit);*/
    
     
     
     
  }
    else {
         float lastAlt = mySensor.readFloatAltitudeMeters() - reference;
         myBuffer.pushElement(lastAlt);
         float tempAlt = myBuffer.averageLast(15); //Do an average of the latest samples
         //float altmeter = lastAlt * 0.3048 ;
         //float altaverage = tempAlt * 0.3048 ;
         float flat, flon, alt,velocity,satelit;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    alt = gps.f_altitude();
    velocity = gps.f_speed_kmph();
    satelit = gps.satellites();
    
    flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
    
    flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
      /*Serial.print(flat);
      Serial.print(",");
      Serial.print(flon);
      Serial.print(",");*/
      Serial.print(tempAlt);
      Serial.println(",");
      /*Serial.print(velocity);
      Serial.print(",");
      Serial.println(satelit);*/
     
    

    


    
    }
    delay(100);             // only read every 0,5 seconds, 10ms for 100Hz, 20ms for 50Hz
}



