/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/Saige/Documents/IoT/Beacon-of-Hope/beacon_code/src/beacon_code.ino"
/***********************************************************************
 * Overview:
 * 
 * The inclusion of drone technology in search and rescue 
 * operations has greatly decreased the time required to locate victims. 
 * However, the time required for search and rescue personnel to 
 * physically reach victims has gone unchanged. For search and rescue 
 * victims, time is of the essence. The more time a victim spends 
 * exposed to the elements or injury the less of a chance they have at 
 * surviving. This project aims to develop a device that can be dropped 
 * from an aerial vehicle to a victim’s location and provide the victim 
 * with essential communication and first aid while waiting for search 
 * and rescue personnel to arrive.
 * 
 * Technology:
 * 
 * This device interfaces with a MPU-6050 3-Axis Accelerometer Module, 
 * a light emitting diode (LED), a BME-280 Barometric Pressure, 
 * Temperature and Humidity Module, a GY_GPS6MV1 GPS Module, a ESP32_CAM
 * Camera Module, and a PARTICLE Argon Development Board. All data is
 * streamed to the ThingSpeak Internet of Things Platform.
 * 
 * Written by Saige Martinez for CNM Ingenuity's Deep Dive Coding
 * Internet of Things Bootcamp.
 **********************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++/TinyGPS++.h>
#include "ThingSpeak.h"

// MPU-6050 Module
void setup();
void loop();
void beginMPU();
void configureACC(int select);
bool readACC();
void flash(int duration);
void flashSOS();
#line 35 "c:/Users/Saige/Documents/IoT/Beacon-of-Hope/beacon_code/src/beacon_code.ino"
const int MPU_ADDRESS = 0x68;
float sensitivity, acceleration;

// LED
const int LED = 11;

// BME-280 Module
Adafruit_BME280 bme;

//GY_GPS6MV1 GPS Module
TinyGPSPlus gps;

// ThingSpeak Platform
TCPClient client;
unsigned long myChannelNumber = 1247589;
const char * myWriteAPIKey = "IEZR5IICK5PD48VB";

// Declare variables for handling system state, time, and data.
bool accelState, accelStateOld, systemState;
int currentTime, previousTime;
float altitude, pressure, temperature, humidity, latitude, longitude;

void setup() {
  Serial.begin(9600);
  //Serial1.begin(9600);
  beginMPU();
  configureACC(3);
  pinMode(LED, OUTPUT);
  bme.begin();
  ThingSpeak.begin(client);
}

void loop() {
  accelState = readACC();
  if(accelState != accelStateOld) {
    if(accelState) {
      systemState = !systemState;
    }
    accelStateOld = accelState;
  }

  if(systemState) {
    flashSOS();
    currentTime = millis();
    while(Serial1.available() > 0) {
      if(gps.encode(Serial1.read())) {
        if(gps.location.isValid()) {
          if((currentTime-previousTime) > 15000) {
            Serial.print(".");
            altitude = bme.readAltitude(1013.25) * 3.28;
            pressure = bme.readPressure() / 100.0F;
            temperature = bme.readTemperature() * 1.8 + 32.0;
            humidity = bme.readHumidity();
            latitude = gps.location.lat();
            longitude = gps.location.lng();
            ThingSpeak.setField(1, altitude);
            ThingSpeak.setField(2, pressure);
            ThingSpeak.setField(3, temperature);
            ThingSpeak.setField(4, humidity);
            ThingSpeak.setField(5, latitude);
            ThingSpeak.setField(6, longitude);
            ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
            previousTime = millis();
          }
        }
      }
    }  
  }
}

void beginMPU() {
  /*
   * This function configures the power mode and clock source
   * of the accelerometer with the following settings:
   *        DEVICE_RESET | False
   *        SLEEP        | False
   *        CYCLE        | False
   *        TEMP_DIS     | False
   *        CLKSEL[2:0]  | Internal 8MHz Oscillator
   * 
   * The clock source can be selected according to the following
   * table:
   *        CLKSEL | Clock Source
   *          0    | Internal 8MHz oscillator
   *          1    | PLL with X axis gyroscope reference
   *          2    | PLL with Y axis gyroscope reference
   *          3    | PLL with Z axis gyroscope reference
   *          4    | PLL with external 32.768kHz reference
   *          5    | PLL with external 19.2MHz reference
   *          6    | Reserved
   *          7    | Stops the clock and keeps the time
   *               | generator in reset
   */

  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void configureACC(int select) {
  /*
   * This function is used to trigger the accelerometer self test
   * and configure the full scale of the accelerometer according to
   * the following table:
   *        AFS_SEL | Full Scale Range | LSB Sensitivity 
   *           0    |       +- 2 g     |   16384 LSB/g
   *           1    |       +- 4 g     |   8192 LSB/g
   *           2    |       +- 8 g     |   4096 LSB/g
   *           3    |       +- 16 g    |   2048 LSB/g
   */

  switch (select) {
    case 1:
      select = 0x08;
      sensitivity = 8192.0;
      break;
    case 2:
      select = 0x10;
      sensitivity = 4096.0;
      break;
    case 3:
      select = 0x18;
      sensitivity = 2048.0;
      break;
    default:
      select = 0x00;
      sensitivity = 16384.0;
      break;
  }

  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1C);
  Wire.write(select);
  Wire.endTransmission(true);
}

bool readACC() {
  /*
   * This function requests the most recent 16-bit 2's compliment
   * accelerometer values for all axes and returns total acceleration.
   */

  int16_t xRaw, yRaw, zRaw;
  float xCal, yCal, zCal, accel;

  // Request most recent accelerometer measurements.
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B); // Start at 0x3B ACCEL_XOUT[15:8]
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true); // Read six registers.

  // Shift bits.
  xRaw = Wire.read() << 8 | Wire.read(); // 0x3B ACCEL_XOUT[15:8] & 0x3C ACCEL_XOUT[7:0]
  yRaw = Wire.read() << 8 | Wire.read(); // 0x3D ACCEL_YOUT[15:8] & 0x3E ACCEL_YOUT[7:0]
  zRaw = Wire.read() << 8 | Wire.read(); // 0x3F ACCEL_ZOUT[15:8] & 0x40 ACCEL_ZOUT[7:0]
  
  // Calibrate measurements to least significant bit sensitivity.
  xCal = xRaw / sensitivity * (-1);
  yCal = yRaw / sensitivity * (-1);
  zCal = zRaw / sensitivity * (-1);

  // Calculate total acceleration.
  accel = sqrt(pow(xCal, 2) + pow(yCal, 2) + pow(zCal, 2));
  
  if(accel > 2) {
    return true;
  }
  else {
    return false;
  }
}

void flash(int duration) {
  /*
   * This function switches on and off a light-emitting diode (LED).
   */

  digitalWrite(LED, HIGH);
  delay(duration);
  digitalWrite(LED, LOW);
  delay(duration);
}

void flashSOS() {
  /*
   * This function switches on and off a light-emitting diode (LED)
   * in an SOS pattern: 3 short flashes, 3 long flashes, and 3 short flashes.  
   */

  for(int i=1; i<5; i+=2) {
    for(int j=0; j<3; j++) {
      flash(200*i);
    }
    delay(400);
  }
}