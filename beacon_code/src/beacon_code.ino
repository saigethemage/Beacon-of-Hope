SYSTEM_MODE(SEMI_AUTOMATIC);

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

const int MPU_ADDRESS = 0x68;
float sensitivity, acceleration;

const int LED = 11;

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  bme.begin();
  beginMPU();
  configureACC(3);
}

void loop() {
  acceleration = getACC();
  if(acceleration > 27) {
    Serial.printlnf("Acceleration: %0.2f", acceleration);
  }
}

void flash(int duration) {
  digitalWrite(LED, HIGH);
  delay(duration);
  digitalWrite(LED, LOW);
  delay(duration);
}

void flashSOS() {
  for(int i=1; i<5; i+=2) {
    for(int j=0; j<3; j++) {
      flash(200*i);
    }
    delay(400);
  }
}

void printBME() {
    Serial.printf("Altitude: %0.2f m\t", bme.readAltitude(1013.25) * 3.28);
    Serial.printf("Pressure: %0.2f hPa\t", bme.readPressure() / 100.0F);
    Serial.printf("Temperature: %0.2f *F\t", bme.readTemperature() * 1.8 + 32.0);
    Serial.printf("Humidity: %0.2f%%\n\n", bme.readHumidity());
}

void beginMPU() {
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
   *                      AFS_SEL | Full Scale Range | LSB Sensitivity 
   *                         0    |       +- 2 g     |   16384 LSB/g
   *                         1    |       +- 4 g     |   8192 LSB/g
   *                         2    |       +- 8 g     |   4096 LSB/g
   *                         3    |       +- 16 g    |   2048 LSB/g
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

float getACC() {
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
  
  return accel;
}