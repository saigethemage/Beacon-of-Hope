SYSTEM_MODE(SEMI_AUTOMATIC);

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

const int LED = 11;

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  bme.begin();
}

void loop() {
  flashSOS();
  printBME();
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