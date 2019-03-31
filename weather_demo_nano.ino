/*
 * A portable weather station demo prototype.
 * Using Arduino Nano, DHT22, BMP180, LCD 20x4 display with YwRobot LCM1602,
 * light resistor
 *
 *
*/
// Adafruit BMP085 library https://github.com/adafruit/Adafruit-BMP085-Library
#include <Adafruit_BMP085.h>
// Adafruit DHT library https://github.com/adafruit/Adafruit_Sensor
#include <DHT.h>
// standard Arduino library
#include <Wire.h>
// LiquidCrystal_I2C library from https://github.com/marcoschwartz/LiquidCrystal_I2C
#include <LiquidCrystal_I2C.h>
// SDS011 library https://github.com/lewapek/sds-dust-sensors-arduino-library
//#include "SdsDustSensor.h"

#define DHTPIN 7 // pin DHT22 is connected to
#define DHTTYPE DHT22 // type of sensor is DHT 22  (AM2302)

#define AVGNUM 1 // number of measurements to average
#define DELAY 2000 // interval between measurements in ms

#define PHOTOPIN 0 // pin light resistor is connected to
#define BACKLIGHTPIN 3 // pin for backlight control

#define RXPIN 12 // RX pin connected to sensor's TX
#define TXPIN 11 // TX pin connected to sensor's RX

DHT dht (DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp;
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Set the LCD I2C address
//SdsDustSensor sds(RXPIN, TXPIN);


byte counter;
float hum, tempdht;
float tempdhtavg, humavg;
float tempbmp, pres;
float tempbmpavg, presavg;
int pm25, pm10;
word light;

void setup() {
  pinMode(BACKLIGHTPIN, OUTPUT);
  light = analogRead(PHOTOPIN);
  backlightControl(BACKLIGHTPIN, light);
  Serial.begin(9600);
  Wire.begin();
  lcd.init();
//  sds.begin();
  delay(200);
  lcd.backlight();
  lcd.setCursor(0, 0);
  dht.begin();
  tempdht = dht.readTemperature();
  hum = dht.readHumidity();
  if (isnan(tempdht) || isnan(hum)) {
    Serial.println("DHT failure!");
    lcd.print("DHT failure.");
    while(1);
  }
  if (!bmp.begin()) {
    Serial.println("BMP failure!");
    lcd.print("BMP failure.");
    while(1);
  }
  pres = bmp.readPressure();
  tempbmp = bmp.readTemperature();
//  sds.setActiveReportingMode(); // ensures sensor is in 'active' reporting mode
//  sds.setCustomWorkingPeriod(1); // sensor sends data every 1 minute
  Serial.println("Seconds;TempDHT *C;TempBMP *C;Humidity %;Pressure hPa;PM2.5;PM10;Light(0-1023)");
  
  lcd.clear();
  show(tempdht, hum, pres, tempbmp, light, 0, 0);
}            

void loop() { 

  light = analogRead(PHOTOPIN);
  backlightControl(BACKLIGHTPIN, light);
  delay(DELAY);
  counter++;
  tempdht = dht.readTemperature();
  hum = dht.readHumidity();

  pres = bmp.readPressure();
  tempbmp = bmp.readTemperature();

  tempdhtavg += tempdht;
  humavg += hum;
  tempbmpavg += tempbmp;
  presavg += pres;

  if (counter == AVGNUM) {
    tempdhtavg /= AVGNUM;
    humavg /= AVGNUM;
    tempbmpavg /= AVGNUM;
    presavg /= AVGNUM;

    show(tempdhtavg, humavg, presavg, tempbmpavg, light, pm25, pm10);
    
    tempdhtavg = 0;
    humavg = 0;
    tempbmpavg = 0;
    presavg = 0;
    counter = 0;
  }

}

void show(float tempdht, float hum, float pres, float tempbmp, word light, int pm25, int pm10) {
    pres /= 100;
    Serial.print(millis()/1000);
    Serial.print(";");
    Serial.print(tempdht);
    Serial.print(";");
    Serial.print(tempbmp);
    Serial.print(";");
    Serial.print(hum);
    Serial.print(";");
    Serial.print(pres);
    Serial.print(";");
    Serial.print(pm25);
    Serial.print(";");
    Serial.print(pm10);
    Serial.print(";");
    Serial.print(light);
    Serial.println("");
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(tempdht, 1);
    lcd.print("/");
    lcd.print(tempbmp, 1);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(round(hum), 1);
    lcd.print("% ");
    lcd.setCursor(14, 1);
    lcd.print("* ");
    lcd.print(light);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    lcd.print("Pressure: ");
    lcd.print(pres, 1);
    lcd.print(" hPa");
    lcd.setCursor(0, 3);
    lcd.print("PM 2.5:");
    lcd.print(pm25);
    lcd.print(" ");
    lcd.print("10:");
    lcd.print(pm10);
}

void backlightControl(byte pin, word light) {
  // values for 10 kOhm voltage divider resistor and 100 Ohm output resistor
  light = constrain(light, 40, 900);
  int level = map(light, 40, 900, 4, 255);
  analogWrite(pin, level);
}
