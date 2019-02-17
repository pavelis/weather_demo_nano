/*
 * A portable weather station demo prototype.
 * Using Arduino Nano, DHT22, BMP180, LCD 20x4 display with YwRobot LCM1602.
 *
 *
 *
*/
// Adafruit BMP085 library https://github.com/adafruit/Adafruit-BMP085-Library
#include <Adafruit_BMP085.h>
// Adafruit DHT library https://github.com/adafruit/Adafruit_Sensor
#include <DHT.h>
// standard Arduino Wire.h library
#include <Wire.h>
// LiquidCrystal_I2C library from https://github.com/marcoschwartz/LiquidCrystal_I2C
#include <LiquidCrystal_I2C.h>

#define DHTPIN 7 // pin DHT22 is connected to
#define DHTTYPE DHT22 // type of sensor is DHT 22  (AM2302)

#define AVGNUM 1 // number of measurements to average
#define DELAY 2000 // interval between measurements in ms

DHT dht (DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp;
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Set the LCD I2C address

byte counter;
float hum, tempdht;
float tempdhtavg, humavg;
float tempbmp, pres;
float tempbmpavg, presavg;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("... ");
  delay(1000);
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
    lcd.clear();
  show(tempdht, hum, pres, tempbmp);
}            

void loop() { 

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

    show(tempdhtavg, humavg, presavg, tempbmpavg);
    
    tempdhtavg = 0;
    humavg = 0;
    tempbmpavg = 0;
    presavg = 0;
    counter = 0;
  }

}

void show(float tempdht, float hum, float pres, float tempbmp) {
    pres /= 100;
    Serial.print("TempDHT: ");
    Serial.print(tempdht);
    Serial.print((char)223);
    Serial.print("C; ");
    Serial.print("TempBMP: ");
    Serial.print(tempbmp);
    Serial.print("; Hum: ");
    Serial.print(hum);
    Serial.print("%. ");
    Serial.print("Pressure: ");
    Serial.print(pres);
    Serial.println(" hPa.");
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(tempdht, 1);
    lcd.print((char)223);
    lcd.print("C  ");
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(round(hum), 1);
    lcd.print("%");
    lcd.setCursor(0, 2);
    lcd.print("Pressure: ");
    lcd.print(pres, 1);
    lcd.print(" hPa");
    lcd.setCursor(0, 3);
    lcd.print("Temp (BMP): ");
    lcd.print(tempbmp, 1);
    lcd.print((char)223);
    lcd.print("C  ");
}
