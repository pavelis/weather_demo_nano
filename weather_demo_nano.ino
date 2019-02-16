// using Adafruit BMP085 library https://github.com/adafruit/Adafruit-BMP085-Library
#include <Adafruit_BMP085.h>
// using Adafruit DHT library https://github.com/adafruit/Adafruit_Sensor
#include <DHT.h>
#include <Wire.h>
// using LiquidCrystal_I2C library from https://github.com/marcoschwartz/LiquidCrystal_I2C
#include <LiquidCrystal_I2C.h>

#define DHTPIN 7     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define AVGNUM 5 // number of measurements in average value
#define ALT 76000 // altitude is 760 meters

DHT dht (DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp;
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Set the LCD I2C address

byte counter;
float hum, tempdht;
float tempdhtavg, humavg;
long tempbmp, pres;
float tempbmpavg, presavg;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Weather Demo Nano 16 Feb 2019. ");
  delay(1000);
  dht.begin();
  tempdht = dht.readTemperature();
  hum = dht.readHumidity();
  if (isnan(tempdht) || isnan(hum)) {
    Serial.println("DHT failure!");
    lcd.print("DHT failure.");
    return;
  }
  if (!bmp.begin()) {
    Serial.println("BMP failure!");
    lcd.print("BMP failure.");
    return;
  }
  pres = bmp.readPressure() / 100;
  tempbmp = bmp.readTemperature();
  show(tempdht, hum, pres, tempbmp);
}            

void loop() { 

  delay(2000);
  counter++;
  tempdht = dht.readTemperature();
  hum = dht.readHumidity();

  pres = bmp.readPressure();
  tempbmp = bmp.readTemperature();

  tempdhtavg += tempdht;
  humavg += hum;
  tempbmpavg += tempbmp;
  presavg += pres / 100;

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
    lcd.clear();
    lcd.print("Temp: ");
    lcd.print(tempdht, 1);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(hum, 1);
    lcd.print("%");
    lcd.setCursor(0, 2);
    lcd.print("Pressure: ");
    lcd.print(pres, 1);
    lcd.print(" hPa");
    lcd.setCursor(0, 3);
    lcd.print("Temp (BMP): ");
    lcd.print(tempbmp, 1);
    lcd.print((char)223);
    lcd.print("C");
}
