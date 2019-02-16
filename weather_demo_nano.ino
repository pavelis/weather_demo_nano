// using BMP085/180 library from Filipe Vieira
#include <BMP085.h>
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
BMP085 bmp;
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
  lcd.print("Starting... ");
  delay(1000);
  dht.begin();
  tempdht = dht.readTemperature();
  hum = dht.readHumidity();
  // Check if any reads failed and exit early (to try again).
  if (isnan(tempdht) || isnan(hum)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.print("DHT failure");
    return;
  }
  bmp.init();
}            

void loop() { 

  delay(2000);
  counter++;
  tempdht = dht.readTemperature();
  hum = dht.readHumidity();

  bmp.getPressure(&pres);
  bmp.getTemperature(&tempbmp);

  tempdhtavg += tempdht;
  humavg += hum;
  tempbmpavg += tempbmp / 10;
  presavg += pres / 100;

  if (counter == AVGNUM) {
    tempdhtavg /= AVGNUM;
    humavg /= AVGNUM;
    tempbmpavg /= AVGNUM;
    presavg /= AVGNUM;
    Serial.print("TempDHT: ");
    Serial.print(tempdhtavg);
    Serial.print((char)223);
    Serial.print("C; ");
    Serial.print("TempBMP: ");
    Serial.print(tempbmpavg);
    Serial.print("; Hum: ");
    Serial.print(humavg);
    Serial.print("%. ");
    Serial.print("Pressure: ");
    Serial.print(presavg);
    Serial.println(" hPa.");
    lcd.clear();
    lcd.print("Temp: ");
    lcd.print(tempdhtavg, 1);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(humavg, 1);
    lcd.print("%");
    lcd.setCursor(0, 2);
    lcd.print("Pressure: ");
    lcd.print(presavg, 1);
    lcd.print(" hPa");
    lcd.setCursor(0, 3);
    lcd.print("Temp (BMP): ");
    lcd.print(tempbmpavg, 1);
    lcd.print((char)223);
    lcd.print("C");
 
    tempdhtavg = 0;
    humavg = 0;
    tempbmpavg = 0;
    presavg = 0;
    counter = 0;
  }
    
}
