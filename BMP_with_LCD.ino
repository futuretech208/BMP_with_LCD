#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD
// Wiring: SDA pin is connected to A4 and SCL pin to A5.
// Connect to LCD via I2C, default address 0x27 (A0-A2 not jumpered)
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // Change to (0x27,16,2) for 16x2 LCD.
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

const int pinA = 2;
const int pinB = 3;
float A, B, C;
unsigned long previousMillis = 0; 
const long interval1 = 1000; 
const long interval2 = 10000; 


void setup()
{
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  // Initiate the LCD:
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BMP280 Altimeter");

  if (!bmp.begin(0x76))
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("BMP280 Not Found");
    while (1) delay(10);
  }
  delay(2000);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();

}

void loop()
{


//    sensors_event_t temp_event, pressure_event;
//  bmp_temp->getEvent(&temp_event);
//  bmp_pressure->getEvent(&pressure_event);
  if ((digitalRead(pinA == HIGH)) && (digitalRead(pinB == HIGH)))
{
  lcd.setCursor(0, 0);

    lcd.print("P = ");
   // lcd.print(bmp.readPresssure());
    lcd.println(" hPa  ");

    lcd.setCursor(0, 1);
    lcd.print("Alt = ");
    lcd.print(bmp.readAltitude(1005)); /* Adjusted to local forecast! */
    lcd.print(" m");

    delay(1000);
  }


  if (digitalRead(pinA == LOW))
{
  A = bmp.readAltitude(1005);
  delay(1000);
  }
  if (digitalRead(pinB == LOW))
{
  B = bmp.readAltitude(1005);
    C = B - A;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Difference");
    lcd.setCursor(0, 1);
    lcd.print(C);
    lcd.print(" m");
    delay(10000);
  }
}
