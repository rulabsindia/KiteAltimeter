/* This program logs bmp280 data to an external memory card via an SD card module.
 * Output formats: .txt, .csv
 */

#include <SPI.h>        // Include SPI library (needed for the SD card)
#include <SD.h>         // Include SD library
#include <Adafruit_BMP280.h>
 
File dataCsv;
File dataTxt;
int sample_interval = 250; //milliseconds
 
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

#define LED_PIN_1 9  //SD card Initialized
#define LED_PIN_2 8  // Sensor initialized
#define LED_PIN_3 7

Adafruit_BMP280 bmp; // I2C

  byte degree_symbol[8] = 
              {
                0b00111,
                0b00101,
                0b00111,
                0b00000,
                0b00000,
                0b00000,
                0b00000,
                0b00000
              };
 
void setup() {

    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);
    pinMode(LED_PIN_3, OUTPUT);
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB port only
  Serial.print("Initializing SD card...");
  if (!SD.begin()) 
  {
    Serial.println("Failed!");
    while (1);
  }
  Serial.println("Success!\n");
  Serial.println("***********************************");
  Serial.println(F("BMP280 Logger by skg-1"));
  Serial.println("***********************************");
  digitalWrite(LED_PIN_1, HIGH);
  delay(1000);
  //digitalWrite(LED_PIN_1, LOW);

  //checking for a valid BMP280 sensor
  if (bmp.begin(0x76)) {
    Serial.println(F("\nFound a valid BMP280 sensor at 0x76...\n"));
    delay(1000);
    
  }

  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  delay(5000);
  //dht.begin();
}
 
uint16_t line = 1;
 
void loop() {
  delay(sample_interval);
  // Read humidity
  //byte RH = dht.readHumidity();
  //Read temperature in degree Celsius
  //byte Temp = dht.readTemperature();

   float temp = bmp.readTemperature();
   float pressure = bmp.readPressure();
   float alt = bmp.readAltitude(1013.25);
  
  dataTxt = SD.open("BMP280.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (dataTxt) 
  {
    Serial.print(line);
     Serial.print(":    Temperature = ");
    Serial.print(temp);
    Serial.print("°C,    Pressure = ");
    Serial.print(pressure/100);
    Serial.println("hPa,   Altitude = ");
    Serial.print(alt);
     Serial.println("m");
    // Write data to SD card file (DHT11Log.txt)
    dataTxt.print(line++);
    dataTxt.print(":    Temperature = ");
    dataTxt.print(temp);
    dataTxt.print("°C,    Pressure = ");
    dataTxt.print(pressure/100);
    dataTxt.print("hPa,    Altitude = ");    
    dataTxt.print(alt);
    dataTxt.println("m");
    dataTxt.close();
    
  }

  dataCsv = SD.open("BMP280.csv", FILE_WRITE);
  
   if (dataCsv) 
  {
    // Write data to SD card file (DHT11Log.txt)

    dataCsv.print(temp);
    dataCsv.print(",");
    dataCsv.print(pressure/100);
    dataCsv.print(",");    
    dataCsv.print(alt);
    dataCsv.print("\n"); 
    dataCsv.close();

     digitalWrite(LED_PIN_2, HIGH);
     delay(1000);
     digitalWrite(LED_PIN_2, LOW);
     
    delay(5000);
  }
  // if the file didn't open, print an error:
  else
    Serial.println("Error opening BMP280.txt & BMP280.csv !");
}
