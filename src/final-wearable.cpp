#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>


Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);

#define NEOPIXEL_PIN 5  // GPIO pin connected to the NeoPixel
#define NUMPIXELS 1     // Number of pixels

Adafruit_NeoPixel strip(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);


float temperature;
float humidity;
float pressure;
float altitude;

float const ALTITUDE = 207; // Altitude at Plano TX
float const SEA_LEVEL_PRESSURE = 1013.25;  // Average pressure at sea level

// Buffer for the moving average filter
const int bufferSize = 10;
float tempBuffer[bufferSize];
float accelXBuffer[bufferSize];
float accelYBuffer[bufferSize];
float accelZBuffer[bufferSize];

int bufferIndex = 0;


Adafruit_BME280 bme;

void getTemperature() {
  float temperature = bme.readTemperature();

  // Update buffer with current temperature
  tempBuffer[bufferIndex] = temperature;
  bufferIndex = (bufferIndex + 1) % bufferSize;

  // Calculate moving average
  float avgTemp = 0;
  for (int i = 0; i < bufferSize; i++) {
      avgTemp += tempBuffer[i];
  }
  avgTemp /= bufferSize;

  String temperatureString = String(avgTemp, 1);  // One decimal position
  Serial.print("Temperature: ");
  Serial.print(avgTemp);
  Serial.println("C");
  display.print("Temperature: ");
  display.print(avgTemp);
  display.println("C");
}
//===============================================================================
//  getHumidity - Subroutine to get and print humidity
//===============================================================================
void getHumidity() {
  humidity = bme.readHumidity();
  String humidityString = String(humidity, 0);
  Serial.print("Humidity: ");
  Serial.print(humidityString);
  Serial.println("%");
  display.print("Humidity: ");
  display.print(humidityString);
  display.println("%");
}
//===============================================================================
//  getPressure - Subroutine to get and print pressure
//===============================================================================
void getPressure() {
  pressure = bme.readPressure();
  pressure = bme.seaLevelForAltitude(ALTITUDE, pressure);
  pressure = pressure / 3386.39;  // Convert hPa to in/Hg
  String pressureString = String(pressure, 2);
  Serial.print("Pressure: ");
  Serial.print(pressureString);
  Serial.println("in");
  display.print("Pressure: ");
  display.print(pressureString);
  display.println("in");
}
//===============================================================================
//  getAltitude - Subroutine to get and print temperature
//===============================================================================
void getAltitude() {
  altitude = bme.readAltitude(SEA_LEVEL_PRESSURE);
  altitude = altitude * 3.28084;  // Convert meters to feet
  String altitudeString = String(altitude, 0);
  Serial.print("Altitude: ");
  Serial.print(altitudeString);
  Serial.println("ft");
  display.print("Altitude: ");
  display.print(altitudeString);
  display.println("ft");
}

void setPixelColor(uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(0, strip.Color(red, green, blue));
  strip.show();
}

void setGradientColor(float xValue) {
  // Map the xValue from -9.8 to 9.8 to 0 to 255
  int blueValue = map(xValue * 100, -980, 980, 255, 0); 
  int redValue = 255 - blueValue;

  // Ensure the values are within the 0-255 range
  blueValue = constrain(blueValue, 0, 255);
  redValue = constrain(redValue, 0, 255);

  setPixelColor(redValue, 0, blueValue); // Set the pixel color
}

// Function to change LED color based on temperature
void changeLedColorBasedOnTemperature(float temp) {
  if (temp > 20.0) {
    setPixelColor(255, 0, 0); // Red color for high temperature
  }
}

float calculateAverage(float *buffer, int size) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

void setup() {
  Serial.begin(115200);
  // while (!Serial);

  Serial.println("Reading BME");

  bool status;

  // default settings
  status = bme.begin(0x76);  // The I2C address of the sensor is 0x76
  if (!status) {             // Loop if sensor not found
    Serial.print("Error. Check BME280 Connections");
    while (1)
      ;
  }
  Serial.println("MPU6050 OLED demo");

  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.display();
  delay(500);  // Pause for 2 seconds
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);

  strip.begin();
  strip.show();  // Initialize all pixels to 'off'
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  display.clearDisplay();
  display.setCursor(0, 0);

  setGradientColor(a.acceleration.x);

  accelXBuffer[bufferIndex] = a.acceleration.x;
  accelYBuffer[bufferIndex] = a.acceleration.y;
  accelZBuffer[bufferIndex] = a.acceleration.z;
  bufferIndex = (bufferIndex + 1) % bufferSize;

  float avgAccelX = calculateAverage(accelXBuffer, bufferSize);
  float avgAccelY = calculateAverage(accelYBuffer, bufferSize);
  float avgAccelZ = calculateAverage(accelZBuffer, bufferSize);

  Serial.print("Accelerometer ");
  Serial.print("X: ");
  Serial.print(avgAccelX, 1);
  Serial.print(" m/s^2, ");
  Serial.print("Y: ");
  Serial.print(avgAccelY, 1);
  Serial.print(" m/s^2, ");
  Serial.print("Z: ");
  Serial.print(avgAccelZ, 1);
  Serial.println(" m/s^2");

  display.println("Accelerometer - m/s^2");
  display.print(avgAccelX, 2);
  display.print(", ");
  display.print(avgAccelY, 2);
  display.print(", ");
  display.print(avgAccelZ, 2);
  display.println("");

  Serial.print("Gyroscope ");
  Serial.print("X: ");
  Serial.print(g.gyro.x, 1);
  Serial.print(" rps, ");
  Serial.print("Y: ");
  Serial.print(g.gyro.y, 1);
  Serial.print(" rps, ");
  Serial.print("Z: ");
  Serial.print(g.gyro.z, 1);
  Serial.println(" rps");

  display.println("Gyroscope - rps");
  display.print(g.gyro.x, 2);
  display.print(", ");
  display.print(g.gyro.y, 2);
  display.print(", ");
  display.print(g.gyro.z, 2);
  display.println("");

  getPressure();  // Get sensor data and print to serial monitor window
  getHumidity();
  getTemperature();
  changeLedColorBasedOnTemperature(temperature);  // Change LED color based on temperature
  getAltitude();


  display.display();
  delay(100);
}


