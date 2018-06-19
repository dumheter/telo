// ============================================================ //
// Includes and Globals
// ============================================================ //

// ============================================================ //
// Temperature Sensor
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHT_PIN 5
#define DHT_TYPE DHT22
DHT_Unified dht(DHT_PIN, DHT_TYPE);

// ============================================================ //
// Display
#include <GxEPD.h>
#include <GxGDEW0154Z04/GxGDEW0154Z04.cpp>

#include GxEPD_BitmapExamples

// FreeFonts from Adafruit_GFX
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>

#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>

// generic/common.h
//static const uint8_t SS    = 15; // D8
//static const uint8_t MOSI  = 13; // D7
//static const uint8_t MISO  = 12; // D6
//static const uint8_t SCK   = 14; // D5

GxIO_Class io(SPI, /*CS=D8*/ SS, /*DC=D3*/ 0, /*RST=D4*/ 2); // arbitrary selection of D3(=0), D4(=2), selected for default of GxEPD_Class
GxEPD_Class display(io /*RST=D4*/ /*BUSY=D2*/); // default selection of D4(=2), D2(=4)

// ============================================================ //
// Setup
// ============================================================ //

void setup()
{
  constexpr unsigned long BAUDRATE = 115200;
  Serial.begin(BAUDRATE);
  Serial.println();
  Serial.println("telo");

  dht.begin();

  display.init(BAUDRATE);

  Serial.println("setup done");
}

// ============================================================ //
// Loop
// ============================================================ //

void loop()
{
  static int c = 0;
  Serial.print(c++);
  Serial.println("\trunning");
  delay(8000);

  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    Serial.print("Temperature: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
    draw_temp(event.temperature);
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    Serial.print("Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }
}

// ============================================================ //
// Functions
// ============================================================ //

void draw_temp(float temp)
{
  Serial.println("draw temp");

  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);

  display.setFont(&FreeSans18pt7b);
  display.setCursor(60, 30);
  display.println("TeLo");

  display.setFont(&FreeSans12pt7b);
  display.setCursor(30, 80);
  display.print("temp ");
  display.print(temp);
  display.println("*C");

  display.setFont(&FreeSans9pt7b);
  display.setCursor(30, 150);
  display.print("count ");
  static int c = 0;
  display.println(c++);

  display.update();
  Serial.println("draw done");
}
