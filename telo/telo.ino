// ============================================================ //
// Includes and Globals
// ============================================================ //

// ============================================================ //
// various
#include <math.h>
#include "scheduler.hpp"
#include <vector>

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

template <typename T1, typename T2>
struct Sensor_data
{
public:
  Sensor_data(T1 new_temp, T2 new_hum)
    : temp(new_temp), hum(new_hum), timestamp(millis()) {}

  T1 temp;
  T2 hum;
  unsigned long timestamp;
};

typedef float point_t;
typedef uint8_t uint_point_t;

struct Point
{
  Point(point_t new_x, point_t new_y) : x(new_x), y(new_y) {}
  point_t x, y;
};

uint_point_t point_to_uint(point_t point)
{
  return static_cast<uint_point_t>(lround(point));
}

/**
 * functor that poplulates a queue of data points with
 * temperature and humidity data.
 */
struct measure
{
  void operator()(std::vector<Sensor_data<float, float>>& data)
  {
    constexpr float nan_value = -100;
    float temp, hum;
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      temp = nan_value;
    }
    else {
      temp = event.temperature;
    }

    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      hum = nan_value;
    }
    else {
      hum = event.relative_humidity;
    }

    Serial.print("measurement taken: ");
    Serial.print(temp);
    Serial.print(",");
    Serial.println(hum);

    data.emplace(data.begin(), temp, hum);
  }
};

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
  constexpr unsigned long measure_interval_ms = 5000;
  static Scheduler<std::vector<Sensor_data<float, float>>, measure>
    scheduler{measure_interval_ms, measure()};
  if (scheduler.update()){
    Serial.println("scheduler updated, draw sensor data");

    static int draw_interval = 10;

    if (draw_interval++ > 5) {
      draw_interval = 0;
      draw_sensor_data(scheduler.get());
    }
  }
}

// ============================================================ //
// Functions
// ============================================================ //

void draw_line(const Point& a, const Point& b, uint16_t color = GxEPD_BLACK)
{
  const float length_x = (a.x - b.x) * (-1);
  const float length_y = (a.y - b.y) * (-1);
  const float length = sqrtf(pow(a.x-b.x, 2) + pow(a.y-b.y, 2));
  const float dx = length_x / length;
  const float dy = length_y / length;
  for (point_t i=0; i<=length; i++) {
    display.drawPixel(a.x + dx*i, a.y + dy*i, color);
  }
}

void draw_sensor_data(const std::vector<Sensor_data<float, float>>& data)
{
  display.fillScreen(GxEPD_WHITE);

  // title text
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(0, 10);
  display.setFont(&FreeSans9pt7b);
  display.println("   temperatur");

  // graph lines
  constexpr point_t max_height = 200;
  constexpr point_t max_width = 200;
  constexpr point_t margin = 24;
  const Point top_left{margin, 24};
  const Point bot_left{margin, max_height - margin};
  const Point bot_right{max_width - margin, max_height - margin};
  const Point top_right{bot_right.x, top_left.y};
  draw_line(top_left, bot_left);
  draw_line(bot_left, bot_right);
  draw_line(top_left, top_right, GxEPD_RED);
  draw_line(top_right, bot_right, GxEPD_RED);

  // graph markings
  display.setCursor(0, top_left.y + 15);
  display.print("*C");
  display.setCursor(0, top_left.y + 45);
  display.print("30");

  display.setTextColor(GxEPD_RED);
  display.setCursor(top_right.x+2, top_right.y + 15);
  display.print("%");
  display.setCursor(top_right.x+2, top_left.y + 45);
  display.print("80");

  display.setTextColor(GxEPD_BLACK);
  display.setCursor(bot_left.x-8, max_height-4);
  display.print("24");

  // draw sensor data
  constexpr point_t max_temp = 40;
  constexpr point_t min_temp = -10;
  constexpr point_t max_hum = 90;
  constexpr point_t min_hum = 10;
  for (auto& d : data) {
    float temp = constrain(d.temp, min_temp, max_temp);
    float hum = constrain(d.hum, min_hum, max_hum);
  }

  display.update();
}

void draw_temp(float temp)
{
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);

  display.setFont(&FreeSans18pt7b);
  display.setCursor(60, 30);
  display.println("TeLo");

  display.setFont(&FreeSans12pt7b);
  display.setCursor(30, 80);
  display.print("temp ");
  display.setTextColor(GxEPD_RED);
  display.print(static_cast<int>(round(temp)));
  display.setTextColor(GxEPD_BLACK);
  display.println(" *C");

  display.update();
}
