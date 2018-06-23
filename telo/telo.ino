// ============================================================ //
// Includes and Globals
// ============================================================ //

// ============================================================ //
// various
#include <math.h>
#include "scheduler.hpp"
#include "util.hpp"
#include <vector>
#include "ESP8266WiFi.h"

#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif

// ============================================================ //
// Debug
#define DEBUG

#ifdef DEBUG
#  define dprintln(...) Serial.println(__VA_ARGS__)
#  define dprint(...) Serial.print(__VA_ARGS__)
#else
#  define dprintln(...)
#  define dprint(...)
#endif

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
#include <Fonts/Org_01.h>

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
// Battery voltage
#define battery_level_pin A0

// ============================================================ //

typedef float point_t;
typedef uint8_t uint_point_t;

template <typename T1, typename T2>
struct Sensor_data
{
public:
  Sensor_data(T1 new_temp, T2 new_hum) : temp(new_temp), hum(new_hum) {};

  T1 temp;
  T2 hum;
};

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
    constexpr uint16_t max_data_points = 20;
    if (data.size() > max_data_points) {
      data.pop_back();
    }

    constexpr float nan_value = -100;
    float temp, hum;
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      temp = nan_value;
    }
    else {
      constexpr float calibration_offset = -1.0f;
      temp = event.temperature + calibration_offset;
    }

    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      hum = nan_value;
    }
    else {
      hum = event.relative_humidity;
    }

    dprint("measurement taken: ");
    dprint(temp);
    dprint(",");
    dprintln(hum);

    data.emplace(data.begin(), temp, hum);
  }
};

const char * const RST_REASONS[] = {
  "REASON_DEFAULT_RST",
  "REASON_WDT_RST",
  "REASON_EXCEPTION_RST",
  "REASON_SOFT_WDT_RST",
  "REASON_SOFT_RESTART",
  "REASON_DEEP_SLEEP_AWAKE",
  "REASON_EXT_SYS_RST"
};

// ============================================================ //
// Setup
// ============================================================ //

void setup()
{
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  pinMode(battery_level_pin, INPUT);

  // if (is_battery_low()) {
  //   display_battery_low();
  //   ESP.deepSleep(10e6, WAKE_RF_DISABLED);
  // }

#ifdef DEBUG
  constexpr unsigned long BAUDRATE = 74880;
  Serial.begin(BAUDRATE);
  Serial.setDebugOutput(true);
  //Serial.setTimeout(2000);
  dprintln();
  dprintln("~ TeLo ~");
  display.init(BAUDRATE);
#else
  display.init();
#endif

  dht.begin();

  dprintln("up and running");

  check_battery_level();

#ifdef DEBUG
  debug_test_all_components();
#endif

  //delay(5000);
  dprintln("deep sleep");
  //ESP.deepSleep(10e6, WAKE_RF_DISABLED);
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
    dprintln("scheduler updated, draw sensor data");

    static int draw_interval = 10;

    if (draw_interval++ > 5) {
      draw_interval = 0;
      std::vector<Sensor_data<float, float>> fake_data{};
      gen_fake_data(fake_data);
      //draw_sensor_data(scheduler.get());
      dprint("fake: ");
      for (auto& d : fake_data) {
        dprint(d.temp);
        dprint(", ");
      }
      dprintln();
      draw_sensor_data(fake_data);
    }
  }
}

// ============================================================ //
// Functions
// ============================================================ //

void debug_test_all_components()
{
  dprintln("~ debug test all components ~");

  // temp
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
  dprint("temp sensor: ");
  dprint(temp);
  dprint(",");
  dprintln(hum);

  // display
  debug_draw();

  // voltage sensor
  dprint("voltage: ");
  dprintln(get_battery_voltage(battery_level_pin));

  dprintln("~ done with test ~");
  delay(2000);
}

/**
 * Is the battey level low and we should go to sleep instantly?
 *
 * @return If true you should enter deep sleep asap.
 */
bool is_battery_low()
{
  constexpr float battery_low_voltage = 3.1;
  return (get_battery_voltage(battery_level_pin) < battery_low_voltage);
}

void check_battery_level()
{

}

float get_battery_voltage(int pin)
{
  constexpr float calibration_offset = 0.05;
  constexpr float voltage_divider_ratio = 1.0f/2.0f;
  constexpr float max_voltage = 3.2;
  return tmap(static_cast<float>(analogRead(pin)), 0.0f, 1023.0f, 0.0f,
              max_voltage / voltage_divider_ratio) - calibration_offset;
}

void gen_fake_data(std::vector<Sensor_data<float, float>>& fake_data)
{
  constexpr int nr_elem = 30;
  constexpr float temp_koef = 6.28 / nr_elem;
  const float hum_koef = (0.81+millis()%10*0.15) / nr_elem;
  const float offset = millis() / 10000;
  for (int i=0; i<nr_elem; i++) {
    fake_data.emplace(fake_data.begin(), tmap<float>(cosf(offset+i*temp_koef), -1, 1, 18, 38),
                      tmap<float>(sinf(offset+i*hum_koef), -1, 1, 58, 83));
  }
}

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
  display.setCursor(60, 10);
  display.setFont(&FreeSans9pt7b);
  display.println("temperatur");

  // graph settings
  constexpr point_t max_height = 200;
  constexpr point_t max_width = 200;
  constexpr point_t margin = 26;
  const Point top_left{margin, 24};
  const Point bot_left{margin, max_height - margin};
  const Point bot_right{max_width - 2, max_height - margin};
  const Point top_right{bot_right.x, top_left.y};

  // draw outlines
  draw_line(top_left, bot_left);
  draw_line(bot_left, bot_right);
  draw_line(top_left, top_right, GxEPD_RED);
  draw_line(top_right, bot_right, GxEPD_RED);

  // temp axis markings
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(0, top_left.y - 4);
  display.print("*C");
  constexpr int v_marking_slots = 5;
  constexpr point_t v_shrink = 6;
  const float v_spacing = (bot_right.x - bot_left.x - v_shrink) / (v_marking_slots+1);
  constexpr float temp_offset_x = 0;
  constexpr float temp_offset_y = 14;
  constexpr point_t max_temp = 40;
  constexpr point_t min_temp = -10;
  constexpr point_t max_hum = 90;
  constexpr point_t min_hum = 10;
  for (int i=0; i<=v_marking_slots; i++) {
    display.setCursor(0+temp_offset_x, top_left.y+temp_offset_y + i*v_spacing);
    display.print(lroundf(max_temp - i*((abs(max_temp)+abs(min_temp))/v_marking_slots)));
  }

  // hum axis markings
  display.setTextColor(GxEPD_RED);
  display.setCursor(top_right.x-margin+2, top_right.y - 4);
  display.print("%");
  // constexpr float hum_offset_x = 2;
  // constexpr float hum_offset_y = 14;
  // for (int i=0; i<=v_marking_slots; i++) {
  //   display.setCursor(top_right.x+hum_offset_x, top_right.y+hum_offset_y + i*v_spacing);
  //   display.print(lroundf(max_hum - i*((abs(max_hum)+abs(min_hum))/v_marking_slots)));
  // }

  // time axis markings
  constexpr float timeframe_hr = 24;
  constexpr int h_marking_slots = 5;
  const float h_spacing = (bot_right.x - bot_left.x - margin) / h_marking_slots;
  constexpr float h_offset_x = -8;
  constexpr float h_offset_y = -4;
  display.setTextColor(GxEPD_BLACK);
  for (int i=0; i<=h_marking_slots; i++) {
    display.setCursor(bot_left.x+h_offset_x + i*h_spacing, max_height+h_offset_y);
    display.print(lroundf(i*(timeframe_hr/h_marking_slots)));
  }

  // draw sensor data
  const int data_points = data.size();
  const float data_spacing = (bot_right.x - bot_left.x) / data_points;
  constexpr uint16_t radius = 2;
  for (int i=0; i<data_points; i++) {
    float temp = constrain(data[i].temp, min_temp, max_temp);
    float hum = constrain(data[i].hum, min_hum, max_hum);
    display.drawCircle(bot_left.x + i*data_spacing,
                       top_left.y + tmap(temp, min_temp, max_temp, bot_left.y, top_left.y),
                       radius, GxEPD_BLACK);
    display.drawCircle(bot_left.x + i*data_spacing,
                       top_left.y + tmap(hum, min_hum, max_hum, bot_left.y, top_left.y),
                       radius, GxEPD_RED);
  }

  display.update();
}

void debug_draw()
{
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);

  display.setFont(&FreeSans18pt7b);
  display.setCursor(60, 30);
  display.println("TeLo");

  display.setFont(&FreeSans12pt7b);
  display.setCursor(30, 80);
  display.print("abc ");
  display.setTextColor(GxEPD_RED);
  display.print(123);
  display.setTextColor(GxEPD_BLACK);
  display.println(" wow");

  display.update();
}
