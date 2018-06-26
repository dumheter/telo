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
#include "FS.h"
#include <ArduinoJson.h>

#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif

// ============================================================ //
// Batter level sensor
#include "battery_level.hpp"

// Battery voltage
#define battery_level_pin A0

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

const char * const RST_REASONS[] = {
  "REASON_DEFAULT_RST",
  "REASON_WDT_RST",
  "REASON_EXCEPTION_RST",
  "REASON_SOFT_WDT_RST",
  "REASON_SOFT_RESTART",
  "REASON_DEEP_SLEEP_AWAKE",
  "REASON_EXT_SYS_RST"
};

#define ONE_MINUTE 60e6
#define ONE_SECOND 1e6

constexpr uint8_t g_data_points = 48;
constexpr uint8_t g_samples_per_hour = 1;
constexpr unsigned long g_wake_time_ms = 9000e3;
constexpr unsigned long g_sleep_time = ONE_MINUTE * 60 / g_samples_per_hour - g_wake_time_ms;

// ============================================================ //
// Setup
// ============================================================ //

void setup()
{
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  battery_level_begin(battery_level_pin);

#ifdef DEBUG
  constexpr unsigned long BAUDRATE = 74880;
  Serial.begin(BAUDRATE);
  Serial.setDebugOutput(true);
  dprintln();
  dprintln("~ TeLo ~");
  display.init(BAUDRATE);
#else
  display.init();
#endif

#ifndef DEBUG
  if (is_battery_low(battery_level_pin)) {
    draw_low_battery();
    ESP.deepSleep(70*ONE_MINUTE, WAKE_RF_DISABLED);
  }
#else
  dprintln("skipping battery check cause of debug mode");
#endif

  dht.begin();
  dprintln("init done");

  //draw_fake_data();
  update_temperature();

  dprintln("deep sleep");
  //ESP.deepSleep(g_sleep_time, WAKE_RF_DISABLED);
}

// ============================================================ //
// Loop
// ============================================================ //

void loop() {}

// ============================================================ //
// Functions
// ============================================================ //

void debug_print_data(const std::vector<Sensor_data<float, float>>& data)
{
  dprint("data: {");
  for (auto& d : data) {
    dprint("(");
    dprint(d.temp);
    dprint(",");
    dprint(d.hum);
    dprint("), ");
  }
  dprintln("}");
}

void take_measurement(std::vector<Sensor_data<float, float>>& data)
{
  dprintln("taking measurement");

#ifdef DEBUG
  int count = 0;
#endif

  while (data.size() >= g_data_points) {
    data.pop_back();
#ifdef DEBUG
    count++;
#endif
  }

#ifdef DEBUG
  if (count > 1) {
    dprint("warn: deleted ");
    dprint(count);
    dprint(" elements from data.");
    dprint(" Data.size(): ");
    dprintln(data.size());
  }
#endif

  // Measure, throw away the value and measure again after 1 sec.
  // Second measurement is more stable.
  float temp, hum;
  for (int i=0; i<2; i++) {
    constexpr float nan_value = -100;
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

    delay(1000);
  }

  dprint("measurement taken: ");
  dprint(temp);
  dprint(",");
  dprintln(hum);

  data.emplace(data.begin(), temp, hum);
}

void load_sensor_data(std::vector<Sensor_data<float, float>>& data, const char* path)
{
  dprintln("loading sensor data from SPIFFS");

  const auto gen_empty_data = [](std::vector<Sensor_data<float, float>>& _data) {
    for (int i=0; i<g_data_points; i++) {
      _data.emplace(_data.begin(), 0, 0);
    }
  };

  if (SPIFFS.exists(path)) {
    auto f = SPIFFS.open(path, "r");

    const auto fsize = f.size();
    if (f.size() < 1024) {
      std::unique_ptr<int8_t[]> buf(new int8_t[fsize]);
      auto size = f.read(reinterpret_cast<uint8_t*>(buf.get()), fsize);
      for (int i=0; i<size/2; i++) {
        data.emplace_back(static_cast<point_t>(buf[i*2]),
                     static_cast<point_t>(buf[i*2+1]));
      }
      dprint("loaded ");
      dprint(data.size());
      dprintln(" data points");
      debug_print_data(data);
    }
    else {
      dprintln("data file too large");
      gen_empty_data(data);
    }

    f.close();
  }
  else {
    dprintln("data file doesnt exist");
    gen_empty_data(data);
  }
}

void save_sensor_data(const std::vector<Sensor_data<float, float>>& data, const char* path)
{
  dprintln("saving sensor data to SPIFFS");

  size_t bsize = data.size() * 2;
  dprint("setting buffer size to ");
  dprintln(bsize);
  std::unique_ptr<int8_t[]> buf(new int8_t[bsize]);
  int i=0;
  for (const auto& d : data) {
    buf[i*2] = static_cast<int8_t>(lroundf(d.temp));
    buf[i*2+1] = static_cast<int8_t>(lroundf(d.hum));
    i++;
  }

  auto f = SPIFFS.open(path, "w");

  if (f) {
    auto written = f.write(reinterpret_cast<uint8_t*>(buf.get()), bsize);
    dprint("wrote ");
    dprint(written / 2);
    dprint(" data points to SPIFFS");
    dprint(". Total size ");
    dprintln(written);
    debug_print_data(data);
  }

  f.close();
}

void update_temperature()
{
  dprintln("~ update temperature ~");

  std::vector<Sensor_data<float, float>> data{};
  if (!SPIFFS.begin()) {
    dprintln("spiffs init fail");
    return;
  }
  constexpr char* path = "/data.raw";
  load_sensor_data(data, path);
  take_measurement(data);
  draw_sensor_data(data);
  save_sensor_data(data, path);
  SPIFFS.end();
}

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

void draw_fake_data()
{
  dprintln("draw fake sensor data");
  std::vector<Sensor_data<float, float>> fake_data{};
  gen_fake_data(fake_data);
  dprint("fake: ");
  for (auto& d : fake_data) {
    dprint(d.temp);
    dprint(", ");
  }
  dprintln();
  draw_sensor_data(fake_data);
}

void gen_fake_data(std::vector<Sensor_data<float, float>>& fake_data)
{
  constexpr int nr_elem = 48;
  constexpr float temp_koef = 12.5 / nr_elem;
  const float hum_koef = (0.81+millis()%10*0.15) / nr_elem;
  const float offset = millis() / 10000;
  for (int i=0; i<nr_elem; i++) {
    fake_data.emplace(fake_data.begin(),
                      tmap<float>(cosf(offset+i*temp_koef), -1, 1, -2, 15),
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
  dprint("drawing sensor data (");
  dprint(data.size());
  dprintln(" data points)");

  // find max and min temp
  point_t min_temp = data[0].temp;
  point_t max_temp = data[0].temp;
  for (const auto& d : data) {
    if (d.temp > max_temp)
      max_temp = d.temp;
    else if (d.temp < min_temp)
      min_temp = d.temp;
  }
  min_temp = floorf(min_temp);
  max_temp = ceilf(max_temp);

  //constexpr point_t max_temp = 0;
  //constexpr point_t min_temp = -10;
  constexpr point_t max_hum = 100;
  constexpr point_t min_hum = 0;

  display.fillScreen(GxEPD_WHITE);

  // title text
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(30, 10);
  display.setFont(&FreeSans9pt7b);
  //display.println("temperatur");
  display.print("temperatur - ");
  const float v = get_battery_voltage(battery_level_pin);
  display.println(v);

  // graph settings
  constexpr point_t max_height = 200;
  constexpr point_t max_width = 200;
  constexpr point_t margin = 26;
  const Point top_left{margin, 24};
  const Point bot_left{margin, max_height - margin};
  const Point bot_right{max_width - 2, max_height - margin};
  const Point top_right{bot_right.x, top_left.y};

  // draw outlines
  draw_line(top_left, bot_left); // left
  draw_line(bot_left, bot_right); // down
  //draw_line(top_left, top_right, GxEPD_RED); //up
  //draw_line(top_right, bot_right, GxEPD_RED); // right

  // temp axis markings
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(0, top_left.y - 4);
  display.print("*C");
  constexpr int v_marking_slots = 5;
  const float v_spacing = (bot_left.y - top_left.y) / (v_marking_slots+1);
  const float v_spacing_m = (bot_left.y - top_left.y) / (v_marking_slots);
  constexpr float temp_offset_x = 0;
  constexpr float temp_offset_y = 20;
  constexpr point_t line_len = 8;
  for (int i=0; i<=v_marking_slots; i++) {
    display.setCursor(0+temp_offset_x, top_left.y+temp_offset_y + i*v_spacing);
    display.print(tmap<long>(static_cast<long>(i), 0, v_marking_slots, static_cast<long>(max_temp), static_cast<long>(min_temp)));
    const Point a{top_left.x - line_len/2, top_left.y + i*v_spacing_m};
    const Point b{top_left.x + line_len/2, top_left.y + i*v_spacing_m};
    draw_line(a, b);
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
  const float h_spacing = (bot_right.x - bot_left.x - 14) / h_marking_slots;
  const float h_spacing_m = (bot_right.x - bot_left.x) / h_marking_slots;
  constexpr float h_offset_x = -8;
  constexpr float h_offset_y = -4;
  display.setTextColor(GxEPD_BLACK);
  for (int i=0; i<=h_marking_slots; i++) {
    display.setCursor(bot_left.x+h_offset_x + i*h_spacing, max_height+h_offset_y);
    display.print(lroundf(i*(timeframe_hr/h_marking_slots)));
    const Point a{bot_left.x + i*h_spacing_m, bot_left.y - line_len/2};
    const Point b{bot_left.x + i*h_spacing_m, bot_left.y + line_len/2};
    draw_line(a, b);
  }

  // draw sensor data
  const int data_points = data.size();
  const float data_spacing = (bot_right.x - bot_left.x) / data_points;
  constexpr uint16_t radius = 2;
  for (int i=0; i<data_points; i++) {
    float temp = constrain(data[i].temp, min_temp, max_temp);
    float hum = constrain(data[i].hum, min_hum, max_hum);
    display.drawCircle(lroundf(bot_left.x + i*data_spacing),
                       lroundf(tmap(temp, min_temp, max_temp, bot_left.y, top_left.y)),
                       radius, GxEPD_BLACK);
    display.drawCircle(lroundf(bot_left.x + i*data_spacing),
                       lroundf(tmap(hum, min_hum, max_hum, bot_left.y, top_left.y)),
                       radius, GxEPD_RED);
  }

  display.update();
}

void draw_low_battery()
{
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);

  display.setFont(&FreeSans12pt7b);
  display.setCursor(20, 70);
  display.println("Batteri urladdat");

  display.setFont(&FreeSans12pt7b);
  display.setCursor(20, 110);
  display.print("ladda batteriet");

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
