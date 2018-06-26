#include "battery_level.hpp"
#include "util.hpp"

bool is_battery_low(int pin)
{
  constexpr float battery_low_voltage = 3.1;
  return (get_battery_voltage(pin) < battery_low_voltage);
}

float get_battery_voltage(int pin)
{
  constexpr float calibration_offset = 0.05;
  constexpr float voltage_divider_ratio = 1.0f/2.0f;
  constexpr float max_voltage = 3.2;
  return tmap(static_cast<float>(analogRead(pin)), 0.0f, 1023.0f, 0.0f,
              max_voltage / voltage_divider_ratio) - calibration_offset;
}
