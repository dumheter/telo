#include "Arduino.h"

inline void battery_level_begin(int pin)
{
  pinMode(pin, INPUT);
}

/**
 * Is the battey level low and we should go to sleep instantly?
 *
 * @return If true you should enter deep sleep asap.
 */
bool is_battery_low(int pin);

/**
 * Convert ADC values (0-1023) to voltage.
 * Some setup is required to make it work with your ADC.
 */
float get_battery_voltage(int pin);
