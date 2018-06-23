/**
 * Just like the arduino implementation but templated:
 * https://www.arduino.cc/reference/en/language/functions/math/map/
 */
template <typename T>
T tmap(T x, T in_min, T in_max, T out_min, T out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
