inline unsigned long get_time_ms()
{
  return millis();
}

template <typename T, typename F>
class Scheduler
{
public:
  Scheduler(unsigned long interval_ms, F func) :
    m_interval(interval_ms), m_func(func) {}

  T& get() { return m_data; }

  bool update();

private:
  T m_data{};
  const unsigned long m_interval;
  unsigned long m_timer = 0;
  F m_func;

};

template <typename T, typename F>
bool Scheduler<T, F>::update()
{
  if (get_time_ms() - m_timer > m_interval) {
    m_timer = get_time_ms();
    m_func(m_data);
    return true;
  }
  return false;
}
