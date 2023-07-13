#ifndef STATUS_HPP__
#define STATUS_HPP__

#include <pico/stdlib.h>
#include <pico/time.h>
#include "Ws2812.hpp"

namespace LedDriver {

//------------------------------------------------------------------------------
class Status
{
public:
  Status(int pin);

  void start();

  void connected(bool con);
  void linkup(bool up);
  void reset(bool reset);
  void announce(bool an);

private:
  static bool tick_callback(struct repeating_timer *timer);
  void tick();

  Rp2040::Ws2812 ws_;
  struct repeating_timer timer_;
  bool connected_{false};
  bool up_{false};
  bool reset_{false};
  bool announce_{false};
  bool tick_{false};
};

} // namespace LedDriver

#endif // STATUS_HPP__
