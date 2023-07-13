#ifndef WS2812_HPP__
#define WS2812_HPP__

#include <pico/stdlib.h>
#include <hardware/pio.h>

namespace Rp2040 {

//------------------------------------------------------------------------------
class Ws2812
{
public:
  Ws2812(PIO pio, int sm, int pin, bool rgbw);

  void rgb(uint8_t red, uint8_t green, uint8_t blue);

private:
  PIO pio_;
};

} // namespace Rp2040

#endif // WS2812_HPP__
