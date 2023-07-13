#include "Ws2812.hpp"
#include "ws2812.pio.h"

namespace Rp2040 {

//------------------------------------------------------------------------------
Ws2812::Ws2812(PIO pio, int sm, int pin, bool rgbw) :
    pio_(pio)
{
  auto offset = pio_add_program(pio, &ws2812_program);
  ws2812_program_init(pio, sm, offset, pin, 800000, rgbw);
}

//------------------------------------------------------------------------------
void Ws2812::rgb(uint8_t red, uint8_t green, uint8_t blue)
{
  uint32_t mask = (red << 16) | (green << 8) | (blue << 0);
  pio_sm_put_blocking(pio_, 0, mask << 8u);
}

} // namespace Rp2040
