#include "Gamma.hpp"
#include <cmath>

namespace Rp2040 {

//------------------------------------------------------------------------------
void Gamma::generate(float g)
{
  for (int x = 0; x < 65536; x++) {
    int y = (int)(std::pow((float)x / 65535.0f, g) * 65535.0f + 0.5f);
    if (y > 65535) {
      y = 65535;
    }
    table_[x] = (uint16_t)y;
  }
}

const std::array<uint16_t, 65536>& Gamma::table() const
{
  return table_;
}

} // namespace Rp2040
