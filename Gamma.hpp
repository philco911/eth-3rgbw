#ifndef GAMMA_HPP__
#define GAMMA_HPP__

#include <pico/stdlib.h>
#include <array>

namespace Rp2040 {

//------------------------------------------------------------------------------
class Gamma
{
public:
  void generate(float g);

  const std::array<uint16_t, 65536>& table() const;

private:
  std::array<uint16_t, 65536> table_;
};

} // namespace Rp2040

#endif // GAMMA_HPP__
