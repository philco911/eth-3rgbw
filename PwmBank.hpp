#ifndef PWMBANK_HPP__
#define PWMBANK_HPP__

#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <array>
#include <set>


namespace Rp2040 {

//------------------------------------------------------------------------------
template<std::size_t N>
class PwmBank
{
public:
  template<typename... T>
  PwmBank(T... ts) : gpio_{{ts...}}
  {
    level_.fill(0);

    std::set<uint> slice;
    for (const auto &io : gpio_) {
      gpio_set_function(io, GPIO_FUNC_PWM);
      slice.insert(pwm_gpio_to_slice_num(io));
    }
    for (const auto &sl : slice) {
      pwm_config config = pwm_get_default_config();
      pwm_config_set_phase_correct(&config, true);
      pwm_init(sl, &config, true);
    }
    for (std::size_t idx = 0; idx < N; idx++) {
      pwm_set_gpio_level(gpio_[idx], level_[idx]);
    }
  }

  void gamma(const std::array<uint16_t, 65536> &g) {
    gamma_ = g.data();
  }

  void level(std::size_t channel, uint16_t level) {
    level_[channel] = level;
    if (gamma_) {
      pwm_set_gpio_level(gpio_[channel], gamma_[level]);
    } else {
      pwm_set_gpio_level(gpio_[channel], level);
    }
  }

  uint16_t level(std::size_t channel) const {
    return level_[channel];
  }

private:
  std::array<int, N> gpio_;
  std::array<uint16_t, N> level_;
  const uint16_t *gamma_{nullptr};
};

} // namespace Rp2040

#endif // PWMBANK_HPP__
