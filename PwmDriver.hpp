#ifndef PWMDRIVER_HPP__
#define PWMDRIVER_HPP__

#include <pico/stdlib.h>
#include <pico/time.h>
#include <array>
#include "PwmBank.hpp"
#include "Regs.hpp"

namespace LedDriver {

//------------------------------------------------------------------------------
template<std::size_t N>
class PwmDriver
{
public:
  PwmDriver(Rp2040::PwmBank<N> &pwm, const RegsView<N * 2> &regs) : pwm_(pwm), regs_(regs) {
    shadow_.fill(0);
  }

  void start(int rate, bool stack) {
    stack_ = stack;
    add_repeating_timer_ms(-(1000 / rate), PwmDriver<N>::tick_callback, this, &timer_);
  }

private:
  static bool tick_callback(struct repeating_timer *timer) {
    PwmDriver<N> *pwm = (PwmDriver<N>*)(timer->user_data);
    pwm->tick();
    return true;
  }

  void tick() {
    for (std::size_t idx = 0; idx < N; idx++) {
      auto current = pwm_.level(idx);
      uint16_t target, step;
      // stacking, or maybe double buffering, only pulls a target from the regs if
      // the previous target has been reached. This allows for a very limited action queue.
      if (stack_) {
        target = shadow_[idx << 1];
        step = shadow_[(idx << 1) + 1];
      } else {
        target = regs_.value(idx << 1);
        step = regs_.value((idx << 1) + 1);
      }
      if (current < target) {
        auto delta = target - current;
        if (delta < step) {
          step = delta;
        }
        pwm_.level(idx, current + step);
      } else if (current > target) {
        auto delta = current - target;
        if (delta < step) {
          step = delta;
        }
        pwm_.level(idx, current - step);
      } else if (stack_) {
        shadow_[idx << 1] = regs_.value(idx << 1);
        auto nstep = regs_.value((idx << 1) + 1);
        if (nstep < kMinStackStep) { // since new value won't load until done, protect against excessive ramp time
          nstep = kMinStackStep;
        }
        shadow_[(idx << 1) + 1] = nstep;
      }
    }
  }

  static constexpr uint16_t kMinStackStep{30};

  Rp2040::PwmBank<N> &pwm_;
  RegsView<N * 2> regs_;
  std::array<uint16_t, N * 2> shadow_;
  bool stack_{false};
  struct repeating_timer timer_;
};

} // namespace LedDriver

#endif // PWMDRIVER_HPP__
