#ifndef CH9120ETH_HPP___
#define CH9120ETH_HPP___

#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/uart.h>
#include <hardware/irq.h>
#include <array>

namespace Rp2040 {

//------------------------------------------------------------------------------
extern void *ch9120eth_self;

//------------------------------------------------------------------------------
template<typename S, typename H>
class Ch9120Eth
{
public:
  Ch9120Eth(uart_inst_t *uart, int res, int cfg, int tcp, S &status, H &handler) : uart_(uart), res_(res),
      cfg_(cfg), tcp_(tcp), status_(status), handler_(handler) {
  }

  void start() {
    gpio_put(cfg_, 1);
    gpio_put(res_, 0);
    sleep_ms(kDelay);
    gpio_put(res_, 1);
    sleep_ms(kDelay);

    uart_init(uart_, kBaud);

    status_.linkup(true);
    status_.connected(false);

    while (uart_is_readable(uart_)) {
      (void)uart_getc(uart_); // flush
    }

    ch9120eth_self = this;
    auto irq = (uart_ == uart0)?UART0_IRQ:UART1_IRQ;
    irq_set_exclusive_handler(irq, Ch9120Eth<S,H>::isr);
    irq_set_enabled(irq, true);
    uart_set_irq_enables(uart_, true, false);

    add_repeating_timer_ms(1000, Ch9120Eth<S, H>::tick_callback, this, &timer_);
  }

  void config(uint32_t ip, uint32_t nm, uint32_t gw) {
    uart_set_baudrate(uart_, kCfgBaud);
    uart_set_irq_enables(uart_, false, false);

    sleep_ms(kDelay);
    gpio_put(cfg_, 0);
    sleep_ms(kDelay);

    sendcmd(std::array<uint8_t, 2>{kSetMode, 0x00});
    if (ip == 0) {
      auto port = handler_.port();
      sendcmd(std::array<uint8_t, 3>{kSetLocalPort, static_cast<uint8_t>(port & 0xff), static_cast<uint8_t>((port >> 8) & 0xff)});
      sendcmd(std::array<uint8_t, 2>{kSetDhcp, 0x01});
    } else {
      sendcmd(std::array<uint8_t, 2>{kSetDhcp, 0x00});
      sendcmd(std::array<uint8_t, 5>{kSetIp, static_cast<uint8_t>((ip >> 24) & 0xff), static_cast<uint8_t>((ip >> 16) & 0xff),
          static_cast<uint8_t>((ip >> 8) & 0xff), static_cast<uint8_t>(ip & 0xff)});
      sendcmd(std::array<uint8_t, 5>{kSetNetmask, static_cast<uint8_t>((nm >> 24) & 0xff), static_cast<uint8_t>((nm >> 16) & 0xff),
          static_cast<uint8_t>((nm >> 8) & 0xff), static_cast<uint8_t>(nm & 0xff)});
      sendcmd(std::array<uint8_t, 5>{kSetGateway, static_cast<uint8_t>((gw >> 24) & 0xff), static_cast<uint8_t>((gw >> 16) & 0xff),
          static_cast<uint8_t>((gw >> 8) & 0xff), static_cast<uint8_t>(gw & 0xff)});
      auto port = handler_.port();
      sendcmd(std::array<uint8_t, 3>{kSetLocalPort, static_cast<uint8_t>(port & 0xff), static_cast<uint8_t>((port >> 8) & 0xff)});
    }

    sendcmd(std::array<uint8_t, 2>{kSetClear, 0x01});
    sendcmd(std::array<uint8_t, 5>{kSetBaud, kBaud & 0xff, (kBaud >> 8) & 0xff, (kBaud >> 16) & 0xff, (kBaud >> 24) & 0xff});
    sendcmd(std::array<uint8_t, 1>{kSaveEeprom});
    sleep_ms(kDelay * 2);
    sendcmd(std::array<uint8_t, 1>{kApply});
    sleep_ms(kDelay * 2);

    gpio_put(cfg_, 1);
    uart_set_baudrate(uart_, kBaud);
    uart_set_irq_enables(uart_, true, true);
  }

private:
  static constexpr uint8_t kGetTcpConnected{0x03};
  static constexpr uint8_t kSaveEeprom{0x0d};
  static constexpr uint8_t kApply{0x0e};
  static constexpr uint8_t kExitCfg{0x5e};
  static constexpr uint8_t kSetMode{0x10};
  static constexpr uint8_t kSetIp{0x11};
  static constexpr uint8_t kSetNetmask{0x12};
  static constexpr uint8_t kSetGateway{0x13};
  static constexpr uint8_t kSetLocalPort{0x14};
  static constexpr uint8_t kSetBaud{0x21};
  static constexpr uint8_t kSetClear{0x26};
  static constexpr uint8_t kSetDhcp{0x33};
  static constexpr int kDelay{100};
  static constexpr int kCfgBaud{9600};
  static constexpr int kBaud{460800};

  template<std::size_t N>
  void sendcmd(const std::array<uint8_t, N> &values) {
    uint8_t hdr[] = {0x57, 0xab};
    uint8_t res = 0;

    uart_write_blocking(uart_, hdr, 2);
    uart_write_blocking(uart_, values.data(), values.size());
    sleep_ms(kDelay);
  }

  static void isr() {
    Ch9120Eth<S, H> *ch = (Ch9120Eth<S, H>*)ch9120eth_self;
    ch->handle_isr();
  }

  void handle_isr() {
    bool didwork = false;
    do {
      didwork = false;
      if (uart_is_readable(uart_)) {
        handler_.data(uart_getc(uart_));
        didwork = true;
      }
      if (uart_is_writable(uart_) && handler_.has_data()) {
        uart_putc_raw(uart_, handler_.data());
        didwork = true;
      }
    } while (didwork);

    if (handler_.has_data()) {
      uart_set_irq_enables(uart_, true, true);
    } else {
      uart_set_irq_enables(uart_, true, false);
    }
  }

  static bool tick_callback(struct repeating_timer *timer) {
    Ch9120Eth<S, H> *ch = (Ch9120Eth<S, H>*)(timer->user_data);
    ch->tick();
    return true;
  }

  void tick() {
    status_.connected(gpio_get(tcp_) == 0);
  }

  uart_inst_t *uart_;
  int res_;
  int cfg_;
  int tcp_;
  S &status_;
  H &handler_;
  struct repeating_timer timer_;
};

} // namespace Rp2040

#endif // CH9120ETH_HPP___
