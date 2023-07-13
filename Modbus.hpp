#ifndef MODBUS_HPP__
#define MODBUS_HPP__

#include <pico/stdlib.h>
#include <pico/time.h>
#include "Regs.hpp"

namespace {

const uint16_t kPort = 502;

} // namespace

namespace LedDriver {

//------------------------------------------------------------------------------
template<std::size_t N, std::size_t H>
class Modbus
{
public:
  Modbus(Regs<N> &input_regs, Regs<H> &holding_regs) :
      input_regs_(input_regs),
      holding_regs_(holding_regs) {
  }

  void data(uint8_t byte) {
    // A form of inter-message gap. If the gap between bytes exceeds timeout then reset message parsing state.
    if (get_absolute_time() > timeout_) {
      rx_state_ = State::kTidHigh;
    }

    switch (rx_state_) {
    case State::kTidHigh:
      rx_.tid = byte << 8;
      rx_state_ = State::kTidLow;
      break;
    case State::kTidLow:
      rx_.tid |= byte;
      rx_state_ = State::kPidHigh;
      break;
    case State::kPidHigh:
      rx_.pid = byte << 8;
      rx_state_ = State::kPidLow;
      break;
    case State::kPidLow:
      rx_.pid |= byte;
      rx_state_ = State::kLenHigh;
      break;
    case State::kLenHigh:
      rx_.len = byte << 8;
      rx_state_ = State::kLenLow;
      break;
    case State::kLenLow:
      rx_.len |= byte;
      if (rx_.len < 2) {
        rx_state_ = State::kTidHigh;
      } else {
        rx_state_ = State::kUid;
      }
      break;
    case State::kUid:
      rx_.uid = byte;
      rx_.len--;
      if (rx_.len == 0) {
        rx_state_ = State::kTidHigh;
      } else {
        rx_state_ = State::kFCode;
      }
      break;
    case State::kFCode:
      rx_.fcode = byte;
      rx_.len--;
      rx_.off = 0;
      switch (rx_.fcode) {
      case kReadInput:
      case kReadHolding:
        if (rx_.len == 0) {
          tx_exp_ = kIllegalValue;
          exception();
        } else if (rx_.len != 4) {
          rx_state_ = State::kDrain;
        } else {
          rx_state_ = State::kAddrHigh;
        }
        break;
      case kWriteSingleHolding:
        if (rx_.len == 0) {
          tx_exp_ = kIllegalValue;
          exception();
        } else if (rx_.len != 4) {
          rx_state_ = State::kDrain;
        } else {
          rx_.count = 1;
          rx_state_ = State::kAddrHigh;
        }
        break;
      case kWriteHolding:
        if (rx_.len == 0) {
          tx_exp_ = kIllegalValue;
          exception();
        } else if (rx_.len < 4) {
          rx_state_ = State::kDrain;
        } else {
          rx_state_ = State::kAddrHigh;
        }
        break;
      default:
       tx_exp_ = kIllegalFunction;
        if (rx_.len == 0) {
          exception();
        } else {
          rx_state_ = State::kDrain;
        }
        break;
      }
      break;
    case State::kAddrHigh:
      rx_.len--;
      rx_.addr = byte << 8;
      rx_state_ = State::kAddrLow;
      break;
    case State::kAddrLow:
      rx_.len--;
      rx_.addr |= byte;
      if (rx_.fcode == kWriteSingleHolding) {
        rx_state_ = State::kValueHigh;
      } else {
        rx_state_ = State::kCountHigh;
      }
      break;
    case State::kCountHigh:
      rx_.len--;
      rx_.count = byte << 8;
      rx_state_ = State::kCountLow;
      break;
    case State::kCountLow:
      rx_.len--;
      rx_.count |= byte;
      if ((rx_.fcode == kReadHolding) || (rx_.fcode == kReadInput)) {
        respond((rx_.count << 1) + 3);
      } else {
        rx_state_ = State::kByteCount;
      }
      break;
    case State::kByteCount:
      rx_.len--;
      rx_state_ = State::kValueHigh;
      break;
    case State::kValueHigh:
      rx_.len--;
      rx_.value = byte << 8;
      rx_state_ = State::kValueLow;
      break;
    case State::kValueLow:
      rx_.len--;
      rx_.value |= byte;
      holding_regs_.value(rx_.addr + rx_.off, rx_.value);
      rx_.off++;
      if (rx_.fcode == kWriteSingleHolding) {
        tx_.value = rx_.value;
        respond(6);
      } else if (rx_.len == 0) {
        respond(6);
      } else {
        rx_state_ = State::kValueHigh;
      }
      break;
    case State::kDrain:
      rx_.len--;
      if (rx_.len == 0) {
        exception();
      }
      break;
    }
    timeout_ = make_timeout_time_ms(kTimeout);
  }

  uint16_t port() const {
    return kPort;
  }

  bool has_data() const {
    return tx_state_ != State::kIdle;
  }

  uint8_t data() {
    uint8_t v;

    switch (tx_state_) {
    case State::kTidHigh:
      tx_state_ = State::kTidLow;
      return tx_.tid >> 8;
    case State::kTidLow:
      tx_state_ = State::kPidHigh;
      return tx_.tid & 0xff;
    case State::kPidHigh:
      tx_state_ = State::kPidLow;
      return tx_.pid >> 8;
    case State::kPidLow:
      tx_state_ = State::kLenHigh;
      return tx_.pid & 0xff;
    case State::kLenHigh:
      tx_state_ = State::kLenLow;
      return tx_.len >> 8;
    case State::kLenLow:
      tx_state_ = State::kUid;
      return tx_.len & 0xff;
    case State::kUid:
      tx_state_ = State::kFCode;
      return tx_.uid;
    case State::kFCode:
      if (tx_.fcode & 0x80) {
        tx_state_ = State::kExp;
      } else if ((tx_.fcode == kReadHolding) || (tx_.fcode == kReadInput)) {
        tx_state_ = State::kByteCount;
      } else {
        tx_state_ = State::kAddrHigh;
      }
      return tx_.fcode;
    case State::kExp:
      tx_state_ = State::kIdle;
      return tx_exp_;
    case State::kAddrHigh:
      tx_state_ = State::kAddrLow;
      return tx_.addr >> 8;
    case State::kAddrLow:
      if (tx_.fcode == kWriteSingleHolding) {
        tx_state_ = State::kValueHigh;
      } else {
        tx_state_ = State::kCountHigh;
      }
      return tx_.addr & 0xff;
    case State::kByteCount:
      tx_state_ = State::kValueHigh;
      if (tx_.fcode == kReadHolding) {
        tx_.value = holding_regs_.value(tx_.addr);
      } else {
        tx_.value = input_regs_.value(tx_.addr);
      }
      tx_.off = 1;
      return tx_.count << 1;
    case State::kValueHigh:
      tx_state_ = State::kValueLow;
      return tx_.value >> 8;
    case State::kValueLow:
      v = tx_.value & 0xff;
      tx_.count--;
      if (tx_.count != 0) {
        tx_state_ = State::kValueHigh;
        if (tx_.fcode == kReadHolding) {
          tx_.value = holding_regs_.value(tx_.addr + tx_.off);
        } else {
          tx_.value = input_regs_.value(tx_.addr + tx_.off);
        }
        tx_.off++;
      } else {
        tx_state_ = State::kIdle;
      }
      return v;
    case State::kCountHigh:
      tx_state_ = State::kCountLow;
      return tx_.count >> 8;
    case State::kCountLow:
      tx_state_ = State::kIdle;
      return tx_.count & 0xff;
    }

    return 0;
  }

private:
  static constexpr uint16_t kReadHolding{0x03};
  static constexpr uint16_t kReadInput{0x04};
  static constexpr uint16_t kWriteSingleHolding{0x06};
  static constexpr uint16_t kWriteHolding{0x10};
  static constexpr uint8_t kIllegalFunction{0x01};
  static constexpr uint8_t kIllegalValue{0x03};
  static constexpr uint32_t kTimeout{100};

  enum class State
  {
    kIdle,
    kTidHigh,
    kTidLow,
    kPidHigh,
    kPidLow,
    kLenHigh,
    kLenLow,
    kUid,
    kFCode,
    kAddrHigh,
    kAddrLow,
    kCountHigh,
    kCountLow,
    kValueHigh,
    kValueLow,
    kByteCount,
    kExp,
    kDrain
  };

  struct Action
  {
    uint16_t tid;
    uint16_t pid;
    uint16_t len;
    uint8_t uid;
    uint8_t fcode;
    uint16_t addr;
    uint16_t count;
    uint16_t off;
    uint16_t value;
  };

  void exception() {
    tx_.len = 3;
    tx_.fcode = rx_.fcode | 0x80;
    start_send();
  }

  void respond(uint16_t len) {
    tx_.fcode = rx_.fcode;
    tx_.len = len;
    start_send();
  }

  void start_send() {
    rx_state_ = State::kTidHigh;
    tx_.tid = rx_.tid;
    tx_.pid = rx_.pid;
    tx_.uid = rx_.uid;
    tx_.addr = rx_.addr;
    tx_.count = rx_.count;
    tx_state_ = State::kTidHigh;
  }

  Regs<N> &input_regs_;
  Regs<H> &holding_regs_;
  Action rx_;
  State rx_state_{State::kTidHigh};
  absolute_time_t timeout_;
  Action tx_;
  uint8_t tx_exp_;
  State tx_state_{State::kIdle};
};

} // namespace LedDriver

#endif // MODBUS_HPP__
