#ifndef REGS_HPP__
#define REGS_HPP__

#include <array>
#include <cstdint>

namespace LedDriver {

//------------------------------------------------------------------------------
template<std::size_t Size>
class RegsView
{
public:
  RegsView(uint32_t *data) : data_(data) {
  }

  uint16_t value(std::size_t reg) const {
    if (reg < Size) {
      return data_[reg] & 0xffff;
    } else {
      return 0;
    }
  }

private:
  uint32_t *data_;
};

//------------------------------------------------------------------------------
template<std::size_t Size>
class Regs
{
public:
  Regs() {
    values_.fill(0);
  }

  void fill(std::size_t start, std::size_t count, uint16_t val) {
    for (std::size_t idx = 0; idx < count; idx++) {
      values_[start + idx] = val;
    }
  }

  void value(std::size_t reg, uint16_t val) {
    if (reg < values_.size()) {
      values_[reg] = val;
    }
  }

  uint16_t value(std::size_t reg) const {
    if (reg < values_.size()) {
      return values_[reg] & 0xffff;
    } else {
      return 0;
    }
  }

  template<std::size_t Offset, std::size_t Count>
  constexpr RegsView<Count> view() const {
    return RegsView<Count>(const_cast<uint32_t*>(&values_[Offset]));
  }

private:
  std::array<uint32_t, Size> values_;
};

} // namespace LedDriver

#endif // REGS_HPP__
