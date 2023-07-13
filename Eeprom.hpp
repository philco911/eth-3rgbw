#ifndef EEPROM_HPP__
#define EEPROM_HPP__

#include <pico/stdlib.h>
#include <hardware/flash.h>
#include <array>

namespace Rp2040 {

//------------------------------------------------------------------------------
extern void eeprom_flash_write(const void *ptr, std::size_t len, uint32_t offset);

//------------------------------------------------------------------------------
template<std::size_t N>
class Eeprom
{
public:
  Eeprom() {
    uint16_t *ptr = (uint16_t*)(XIP_BASE + kOffset);
    uint16_t sum = 0;
    for (std::size_t idx = 0; idx < data_.size(); idx++) {
      data_[idx] = ptr[idx];
      if (idx >= kHdrSize) {
        sum += data_[idx];
      }
    }
    sum = 0xffff - sum;

    if ((data_[0] != kMagic) || (data_[1] != sum)) {
      data_.fill(0);
    }
  }

  bool is_valid() const {
    return (data_[0] == kMagic);
  }

  void value(std::size_t reg, uint16_t val) {
    data_[reg + kHdrSize] = val;
  }

  uint16_t value(std::size_t reg) const {
    return data_[reg + kHdrSize];
  }

  void commit() {
    uint16_t sum = 0;
    for (std::size_t idx = kHdrSize; idx < data_.size(); idx++) {
      sum += data_[idx];
    }
    sum = 0xffff - sum;

    data_[0] = kMagic;
    data_[1] = sum;

    eeprom_flash_write(data_.data(), data_.size() * sizeof(uint16_t), kOffset);
  }

private:
  static constexpr uint32_t kOffset = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
  static constexpr uint16_t kMagic = 0x284d;
  static constexpr int kHdrSize = 2;

  std::array<uint16_t, N + kHdrSize> data_;
};

} // namespace Rp2040

#endif // EEPROM_HPP__
