#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/sync.h>
#include <hardware/flash.h>
#include <hardware/watchdog.h>
#include "Eeprom.hpp"

namespace Rp2040 {

//------------------------------------------------------------------------------
static void __not_in_flash_func(flash_write_inner)(const void *ptr, std::size_t len, uint32_t offset)
{
  flash_range_erase(offset, FLASH_SECTOR_SIZE);
  len = ((len + 0xff) & 0xff00);
  flash_range_program(offset, static_cast<const uint8_t*>(ptr), len);

  // defined commit to write then reboot, simpler

  watchdog_enable(1, false);
  sleep_ms(100);
}

//------------------------------------------------------------------------------
void eeprom_flash_write(const void *ptr, std::size_t len, uint32_t offset)
{
  multicore_reset_core1();
  (void)save_and_disable_interrupts();

  flash_write_inner(ptr, len, offset);
}

} // namespace Rp2040
