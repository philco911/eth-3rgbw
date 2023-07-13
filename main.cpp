#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <tuple>

#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/clocks.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>
#include "Status.hpp"
#include "Regs.hpp"
#include "PwmBank.hpp"
#include "PwmDriver.hpp"
#include "Modbus.hpp"
#include "Ch9120Eth.hpp"
#include "Eeprom.hpp"
#include "Gamma.hpp"

constexpr uint16_t kVersion = 0x0100;
constexpr int kWs2812Gpio = 25;
constexpr int kResetGpio = 27;
constexpr int kEthResetGpio = 19;
constexpr int kEthCfgGpio = 18;
constexpr int kEthTcpGpio = 17;
constexpr int kEthRxGpio = 21;
constexpr int kEthTxGpio = 20;
constexpr std::size_t kInputRegSize = 10;
constexpr std::size_t kPwmSize = 12;
constexpr std::size_t kRegSize = (kPwmSize * 2) + 6 /* ip/nm/gw */ + 1 + /* flags */ + 2 /* serial num */ + 1 /* commit */;
constexpr std::size_t kIpIndex = 24;
constexpr std::size_t kNmIndex = 26;
constexpr std::size_t kGwIndex = 28;
constexpr std::size_t kFlagsIndex = 30;
constexpr std::size_t kCommitIndex = 31;
constexpr std::size_t kSerialNumHiIndex = 32;
constexpr std::size_t kSerialNumLoIndex = 33;
constexpr std::size_t kEepromSize = kRegSize;
constexpr uint16_t kCommitValue = 0xaa55;
constexpr uint16_t kMagicValue = 0xbeef;
constexpr int kRate = 30; // hz
constexpr uint16_t kDefaultRamp = 0xffff / kRate;
constexpr int kHoldTime = 100; // ms
constexpr int kHoldCount = (1000 / kHoldTime) * 5; // seconds
constexpr uint16_t kFlagStacking = 0x0001; // for command queueing/double buffering
constexpr uint16_t kDefaultFlags = kFlagStacking;
constexpr float kGamma = 2.8;

//------------------------------------------------------------------------------
using InputRegs = LedDriver::Regs<kInputRegSize>;
using HoldingRegs = LedDriver::Regs<kRegSize>;
using EepromDrv = Rp2040::Eeprom<kEepromSize>;
using ModbusDrv = LedDriver::Modbus<kInputRegSize, kRegSize>;
using Pwm = Rp2040::PwmBank<kPwmSize>;
using PwmDrv = LedDriver::PwmDriver<kPwmSize>;
using EthDrv = Rp2040::Ch9120Eth<LedDriver::Status, ModbusDrv>;

//------------------------------------------------------------------------------
LedDriver::Status status(kWs2812Gpio);
InputRegs input_regs;
HoldingRegs holding_regs;
EepromDrv fake_eeprom;
ModbusDrv modbus(input_regs, holding_regs);
Pwm pwmbank{22, 26, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
Rp2040::Gamma pwmgamma;
PwmDrv pwmdriver(pwmbank, holding_regs.view<0, kPwmSize * 2>());
EthDrv eth(uart1, kEthResetGpio, kEthCfgGpio, kEthTcpGpio, status, modbus);

//------------------------------------------------------------------------------
static void reboot()
{
  watchdog_enable(5, true);
  sleep_ms(100);
}

//------------------------------------------------------------------------------
static void commit()
{
  // update eth eeprom first
  uint32_t ip = (static_cast<uint32_t>(holding_regs.value(kIpIndex)) << 16) | holding_regs.value(kIpIndex + 1);
  uint32_t nm = (static_cast<uint32_t>(holding_regs.value(kNmIndex)) << 16) | holding_regs.value(kNmIndex + 1);
  uint32_t gw = (static_cast<uint32_t>(holding_regs.value(kGwIndex)) << 16) | holding_regs.value(kGwIndex + 1);
  eth.config(ip, nm, gw);

  // then ours
  for (std::size_t idx = 0; idx < kRegSize; idx++) {
    fake_eeprom.value(idx, holding_regs.value(idx));
  }
  fake_eeprom.commit(); // this won't return
}

//------------------------------------------------------------------------------
static void core_start()
{
  eth.start();

  while (true) {
    tight_loop_contents();
  }
}

//------------------------------------------------------------------------------
int main()
{
  //set_sys_clock_48();
  stdio_init_all();

  gpio_init(kResetGpio);
  gpio_set_dir(kResetGpio, false);
  gpio_pull_up(kResetGpio);

  // model number written out and the version
  input_regs.value(0, 0x4554);
  input_regs.value(1, 0x4833);
  input_regs.value(2, 0x5247);
  input_regs.value(3, 0x5733);
  input_regs.value(4, 0x3031);
  input_regs.value(5, 0x3232);
  input_regs.value(6, 0x3400);
  input_regs.value(7, kVersion);

  status.start();

  // setup with default values to start
  for (std::size_t idx = 0; idx < kPwmSize; idx++) {
    holding_regs.value((idx * 2) + 1, kDefaultRamp);
  }
  holding_regs.value(kFlagsIndex, kDefaultFlags);
  holding_regs.value(kCommitIndex, kCommitValue); // start with saving eeprom if not valid

  // check for factory reset by reset button being held down on boot for a period
  int count = 0;
  while ((count < kHoldCount) && (gpio_get(kResetGpio) == 0)) {
    sleep_ms(kHoldTime);
    count++;
  }
  if (count == kHoldCount) {
    status.reset(true);
  } else if (fake_eeprom.is_valid()) {
    // otherwise restore values from eeprom with they are valid
    for (std::size_t idx = 0; idx < kRegSize; idx++) {
      holding_regs.value(idx, fake_eeprom.value(idx));
    }
  }

  input_regs.value(8, holding_regs.value(kSerialNumHiIndex));
  input_regs.value(9, holding_regs.value(kSerialNumLoIndex));

  // get pwm going
  pwmgamma.generate(kGamma);
  pwmbank.gamma(pwmgamma.table());
  pwmdriver.start(kRate, holding_regs.value(kFlagsIndex) & kFlagStacking);

  // setup ethernet
  gpio_init(kEthResetGpio);
  gpio_set_dir(kEthResetGpio, true);
  gpio_init(kEthCfgGpio);
  gpio_set_dir(kEthCfgGpio, true);
  gpio_init(kEthTcpGpio);
  gpio_set_dir(kEthTcpGpio, false);
  gpio_set_function(kEthRxGpio, GPIO_FUNC_UART);
  gpio_set_function(kEthTxGpio, GPIO_FUNC_UART);

  multicore_launch_core1(core_start);
  sleep_ms(1000); // give time for the eth/serial port to finish starting
                  // in case we need to stop it to commit eeprom/flash

  while (true) {
    tight_loop_contents();

    auto com = holding_regs.value(kCommitIndex);
    if ((com == kCommitValue) || (com == kMagicValue)) {
      status.reset(true);
      sleep_ms(1000);
      if (com != kMagicValue) { // special value required for setting serial number
        holding_regs.value(kSerialNumHiIndex, input_regs.value(8)); // restore
        holding_regs.value(kSerialNumLoIndex, input_regs.value(9)); // restore
      }
      holding_regs.value(kCommitIndex, 0xffff); // clear it
      commit(); // save
    } else if (gpio_get(kResetGpio) == 0) {
      reboot();
    }
  }
}
