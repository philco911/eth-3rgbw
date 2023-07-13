#include "Status.hpp"

namespace LedDriver {

//------------------------------------------------------------------------------
bool Status::tick_callback(struct repeating_timer *timer)
{
  Status *status = (Status*)(timer->user_data);
  status->tick();
  return true;
}

//------------------------------------------------------------------------------
Status::Status(int pin) :
    ws_(pio0, 0, pin, true)
{
}

//------------------------------------------------------------------------------
void Status::start()
{
  add_repeating_timer_ms(-500, Status::tick_callback, this, &timer_);
}

//------------------------------------------------------------------------------
void Status::connected(bool con)
{
  connected_ = con;
}

//------------------------------------------------------------------------------
void Status::linkup(bool up)
{
  up_ = up;
}

//------------------------------------------------------------------------------
void Status::reset(bool rst)
{
  reset_ = rst;
}

//------------------------------------------------------------------------------
void Status::announce(bool an)
{
  announce_ = an;
}

//------------------------------------------------------------------------------
void Status::tick()
{
  if (reset_) {
    ws_.rgb(0x00, 0xff, 0x00);
  } else if (connected_) {
    ws_.rgb(0x00, 0x00, 0xff);
  } else if (announce_) {
    ws_.rgb(0x00, tick_?0xff:0x00, tick_?0xff:0x00);
  } else if (up_) {
    ws_.rgb(0x00, 0x00, tick_?0xff:0x00);
  } else {
    ws_.rgb(0xff, 0x00, 0x00);
  }
  tick_ = !tick_;
}

} // namespace LedDriver
