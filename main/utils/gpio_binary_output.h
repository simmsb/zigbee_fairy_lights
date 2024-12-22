#pragma once

#include "binary_output.h"
#include "gpio.h"

namespace gpio {

class GPIOBinaryOutput : public output::BinaryOutput {
public:
  void set_pin(GPIOPin *pin) { pin_ = pin; }

  void setup() {
    this->turn_off();
    this->pin_->setup();
    this->turn_off();
  }

protected:
  void write_state(bool state) override { this->pin_->digital_write(state); }

  GPIOPin *pin_;
};

} // namespace gpio
