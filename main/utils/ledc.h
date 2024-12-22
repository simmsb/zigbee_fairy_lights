#include "float_output.h"
#include "gpio.h"
#include <cinttypes>

#pragma once

namespace ledc {

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern uint8_t next_ledc_channel;

class LEDCOutput : public output::FloatOutput {
public:
  explicit LEDCOutput(InternalGPIOPin *pin) : pin_(pin) {
    this->channel_ = next_ledc_channel++;
  }

  void set_channel(uint8_t channel) { this->channel_ = channel; }
  void set_frequency(float frequency) { this->frequency_ = frequency; }
  void set_phase_angle(float angle) { this->phase_angle_ = angle; }
  /// Dynamically change frequency at runtime
  void update_frequency(float frequency) override;

  /// Setup LEDC.
  void setup();
  void dump_config();

  /// Override FloatOutput's write_state.
  void write_state(float state) override;

protected:
  InternalGPIOPin *pin_;
  uint8_t channel_{};
  uint8_t bit_depth_{};
  float phase_angle_{0.0f};
  float frequency_{};
  float duty_{0.0f};
  bool initialized_ = false;
};

} // namespace ledc
