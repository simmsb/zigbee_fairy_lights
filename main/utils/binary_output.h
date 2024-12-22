#pragma once

namespace output {

#define LOG_BINARY_OUTPUT(this)                                                \
  if (this->inverted_) {                                                       \
    ESP_LOGI(TAG, "  Inverted: YES");                                          \
  }

class BinaryOutput {
public:
  /// Set the inversion state of this binary output.
  void set_inverted(bool inverted) { this->inverted_ = inverted; }

  /// Enable or disable this binary output.
  virtual void set_state(bool state) {
    if (state) {
      this->turn_on();
    } else {
      this->turn_off();
    }
  }

  /// Enable this binary output.
  virtual void turn_on() { this->write_state(!this->inverted_); }

  /// Disable this binary output.
  virtual void turn_off() { this->write_state(this->inverted_); }

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  /// Return whether this binary output is inverted.
  bool is_inverted() const { return this->inverted_; }

protected:
  virtual void write_state(bool state) = 0;

  bool inverted_{false};
};

} // namespace output
