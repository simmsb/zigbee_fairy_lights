#include <cinttypes>
#include <cmath>
#include <optional>

#include <driver/ledc.h>
#include <esp_log.h>

#include "ledc.h"

#define CLOCK_FREQUENCY 80e6f
#define DEFAULT_CLK LEDC_AUTO_CLK

static const uint8_t SETUP_ATTEMPT_COUNT_MAX = 5;

namespace ledc {

static const char *const TAG = "ledc.output";

static const int MAX_RES_BITS = LEDC_TIMER_BIT_MAX - 1;
inline ledc_mode_t get_speed_mode(uint8_t) { return LEDC_LOW_SPEED_MODE; }

float ledc_max_frequency_for_bit_depth(uint8_t bit_depth) {
  return CLOCK_FREQUENCY / float(1 << bit_depth);
}

float ledc_min_frequency_for_bit_depth(uint8_t bit_depth, bool low_frequency) {
  const float max_div_num =
      ((1 << MAX_RES_BITS) - 1) / (low_frequency ? 32.0f : 256.0f);
  return CLOCK_FREQUENCY / (max_div_num * float(1 << bit_depth));
}

std::optional<uint8_t> ledc_bit_depth_for_frequency(float frequency) {
  ESP_LOGV(TAG, "Calculating resolution bit-depth for frequency %f", frequency);
  for (int i = MAX_RES_BITS; i >= 1; i--) {
    const float min_frequency =
        ledc_min_frequency_for_bit_depth(i, (frequency < 100));
    const float max_frequency = ledc_max_frequency_for_bit_depth(i);
    if (min_frequency <= frequency && frequency <= max_frequency) {
      ESP_LOGV(TAG, "Resolution calculated as %d", i);
      return i;
    }
  }
  return {};
}

esp_err_t configure_timer_frequency(ledc_mode_t speed_mode,
                                    ledc_timer_t timer_num,
                                    ledc_channel_t chan_num, uint8_t channel,
                                    uint8_t &bit_depth, float frequency) {
  bit_depth = *ledc_bit_depth_for_frequency(frequency);
  if (bit_depth < 1) {
    ESP_LOGE(TAG, "Frequency %f can't be achieved with any bit depth",
             frequency);
  }

  ledc_timer_config_t timer_conf{};
  timer_conf.speed_mode = speed_mode;
  timer_conf.duty_resolution = static_cast<ledc_timer_bit_t>(bit_depth);
  timer_conf.timer_num = timer_num;
  timer_conf.freq_hz = (uint32_t)frequency;
  timer_conf.clk_cfg = DEFAULT_CLK;

  // Configure the time with fallback in case of error
  int attempt_count_max = SETUP_ATTEMPT_COUNT_MAX;
  esp_err_t init_result = ESP_FAIL;
  while (attempt_count_max > 0 && init_result != ESP_OK) {
    init_result = ledc_timer_config(&timer_conf);
    if (init_result != ESP_OK) {
      ESP_LOGW(
          TAG,
          "Unable to initialize timer with frequency %.1f and bit depth of %u",
          frequency, bit_depth);
      // try again with a lower bit depth
      timer_conf.duty_resolution = static_cast<ledc_timer_bit_t>(--bit_depth);
    }
    attempt_count_max--;
  }

  return init_result;
}

constexpr int ledc_angle_to_htop(float angle, uint8_t bit_depth) {
  return static_cast<int>(angle * ((1U << bit_depth) - 1) / 360.);
}

void LEDCOutput::write_state(float state) {
  if (!initialized_) {
    ESP_LOGW(TAG, "LEDC output hasn't been initialized yet!");
    return;
  }

  if (this->pin_->is_inverted())
    state = 1.0f - state;

  this->duty_ = state;
  const uint32_t max_duty = (uint32_t(1) << this->bit_depth_) - 1;
  const float duty_rounded = roundf(state * max_duty);
  auto duty = static_cast<uint32_t>(duty_rounded);
  ESP_LOGV(TAG, "Setting duty: %" PRIu32 " on channel %u", duty,
           this->channel_);
  auto speed_mode = get_speed_mode(channel_);
  auto chan_num = static_cast<ledc_channel_t>(channel_ % 8);
  int hpoint = ledc_angle_to_htop(this->phase_angle_, this->bit_depth_);
  if (duty == max_duty) {
    ledc_stop(speed_mode, chan_num, 1);
  } else if (duty == 0) {
    ledc_stop(speed_mode, chan_num, 0);
  } else {
    ledc_set_duty_with_hpoint(speed_mode, chan_num, duty, hpoint);
    ledc_update_duty(speed_mode, chan_num);
  }
}

void LEDCOutput::setup() {
  ESP_LOGV(TAG, "Entering setup...");
  auto speed_mode = get_speed_mode(channel_);
  auto timer_num = static_cast<ledc_timer_t>((channel_ % 8) / 2);
  auto chan_num = static_cast<ledc_channel_t>(channel_ % 8);

  esp_err_t timer_init_result =
      configure_timer_frequency(speed_mode, timer_num, chan_num, this->channel_,
                                this->bit_depth_, this->frequency_);

  if (timer_init_result != ESP_OK) {
    ESP_LOGE(TAG, "Frequency %f can't be achieved with computed bit depth %u",
             this->frequency_, this->bit_depth_);
    return;
  }
  int hpoint = ledc_angle_to_htop(this->phase_angle_, this->bit_depth_);

  ESP_LOGV(TAG, "Configured frequency %f with a bit depth of %u bits",
           this->frequency_, this->bit_depth_);
  ESP_LOGV(TAG, "Angle of %.1f° results in hpoint %u", this->phase_angle_,
           hpoint);

  ledc_channel_config_t chan_conf{};
  chan_conf.gpio_num = pin_->get_pin();
  chan_conf.speed_mode = speed_mode;
  chan_conf.channel = chan_num;
  chan_conf.intr_type = LEDC_INTR_DISABLE;
  chan_conf.timer_sel = timer_num;
  chan_conf.duty = inverted_ == pin_->is_inverted() ? 0 : (1U << bit_depth_);
  chan_conf.hpoint = hpoint;
  ledc_channel_config(&chan_conf);
  initialized_ = true;
}

void LEDCOutput::dump_config() {
  ESP_LOGI(TAG, "LEDC Output:");
  LOG_PIN("  Pin ", this->pin_);
  ESP_LOGI(TAG, "  LEDC Channel: %u", this->channel_);
  ESP_LOGI(TAG, "  PWM Frequency: %.1f Hz", this->frequency_);
  ESP_LOGI(TAG, "  Phase angle: %.1f°", this->phase_angle_);
  ESP_LOGI(TAG, "  Bit depth: %u", this->bit_depth_);
  ESP_LOGV(TAG, "  Max frequency for bit depth: %f",
           ledc_max_frequency_for_bit_depth(this->bit_depth_));
  ESP_LOGV(TAG, "  Min frequency for bit depth: %f",
           ledc_min_frequency_for_bit_depth(this->bit_depth_,
                                            (this->frequency_ < 100)));
  ESP_LOGV(TAG, "  Max frequency for bit depth-1: %f",
           ledc_max_frequency_for_bit_depth(this->bit_depth_ - 1));
  ESP_LOGV(TAG, "  Min frequency for bit depth-1: %f",
           ledc_min_frequency_for_bit_depth(this->bit_depth_ - 1,
                                            (this->frequency_ < 100)));
  ESP_LOGV(TAG, "  Max frequency for bit depth+1: %f",
           ledc_max_frequency_for_bit_depth(this->bit_depth_ + 1));
  ESP_LOGV(TAG, "  Min frequency for bit depth+1: %f",
           ledc_min_frequency_for_bit_depth(this->bit_depth_ + 1,
                                            (this->frequency_ < 100)));
  ESP_LOGV(TAG, "  Max res bits: %d", MAX_RES_BITS);
  ESP_LOGV(TAG, "  Clock frequency: %f", CLOCK_FREQUENCY);
}

void LEDCOutput::update_frequency(float frequency) {
  auto bit_depth_opt = ledc_bit_depth_for_frequency(frequency);
  if (!bit_depth_opt.has_value()) {
    ESP_LOGE(TAG, "Frequency %f can't be achieved with any bit depth",
             this->frequency_);
  }
  this->bit_depth_ = bit_depth_opt.value_or(8);
  this->frequency_ = frequency;
  if (!initialized_) {
    ESP_LOGW(TAG, "LEDC output hasn't been initialized yet!");
    return;
  }

  auto speed_mode = get_speed_mode(channel_);
  auto timer_num = static_cast<ledc_timer_t>((channel_ % 8) / 2);
  auto chan_num = static_cast<ledc_channel_t>(channel_ % 8);

  esp_err_t timer_init_result =
      configure_timer_frequency(speed_mode, timer_num, chan_num, this->channel_,
                                this->bit_depth_, this->frequency_);

  if (timer_init_result != ESP_OK) {
    ESP_LOGE(TAG, "Frequency %f can't be achieved with computed bit depth %u",
             this->frequency_, this->bit_depth_);
    return;
  }

  // re-apply duty
  this->write_state(this->duty_);
}

uint8_t next_ledc_channel =
    0; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

} // namespace ledc
