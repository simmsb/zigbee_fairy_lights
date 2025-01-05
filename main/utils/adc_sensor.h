#pragma once

#include "driver/adc_types_legacy.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h" // This defines ADC_CHANNEL_MAX
#include "isr_gpio.h"

#define USE_ESP32

namespace adc {

// Map old channel names to new ones for compatibility, if needed
#ifndef ADC1_CHANNEL_0
#define ADC1_CHANNEL_0 ADC_CHANNEL_0
#define ADC1_CHANNEL_1 ADC_CHANNEL_1
#define ADC1_CHANNEL_2 ADC_CHANNEL_2
#define ADC1_CHANNEL_3 ADC_CHANNEL_3
#define ADC1_CHANNEL_4 ADC_CHANNEL_4
#define ADC1_CHANNEL_5 ADC_CHANNEL_5
#define ADC1_CHANNEL_6 ADC_CHANNEL_6
#define ADC1_CHANNEL_7 ADC_CHANNEL_7
#endif // ADC1_CHANNEL_0

#ifndef ADC1_CHANNEL_MAX
#ifdef ADC_CHANNEL_MAX
#define ADC1_CHANNEL_MAX ADC_CHANNEL_MAX
#else
constexpr adc_channel_t ADC1_CHANNEL_MAX = static_cast<adc_channel_t>(8);
#endif // ADC_CHANNEL_MAX
#endif // ADC1_CHANNEL_MAX

#ifndef ADC2_CHANNEL_0
#define ADC2_CHANNEL_0 ADC_CHANNEL_0
#define ADC2_CHANNEL_1 ADC_CHANNEL_1
#define ADC2_CHANNEL_2 ADC_CHANNEL_2
#define ADC2_CHANNEL_3 ADC_CHANNEL_3
#define ADC2_CHANNEL_4 ADC_CHANNEL_4
#define ADC2_CHANNEL_5 ADC_CHANNEL_5
#define ADC2_CHANNEL_6 ADC_CHANNEL_6
#define ADC2_CHANNEL_7 ADC_CHANNEL_7
#define ADC2_CHANNEL_8 ADC_CHANNEL_8
#define ADC2_CHANNEL_9 ADC_CHANNEL_9
#endif // ADC2_CHANNEL_0

#ifndef ADC2_CHANNEL_MAX
#ifdef ADC_CHANNEL_MAX
#define ADC2_CHANNEL_MAX ADC_CHANNEL_MAX
#else
constexpr adc_channel_t ADC2_CHANNEL_MAX = static_cast<adc_channel_t>(10);
#endif // ADC_CHANNEL_MAX
#endif // ADC2_CHANNEL_MAX

static const adc_atten_t ADC_ATTEN_DB_12_COMPAT = ADC_ATTEN_DB_12;

class ADCSensor {
public:
  /// Update the sensor's state by reading the current ADC value.
  /// This method is called periodically based on the update interval.
  void update();

  /// Set up the ADC sensor by initializing hardware and calibration parameters.
  /// This method is called once during device initialization.
  void setup();

  /// Set the GPIO pin to be used by the ADC sensor.
  /// @param pin Pointer to an InternalGPIOPin representing the ADC input pin.
  void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }

  /// Enable or disable the output of raw ADC values (unprocessed data).
  /// @param output_raw Boolean indicating whether to output raw ADC values
  /// (true) or processed values (false).
  void set_output_raw(bool output_raw) { this->output_raw_ = output_raw; }

  /// Set the number of samples to be taken for ADC readings to improve
  /// accuracy. A higher sample count reduces noise but increases the reading
  /// time.
  /// @param sample_count The number of samples (e.g., 1, 4, 8).
  void set_sample_count(uint8_t sample_count);

  /// Perform a single ADC sampling operation and return the measured value.
  /// This function handles raw readings, calibration, and averaging as needed.
  /// @return The sampled value as a float.
  float sample();

  ~ADCSensor();

#ifdef USE_ESP32
#if true
  /// Set the ADC attenuation level to adjust the input voltage range.
  /// This determines how the ADC interprets input voltages, allowing for
  /// greater precision or the ability to measure higher voltages depending on
  /// the chosen attenuation level.
  /// @param attenuation The desired ADC attenuation level (e.g.,
  /// ADC_ATTEN_DB_0, ADC_ATTEN_DB_11).
  void set_attenuation(adc_atten_t attenuation) {
    this->attenuation_ = attenuation;
    this->do_setup_ = true;
  }

  /// Configure the ADC to use a specific channel on ADC1.
  /// This sets the channel for single-shot or continuous ADC measurements.
  /// @param channel The ADC1 channel to configure, such as ADC_CHANNEL_0,
  /// ADC_CHANNEL_3, etc.
  void set_channel1(adc_channel_t channel) {
    this->channel_ = channel;
    this->is_adc1_ = true;
    this->do_setup_ = true;
  }

  /// Configure the ADC to use a specific channel on ADC2.
  /// This sets the channel for single-shot or continuous ADC measurements.
  /// ADC2 is shared with other peripherals, so care must be taken to avoid
  /// conflicts.
  /// @param channel The ADC2 channel to configure, such as ADC_CHANNEL_0,
  /// ADC_CHANNEL_3, etc.
  void set_channel2(adc_channel_t channel) {
    this->channel_ = channel;
    this->is_adc1_ = false;
    this->do_setup_ = true;
  }
#else
  /// Set the ADC attenuation level to adjust the input voltage range.
  /// This determines how the ADC interprets input voltages, allowing for
  /// greater precision or the ability to measure higher voltages depending on
  /// the chosen attenuation level.
  /// @param attenuation The desired ADC attenuation level (e.g.,
  /// ADC_ATTEN_DB_0, ADC_ATTEN_DB_11).
  void set_attenuation(adc_atten_t attenuation) {
    this->attenuation_ = attenuation;
  }

  /// Configure the ADC to use a specific channel on ADC1.
  /// This sets the ADC1 channel for measurement and disables ADC2 channel
  /// usage.
  /// @param channel The ADC1 channel to configure (e.g., ADC1_CHANNEL_0,
  /// ADC1_CHANNEL_3).
  void set_channel1(adc1_channel_t channel) {
    this->channel1_ = channel;
    this->channel2_ = ADC2_CHANNEL_MAX;
  }

  /// Configure the ADC to use a specific channel on ADC2.
  /// This sets the ADC2 channel for measurement and disables ADC1 channel
  /// usage. Note: ADC2 may have limitations due to shared resources with Wi-Fi
  /// or other peripherals.
  /// @param channel The ADC2 channel to configure (e.g., ADC2_CHANNEL_0,
  /// ADC2_CHANNEL_3).
  void set_channel2(adc2_channel_t channel) {
    this->channel2_ = channel;
    this->channel1_ = ADC1_CHANNEL_MAX;
  }
#endif // ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  /// Set whether autoranging should be enabled for the ADC.
  /// Autoranging automatically adjusts the attenuation level to handle a wide
  /// range of input voltages.
  /// @param autorange Boolean indicating whether to enable autoranging.
  void set_autorange(bool autorange) { this->autorange_ = autorange; }
#endif // USE_ESP32

#ifdef USE_ESP8266
  std::string unique_id() override;
#endif // USE_ESP8266

#ifdef USE_RP2040
  void set_is_temperature() { this->is_temperature_ = true; }
#endif // USE_RP2040

protected:
  InternalGPIOPin *pin_;
  bool output_raw_{false};
  uint8_t sample_count_{1};

#ifdef USE_ESP32
  bool autorange_{false};
#if true
  adc_oneshot_unit_handle_t adc1_handle_{nullptr};
  adc_oneshot_unit_handle_t adc2_handle_{nullptr};
  adc_cali_handle_t calibration_handle_{nullptr};
  adc_atten_t attenuation_{ADC_ATTEN_DB_0};
  adc_channel_t channel_;
  bool is_adc1_{true};
  bool do_setup_{false};
  bool init_complete_{false};
  bool config_complete_{false};
  bool handle_init_complete_{false};
  bool calibration_complete_{false};
  static adc_oneshot_unit_handle_t shared_adc1_handle;
  static adc_oneshot_unit_handle_t shared_adc2_handle;
#else
  adc_atten_t attenuation_{ADC_ATTEN_DB_0};
  adc1_channel_t channel1_{ADC1_CHANNEL_MAX};
  adc2_channel_t channel2_{ADC2_CHANNEL_MAX};
#if true
  esp_adc_cal_characteristics_t cal_characteristics_[SOC_ADC_ATTEN_NUM] = {};
#else
  esp_adc_cal_characteristics_t cal_characteristics_[ADC_ATTEN_MAX] = {};
#endif // ESP_IDF_VERSION_MAJOR >= 5
#endif // ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#endif // USE_ESP32

#ifdef USE_RP2040
  bool is_temperature_{false};
#endif // USE_RP2040
};

} // namespace adc
