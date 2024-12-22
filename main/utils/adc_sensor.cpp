#include "adc_sensor.h"
#include <cmath>
#include <esp_log.h>

#define USE_ESP32_VARIANT_ESP32C6 true

namespace adc {

static const char *const TAG = "adc.esp32";

adc_oneshot_unit_handle_t ADCSensor::shared_adc1_handle = nullptr;
adc_oneshot_unit_handle_t ADCSensor::shared_adc2_handle = nullptr;

void ADCSensor::setup() {
  ESP_LOGI(TAG, "Setting up ADC ");

  if (!this->do_setup_) {
    return;
  }

  this->init_complete_ = false;
  this->handle_init_complete_ = false;
  this->config_complete_ = false;
  this->calibration_complete_ = false;

  if (this->is_adc1_) {
    if (this->adc1_handle_ == nullptr) {
      // Check if another sensor already initialized ADC1
      if (shared_adc1_handle != nullptr) {
        this->adc1_handle_ = shared_adc1_handle;
        this->handle_init_complete_ = true;
      } else {
        adc_oneshot_unit_init_cfg_t init_config1 = {};
        init_config1.unit_id = ADC_UNIT_1;
        init_config1.ulp_mode = ADC_ULP_MODE_DISABLE;
#if USE_ESP32_VARIANT_ESP32C3 || USE_ESP32_VARIANT_ESP32C6 ||                  \
    USE_ESP32_VARIANT_ESP32H2
        init_config1.clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
#endif // USE_ESP32_VARIANT_ESP32C3 || USE_ESP32_VARIANT_ESP32C6 ||
       // USE_ESP32_VARIANT_ESP32H2
        esp_err_t err =
            adc_oneshot_new_unit(&init_config1, &this->adc1_handle_);
        if (err != ESP_OK) {
          ESP_LOGE(TAG, "Error initializing ADC1 unit: %d", err);
          this->handle_init_complete_ = false;
          return;
        }
        shared_adc1_handle = this->adc1_handle_;
        this->handle_init_complete_ = true;
      }
    }

    adc_oneshot_chan_cfg_t config = {
        .atten = this->attenuation_,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t err =
        adc_oneshot_config_channel(this->adc1_handle_, this->channel_, &config);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error configuring ADC1 channel: %d", err);
      this->config_complete_ = false;
      return;
    }
    this->config_complete_ = true;

  } else {
    if (this->adc2_handle_ == nullptr) {
      // Check if another sensor already initialized ADC2
      if (shared_adc2_handle != nullptr) {
        this->adc2_handle_ = shared_adc2_handle;
        this->handle_init_complete_ = true;
      } else {
        adc_oneshot_unit_init_cfg_t init_config2 = {};
        init_config2.unit_id = ADC_UNIT_2;
        init_config2.ulp_mode = ADC_ULP_MODE_DISABLE;
#if USE_ESP32_VARIANT_ESP32C3 || USE_ESP32_VARIANT_ESP32C6 ||                  \
    USE_ESP32_VARIANT_ESP32H2
        init_config2.clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
#endif // USE_ESP32_VARIANT_ESP32C3 || USE_ESP32_VARIANT_ESP32C6 ||
       // USE_ESP32_VARIANT_ESP32H2
        esp_err_t err =
            adc_oneshot_new_unit(&init_config2, &this->adc2_handle_);
        if (err != ESP_OK) {
          ESP_LOGE(TAG, "Error initializing ADC2 unit: %d", err);
          this->handle_init_complete_ = false;
          return;
        }
        shared_adc2_handle = this->adc2_handle_;
        this->handle_init_complete_ = true;
      }
    }

    adc_oneshot_chan_cfg_t config = {
        .atten = this->attenuation_,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t err =
        adc_oneshot_config_channel(this->adc2_handle_, this->channel_, &config);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error configuring ADC2 channel: %d", err);
      this->config_complete_ = false;
      return;
    }
    this->config_complete_ = true;
  }

  // Initialize ADC calibration if not already done
  if (this->calibration_handle_ == nullptr) {
    adc_cali_handle_t handle = nullptr;
    adc_unit_t unit_id = this->is_adc1_ ? ADC_UNIT_1 : ADC_UNIT_2;
    esp_err_t err;

#if USE_ESP32_VARIANT_ESP32C3 || USE_ESP32_VARIANT_ESP32C6 ||                  \
    USE_ESP32_VARIANT_ESP32S3 || USE_ESP32_VARIANT_ESP32H2
    // RISC-V variants and S3 use curve fitting calibration
    adc_cali_curve_fitting_config_t cali_config = {}; // Zero initialize first
#if true
    cali_config.chan = this->channel_;
#endif // ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
    cali_config.unit_id = unit_id;
    cali_config.atten = this->attenuation_;
    cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;

    err = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (err == ESP_OK) {
      this->calibration_handle_ = handle;
      this->calibration_complete_ = true;
      ESP_LOGV(TAG, "Using curve fitting calibration");
    } else {
      ESP_LOGW(TAG,
               "Curve fitting calibration failed with error %d, will use "
               "uncalibrated readings",
               err);
      this->calibration_complete_ = false;
    }
#else // Other ESP32 variants use line fitting calibration
    adc_cali_line_fitting_config_t cali_config = {
      .unit_id = unit_id,
      .atten = this->attenuation_,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
#if !defined(USE_ESP32_VARIANT_ESP32S2)
      .default_vref = 1100, // Default reference voltage in mV
#endif // !defined(USE_ESP32_VARIANT_ESP32S2)
    };
    err = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (err == ESP_OK) {
      this->calibration_handle_ = handle;
      this->calibration_complete_ = true;
      ESP_LOGV(TAG, "Using line fitting calibration");
    } else {
      ESP_LOGW(TAG,
               "Line fitting calibration failed with error %d, will use "
               "uncalibrated readings",
               err);
      this->calibration_complete_ = false;
    }
#endif // USE_ESP32_VARIANT_ESP32C3 || ESP32C6 || ESP32S3 || ESP32H2
  }

  this->init_complete_ = true;
  this->do_setup_ = false;
}

void ADCSensor::set_sample_count(uint8_t sample_count) {
  if (sample_count != 0) {
    this->sample_count_ = sample_count;
  }
}

float ADCSensor::sample() {
  if (!this->autorange_) {
    uint32_t sum = 0;
    uint8_t success_samples = 0;

    for (uint8_t sample = 0; sample < this->sample_count_; sample++) {
      int raw;
      esp_err_t err;

      if (this->is_adc1_) {
        err = adc_oneshot_read(this->adc1_handle_, this->channel_, &raw);
      } else {
        err = adc_oneshot_read(this->adc2_handle_, this->channel_, &raw);
      }

      if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed with error %d", err);
        continue;
      }

      if (raw == -1) {
        ESP_LOGW(TAG, "Invalid ADC reading");
        continue;
      }

      sum += raw;
      success_samples++;
    }

    if (success_samples == 0) {
      ESP_LOGE(TAG, "All ADC readings failed");
      return NAN;
    }

    sum = (sum + (success_samples >> 1)) / success_samples;

    if (this->output_raw_) {
      return sum;
    }

    if (this->calibration_handle_ != nullptr) {
      int voltage_mv;
      esp_err_t err =
          adc_cali_raw_to_voltage(this->calibration_handle_, sum, &voltage_mv);
      if (err == ESP_OK) {
        return voltage_mv / 1000.0f;
      } else {
        ESP_LOGW(TAG,
                 "ADC calibration conversion failed with error %d, disabling "
                 "calibration",
                 err);
        if (this->calibration_handle_ != nullptr) {
#if USE_ESP32_VARIANT_ESP32C3 || USE_ESP32_VARIANT_ESP32C6 ||                  \
    USE_ESP32_VARIANT_ESP32S3 || USE_ESP32_VARIANT_ESP32H2
          adc_cali_delete_scheme_curve_fitting(this->calibration_handle_);
#else  // Other ESP32 variants use line fitting calibration
          adc_cali_delete_scheme_line_fitting(this->calibration_handle_);
#endif // USE_ESP32_VARIANT_ESP32C3 || ESP32C6 || ESP32S3 || ESP32H2
          this->calibration_handle_ = nullptr;
        }
      }
    }

    return sum * 3.3f / 4095.0f;

  } else {
    auto read_atten = [this](adc_atten_t atten) -> std::pair<int, float> {
      if (this->is_adc1_) {
        adc_oneshot_chan_cfg_t config = {
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        esp_err_t err = adc_oneshot_config_channel(this->adc1_handle_,
                                                   this->channel_, &config);
        if (err != ESP_OK) {
          ESP_LOGW(TAG, "Error configuring ADC1 channel for autorange: %d",
                   err);
          return {-1, 0.0f};
        }

        int raw;
        err = adc_oneshot_read(this->adc1_handle_, this->channel_, &raw);
        if (err != ESP_OK) {
          ESP_LOGW(TAG, "ADC1 read failed in autorange with error %d", err);
          return {-1, 0.0f};
        }

        if (this->calibration_handle_ != nullptr) {
          int voltage_mv;
          err = adc_cali_raw_to_voltage(this->calibration_handle_, raw,
                                        &voltage_mv);
          if (err == ESP_OK) {
            return {raw, voltage_mv / 1000.0f};
          }
          ESP_LOGW(
              TAG,
              "ADC calibration conversion failed in autorange with error %d",
              err);
        }
        return {raw, raw * 3.3f / 4095.0f};
      } else {
        adc_oneshot_chan_cfg_t config = {
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        esp_err_t err = adc_oneshot_config_channel(this->adc2_handle_,
                                                   this->channel_, &config);
        if (err != ESP_OK) {
          ESP_LOGW(TAG, "Error configuring ADC2 channel for autorange: %d",
                   err);
          return {-1, 0.0f};
        }

        int raw;
        err = adc_oneshot_read(this->adc2_handle_, this->channel_, &raw);
        if (err != ESP_OK) {
          ESP_LOGW(TAG, "ADC2 read failed in autorange with error %d", err);
          return {-1, 0.0f};
        }

        if (this->calibration_handle_ != nullptr) {
          int voltage_mv;
          err = adc_cali_raw_to_voltage(this->calibration_handle_, raw,
                                        &voltage_mv);
          if (err == ESP_OK) {
            return {raw, voltage_mv / 1000.0f};
          }
          ESP_LOGW(
              TAG,
              "ADC calibration conversion failed in autorange with error %d",
              err);
        }
        return {raw, raw * 3.3f / 4095.0f};
      }
    };

    auto [raw12, mv12] = read_atten(ADC_ATTEN_DB_12);
    if (raw12 == -1) {
      ESP_LOGE(TAG, "Failed to read ADC in autorange mode");
      return NAN;
    }

    int raw6 = 4095, raw2 = 4095, raw0 = 4095;
    float mv6 = 0, mv2 = 0, mv0 = 0;

    if (raw12 < 4095) {
      auto [raw6_val, mv6_val] = read_atten(ADC_ATTEN_DB_6);
      raw6 = raw6_val;
      mv6 = mv6_val;

      if (raw6 < 4095 && raw6 != -1) {
        auto [raw2_val, mv2_val] = read_atten(ADC_ATTEN_DB_2_5);
        raw2 = raw2_val;
        mv2 = mv2_val;

        if (raw2 < 4095 && raw2 != -1) {
          auto [raw0_val, mv0_val] = read_atten(ADC_ATTEN_DB_0);
          raw0 = raw0_val;
          mv0 = mv0_val;
        }
      }
    }

    if (raw0 == -1 || raw2 == -1 || raw6 == -1 || raw12 == -1) {
      return NAN;
    }

    const int ADC_HALF = 2048;
    uint32_t c12 = std::min(raw12, ADC_HALF);
    uint32_t c6 = ADC_HALF - std::abs(raw6 - ADC_HALF);
    uint32_t c2 = ADC_HALF - std::abs(raw2 - ADC_HALF);
    uint32_t c0 = std::min(4095 - raw0, ADC_HALF);
    uint32_t csum = c12 + c6 + c2 + c0;

    if (csum == 0) {
      ESP_LOGE(TAG, "Invalid weight sum in autorange calculation");
      return NAN;
    }

    return (mv12 * c12 + mv6 * c6 + mv2 * c2 + mv0 * c0) / csum;
  }
}

} // namespace adc
