#include "esp_log.h"
#include "esp_zigbee_type.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "utils/adc_sensor.h"
#include "utils/binary_output.h"
#include "utils/gpio.h"
#include "utils/gpio_binary_output.h"
#include "utils/isr_gpio.h"
#include "utils/ledc.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zigbee/zigbee.h"
#include "zigbee/zigbee_attribute.h"
#include "zigbee/zigbee_trigger.h"
#include <cstdint>

static const char *TAG = "LIGHTS";

gpio::GPIOBinaryOutput *statusLed;

QueueHandle_t ledqueue;

struct LedState {
  bool on;
  uint8_t level;
};

struct SetLedState {
  union {
    bool on;
    uint8_t level;
  } u;
  bool isOnSet;
};

class OnOffHandler : public zigbee::ZigBeeOnValueTrigger<bool> {
  using zigbee::ZigBeeOnValueTrigger<bool>::ZigBeeOnValueTrigger;

  void trigger(bool x) {
    ESP_LOGI(TAG, "on off triggered: %d", x);
    auto msg = SetLedState{.u =
                               {
                                   .on = x,
                               },
                           .isOnSet = true};
    statusLed->turn_on();
    xQueueSend(ledqueue, &msg, portMAX_DELAY);
    statusLed->turn_off();
  }
};

class SetLevelHandler : public zigbee::ZigBeeOnValueTrigger<uint8_t> {
  using zigbee::ZigBeeOnValueTrigger<uint8_t>::ZigBeeOnValueTrigger;

  void trigger(uint8_t x) {
    ESP_LOGI(TAG, "set level triggered: %d", x);
    auto msg = SetLedState{.u =
                               {
                                   .level = x,
                               },
                           .isOnSet = false};
    statusLed->turn_on();
    xQueueSend(ledqueue, &msg, portMAX_DELAY);
    statusLed->turn_off();
  }
};

ledc::LEDCOutput *ledOutput;

void ledUpdateTask(void *arg) {
  uint8_t level = 0;
  uint8_t savedLevel = 0;

  for (;;) {
    SetLedState newStateSet;
    if (xQueueReceive(ledqueue, &newStateSet, portMAX_DELAY) != pdPASS) {
      continue;
    }

    auto desiredLevel = level;

    if (newStateSet.isOnSet) {
      if (newStateSet.u.on)
        desiredLevel = savedLevel;
      else {
        savedLevel = level;
        desiredLevel = 0;
      }
    } else {
      desiredLevel = newStateSet.u.level;
    }

    if (level > 0 || desiredLevel > 0) {
      zigbee::inhibit_sleep();
      // need to run setup again after sleeping?
      ledOutput->setup();
    }

    int direction = (level < desiredLevel) ? 1 : -1;
    for (size_t i = level, iEnd = desiredLevel + direction; i != iEnd;
         i += direction) {
      ledOutput->set_level((float)i / 255.0);

      vTaskDelay(pdMS_TO_TICKS(32));
    }

    level = desiredLevel;

    if (level == 0) {
      zigbee::allow_sleep();
    }
  }
}

adc::ADCSensor *batteryADC;

float interp_linear(float value, std::vector<std::array<float, 3>> filter) {
  for (std::array<float, 3> f : filter) {
    if (!std::isfinite(f[2]) || value < f[2])
      return (value * f[0]) + f[1];
  }
  return NAN;
}

zigbee::ZigBeeAttribute *power_cfg_battery_remaining;

void batteryUpdateTask(void *arg) {
  std::vector<std::array<float, 3>> filt = {
      {0.30303030303030304f, 0.0f, 3.3f},
      {99.99999999999966f, -328.99999999999886f, 3.39f},
      {111.11111111111114f, -366.6666666666668f, 3.75f},
      {111.11111111111101f, -366.6666666666663f, 4.11f},
      {111.11111111111128f, -366.6666666666674f, NAN}};

  for (;;) {
    auto r = interp_linear(batteryADC->sample() * 2.0, filt);

    if (r > 100.0) {
      r = 100.0;
    } else if (r < 0.0) {
      r = 0.0;
    }

    uint8_t value = r * (200.0 / 100.0);

    ESP_LOGI(TAG, "Ticking adc, got: %d", value);

    power_cfg_battery_remaining->set_attr(&value);

    vTaskDelay(pdMS_TO_TICKS(1000 * 60));
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "hello world");
  ESP_ERROR_CHECK(nvs_flash_init());

  auto ledpin = new esp32::ESP32InternalGPIOPin();
  ledpin->set_pin(GPIO_NUM_15);
  ledpin->set_inverted(false);
  ledpin->set_drive_strength(GPIO_DRIVE_CAP_2);
  ledpin->set_flags(gpio::FLAG_OUTPUT);
  ledpin->setup();
  statusLed = new gpio::GPIOBinaryOutput();
  statusLed->set_pin(ledpin);
  statusLed->setup();

  ledqueue = xQueueCreate(1, sizeof(LedState));

  auto zb = new zigbee::ZigBeeComponent();
  zb->set_basic_cluster("toad-lights", "ben", "2024", 3, 0, 0, 0, "", 0);
  zb->create_default_cluster(1, ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID);
  zb->add_cluster(1, ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  zb->add_cluster(1, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

  auto on_off_attr = new zigbee::ZigBeeAttribute(
      zb, 1, ::ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ::ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      0, ::ESP_ZB_ZCL_ATTR_TYPE_BOOL);
  on_off_attr->add_attr(0, 0);

  auto mainoutputpin = new esp32::ESP32InternalGPIOPin();
  mainoutputpin->set_pin(GPIO_NUM_17);
  mainoutputpin->set_inverted(false);
  mainoutputpin->set_drive_strength(GPIO_DRIVE_CAP_2);
  mainoutputpin->set_flags(gpio::FLAG_OUTPUT);
  mainoutputpin->setup();
  auto mainoutput = new ledc::LEDCOutput(mainoutputpin);
  mainoutput->set_frequency(10000.0);
  mainoutput->set_zero_means_zero(false);
  mainoutput->setup();
  mainoutput->set_state(false);

  ledOutput = mainoutput;

  xTaskCreate(ledUpdateTask, "ledUpdate", 2048, NULL, 10, NULL);

  auto batterypin = new esp32::ESP32InternalGPIOPin();
  batterypin->set_pin(GPIO_NUM_0);
  batterypin->set_inverted(false);
  batterypin->set_drive_strength(GPIO_DRIVE_CAP_2);
  batterypin->set_flags(gpio::FLAG_INPUT);
  batterypin->setup();
  auto batteryLevel = new adc::ADCSensor();
  batteryLevel->set_pin(batterypin);
  batteryLevel->set_output_raw(false);
  batteryLevel->set_sample_count(10);
  batteryLevel->set_attenuation(adc::ADC_ATTEN_DB_12_COMPAT);
  batteryLevel->set_channel1(::ADC1_CHANNEL_0);
  batteryLevel->setup();
  batteryADC = batteryLevel;

  (new OnOffHandler(on_off_attr))->setup();
  // on_off_attr->add_on_value_callback([=](esp_zb_zcl_attr_t x) {
  // });

  zb->add_cluster(1, ::ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                  ::ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  auto level_attr = new zigbee::ZigBeeAttribute(
      zb, 1, ::ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
      ::ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 0, ::ESP_ZB_ZCL_ATTR_TYPE_U8);
  level_attr->add_attr(0, 0);

  (new SetLevelHandler(level_attr))->setup();

  zb->add_cluster(1, ::ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                  ::ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
  power_cfg_battery_remaining = new zigbee::ZigBeeAttribute(
      zb, 1, ::ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
      ::ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
      ::ESP_ZB_ZCL_ATTR_TYPE_U8);
  power_cfg_battery_remaining->add_attr(0, 0);
  power_cfg_battery_remaining->set_report();

  auto power_cfg_battery_size = new zigbee::ZigBeeAttribute(
      zb, 1, ::ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
      ::ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, ::ESP_ZB_ZCL_ATTR_TYPE_U8);
  power_cfg_battery_size->add_attr(0,
                                   ZB_ZCL_POWER_CONFIG_BATTERY_SIZE_BUILT_IN);

  xTaskCreate(batteryUpdateTask, "batteryUpdate", 4096, NULL, 10, NULL);

  zb->setup();
}
