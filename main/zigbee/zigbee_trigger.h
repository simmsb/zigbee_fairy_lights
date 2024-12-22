#pragma once

#include "zigbee.h"
#include "zigbee_attribute.h"

namespace zigbee {

template <typename Ts> class ZigBeeOnValueTrigger {
public:
  virtual void trigger(Ts) = 0;

  explicit ZigBeeOnValueTrigger(ZigBeeAttribute *parent) : parent_(parent) {}
  void setup() {
    this->parent_->add_on_value_callback(
        [this](esp_zb_zcl_attribute_t attribute) {
          this->on_value_(attribute);
        });
  }

protected:
  void on_value_(esp_zb_zcl_attribute_t attribute) {
    if (attribute.data.type == parent_->attr_type() && attribute.data.value) {
      this->trigger(
          get_value_by_type<Ts>(parent_->attr_type(), attribute.data.value));
    }
  }
  ZigBeeAttribute *parent_;
};

template <class T> T get_value_by_type(uint8_t attr_type, void *data) {
  switch (attr_type) {
  case ESP_ZB_ZCL_ATTR_TYPE_SEMI:
    return 0; //(T) * (std::float16_t *) data;
  case ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING:
    return 0;
  case ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING:
    return 0;
  case ESP_ZB_ZCL_ATTR_TYPE_LONG_OCTET_STRING:
    return 0;
  case ESP_ZB_ZCL_ATTR_TYPE_LONG_CHAR_STRING:
    return 0;
  case ESP_ZB_ZCL_ATTR_TYPE_ARRAY:
    return 0;
  case ESP_ZB_ZCL_ATTR_TYPE_16BIT_ARRAY:
    return 0;
  case ESP_ZB_ZCL_ATTR_TYPE_32BIT_ARRAY:
    return 0;
  case ESP_ZB_ZCL_ATTR_TYPE_STRUCTURE:
    return 0;
  case ESP_ZB_ZCL_ATTR_TYPE_SET:
    return 0;
  case ESP_ZB_ZCL_ATTR_TYPE_BAG:
    return 0;
  case ESP_ZB_ZCL_ATTR_TYPE_TIME_OF_DAY:
    return (T) * (uint32_t *)data;
  case ESP_ZB_ZCL_ATTR_TYPE_DATE:
    return (T) * (uint32_t *)data;
  case ESP_ZB_ZCL_ATTR_TYPE_UTC_TIME:
    return (T) * (uint32_t *)data;
  case ESP_ZB_ZCL_ATTR_TYPE_CLUSTER_ID:
    return (T) * (uint16_t *)data;
  case ESP_ZB_ZCL_ATTR_TYPE_ATTRIBUTE_ID:
    return (T) * (uint16_t *)data;
  case ESP_ZB_ZCL_ATTR_TYPE_BACNET_OID:
    return 0;
  case ESP_ZB_ZCL_ATTR_TYPE_IEEE_ADDR:
    return (T) * (uint64_t *)data;
  case ESP_ZB_ZCL_ATTR_TYPE_128_BIT_KEY:
    return 0;
  default:
    return *(T *)data;
  }
}

} // namespace zigbee
