#pragma once

#include "callbackmanager.h"
#include "esp_zigbee_core.h"
#include "zigbee.h"

namespace zigbee {

class ZigBeeAttribute {
public:
  ZigBeeAttribute(ZigBeeComponent *parent, uint8_t endpoint_id,
                  uint16_t cluster_id, uint8_t role, uint16_t attr_id,
                  uint8_t attr_type)
      : zb_(parent), endpoint_id_(endpoint_id), cluster_id_(cluster_id),
        role_(role), attr_id_(attr_id), attr_type_(attr_type) {}
  // void dump_config() override;

  template <typename T> void add_attr(uint8_t attr_access, T value_p);
  void set_report();
  void set_attr(void *value_p);

  uint8_t attr_type() { return attr_type_; }

  void add_on_value_callback(
      std::function<void(esp_zb_zcl_attribute_t attribute)> callback) {
    on_value_callback_.add(std::move(callback));
  }
  void on_value(esp_zb_zcl_attribute_t attribute) {
    this->on_value_callback_.call(attribute);
  }

#ifdef USE_SENSOR
  template <typename T> void connect(sensor::Sensor *sensor);
  template <typename T>
  void connect(sensor::Sensor *sensor, std::function<T(float)> &&f);
#endif
#ifdef USE_BINARY_SENSOR
  template <typename T> void connect(binary_sensor::BinarySensor *sensor);
  template <typename T>
  void connect(binary_sensor::BinarySensor *sensor, std::function<T(bool)> &&f);
#endif

protected:
  ZigBeeComponent *zb_;
  uint8_t endpoint_id_;
  uint16_t cluster_id_;
  uint8_t role_;
  uint16_t attr_id_;
  uint8_t attr_type_;
  CallbackManager<void(esp_zb_zcl_attribute_t attribute)> on_value_callback_{};
};

template <typename T>
void ZigBeeAttribute::add_attr(uint8_t attr_access, T value_p) {
  this->zb_->add_attr(this, this->endpoint_id_, this->cluster_id_, this->role_,
                      this->attr_id_, this->attr_type_, attr_access, value_p);
}

// #ifdef USE_SENSOR
// template<typename T> void ZigBeeAttribute::connect(sensor::Sensor *sensor) {
//   sensor->add_on_state_callback([=, this](float value) {
//     T value_p = (T) (this->scale_ * value);
//     this->set_attr(&value_p);
//   });
// }

// template<typename T> void ZigBeeAttribute::connect(sensor::Sensor *sensor,
// std::function<T(float)> &&f) {
//   sensor->add_on_state_callback([=, this](float value) {
//     T value_p = f(value);
//     this->set_attr(&value_p);
//   });
// }
// #endif

// #ifdef USE_BINARY_SENSOR
// template<typename T> void
// ZigBeeAttribute::connect(binary_sensor::BinarySensor *sensor) {
//   sensor->add_on_state_callback([=, this](bool value) {
//     T value_p = (T) (this->scale_ * value);
//     this->set_attr(&value_p);
//   });
// }

// template<typename T> void
// ZigBeeAttribute::connect(binary_sensor::BinarySensor *sensor,
// std::function<T(bool)> &&f) {
//   sensor->add_on_state_callback([=, this](bool value) {
//     T value_p = f(value);
//     this->set_attr(&value_p);
//   });
// }
// #endif

} // namespace zigbee
