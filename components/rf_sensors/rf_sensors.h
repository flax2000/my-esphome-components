#pragma once

#include <utility>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/automation.h"


#if defined(ESP8266)
// interrupt handler and related code must be in RAM on ESP8266,
// according to issue #46.
#define RECEIVE_ATTR IRAM_ATTR
#define VAR_ISR_ATTR
#elif defined(ESP32)
#define RECEIVE_ATTR IRAM_ATTR
#define VAR_ISR_ATTR DRAM_ATTR
#else
#define RECEIVE_ATTR
#define VAR_ISR_ATTR
#endif




namespace esphome {
namespace rf_sensors {

//#if defined(USE_ESP8266) || defined(USE_LIBRETINY)
struct RemoteReceiverComponentStore {
  static void gpio_intr(RemoteReceiverComponentStore *arg);

};
//#endif


struct RF_SensorsData {
  uint16_t id;
  float temperature;
  float humidity;
};


class RF_SensorsComponent : public Component {
  public:
    void set_pin(InternalGPIOPin *pin) {
      pin_ = pin;
    }
    float get_setup_priority() const override;
    void setup() override;
    void loop() override;

    void dump_config() override;
    void add_on_sensor_received_callback(std::function<void(RF_SensorsData)> callback) {
      this->data_callback_.add(std::move(callback));
    }


  protected:
    //#if defined(USE_ESP8266) || defined(USE_LIBRETINY)
    RemoteReceiverComponentStore store_;
    HighFrequencyLoopRequester high_freq_;
    //#endif
    InternalGPIOPin *pin_;
    CallbackManager<void(RF_SensorsData)> data_callback_;
};

class RF_SensorsReceivedSensorTrigger : public Trigger<RF_SensorsData> {
  public:
    explicit RF_SensorsReceivedSensorTrigger(RF_SensorsComponent *parent) {
      parent->add_on_sensor_received_callback([this](RF_SensorsData data) {
        this->trigger(data);
      });
    }
};



}  // namespace rf_sensors
}  // namespace esphome
