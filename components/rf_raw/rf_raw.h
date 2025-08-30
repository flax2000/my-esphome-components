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
namespace rf_raw {

//#if defined(USE_ESP8266) || defined(USE_LIBRETINY)
struct RemoteReceiverComponentStore {
  static void gpio_intr(RemoteReceiverComponentStore *arg);

};
//#endif


void set_capture433(int8_t);
void set_capture433_repeating(int8_t);
void set_nSeparationLimit(int16_t);
void set_tolerance(int16_t);
void set_rc_switch(int8_t);


    void rx_int_off();
    void rx_int_on();   

class RF_RawComponent : public Component {
  public:
    void set_pin(InternalGPIOPin *pin) {
      pin_ = pin;
    }
    float get_setup_priority() const override;
    void setup() override;
    void loop() override;
 
    void dump_config() override;


    //#endif
    InternalGPIOPin *pin_;


 protected:
//#if defined(USE_ESP8266) || defined(USE_LIBRETINY)
  RemoteReceiverComponentStore store_;
  HighFrequencyLoopRequester high_freq_;
//#endif


};




}  // namespace rf_raw
}  // namespace esphome
