#include "rf_sensors.h"
#include "esphome/core/log.h"
#include <cinttypes>
#include <cstring>
#include "esphome/core/helpers.h"
namespace esphome {
namespace rf_sensors {

static const char *const TAG = "rf_sensors";


//made by swedude

//---------------------------------------------------------------------------------------------------------------------------------------

//#define fineoff_debug
//#define fineoff_rssi_read  //rssi read leftover from when i used ths code with cc1101

volatile VAR_ISR_ATTR  uint64_t code_fine_off_RX;
volatile VAR_ISR_ATTR  uint64_t code_fine_off_RX_done;
volatile VAR_ISR_ATTR  uint16_t fine_off_packet_done = false;

#if defined(fineoff_debug)
volatile VAR_ISR_ATTR  uint16_t max_1000;
volatile VAR_ISR_ATTR  uint16_t min_1000;
volatile VAR_ISR_ATTR  uint16_t max_1500;
volatile VAR_ISR_ATTR  uint16_t min_1500;
volatile VAR_ISR_ATTR  uint16_t max_500;
volatile VAR_ISR_ATTR  uint16_t min_500;
#endif

float temp;
float hum;
uint16_t have_data_sensor = 0;
#if defined(fineoff_rssi_read)
int16_t rssi = 0;
int16_t rssi_packet = 0;
int16_t rssi_background = 0;
volatile uint16_t get_rssi = false;
#endif


 //used "https://github.com/lucsmall/BetterWH2" as a starting point
/* --------------------------------------------------fine off----------------------------------
  // 1 is indicated by 500uS pulse
  // wh2_accept from 2 = 400us to 3 = 600us
  #define IS_HI_PULSE(interval)   (interval >= 200 && interval <= 600)//550
  // 0 is indicated by ~1500us pulse
  // wh2_accept from 7 = 1400us to 8 = 1600us
  #define IS_LOW_PULSE(interval)  (interval >= 1200 && interval <= 1500)//1550 1180
  // worst case packet length
  #define IS_IDLE_PULSE(interval)  (interval >= 600 && interval <= 1200)
  // our expected pulse should arrive after 1ms
*/
// 1 is indicated by 500uS pulse
// wh2_accept from 2 = 400us to 3 = 600us
#define IS_HI_PULSE(interval)   (interval >= 200 && interval <= 800)//699
// 0 is indicated by ~1500us pulse
// wh2_accept from 7 = 1400us to 8 = 1600us
#define IS_LOW_PULSE(interval)  (interval >= 1200 && interval <= 2200)//1550 1180  //1200
// worst case packet length
#define IS_IDLE_PULSE(interval)  (interval >= 800 && interval <= 1500) //700 //1199
// our expected pulse should arrive after 1ms



void RECEIVE_ATTR fine_off_rx(uint16_t duration)
{
#if defined(fineoff_debug)
  if (fine_off_packet_done == false)
  {
#endif
    static uint16_t duration_old = 0;
    static uint16_t preamble_done = false;
    static uint16_t wait_next_fine_off = false;
    static uint16_t counter_fine_off = 0;
    if (wait_next_fine_off == true)
    {
      wait_next_fine_off = false;
    }

    else
    {
#if defined(fineoff_debug)
      if (counter_fine_off == 4 && preamble_done == false)
      {
        max_1000 = 0;
        min_1000 = 1500;
        max_1500 = 0;
        min_1500 = 2000;
        max_500 = 0;
        min_500 = 1000;
      }
#endif



      if (IS_IDLE_PULSE(duration_old) && IS_HI_PULSE(duration))
      {
        code_fine_off_RX <<= 1;
        counter_fine_off++;
        code_fine_off_RX |= 1;
        wait_next_fine_off = true;
#if defined(fineoff_debug)
        if (duration_old > max_1000) max_1000 = duration_old;
        if (duration_old < min_1000) min_1000 = duration_old;
        if (duration > max_500) max_500 = duration;
        if (duration < min_500) min_500 = duration;
#endif
      }

      else if (IS_IDLE_PULSE(duration_old) && IS_LOW_PULSE(duration) && (counter_fine_off > 2 || preamble_done == true)) //preamble 0xff >2 because might miss the first few....
      {
        if (preamble_done == false)
        {
          preamble_done = true;
          counter_fine_off = 0;
        }
        code_fine_off_RX <<= 1;
        counter_fine_off++;
        wait_next_fine_off = true;
#if defined(fineoff_debug)
        if (duration_old > max_1000) max_1000 = duration_old;
        if (duration_old < min_1000) min_1000 = duration_old;
        if (duration > max_1500) max_1500 = duration;
        if (duration < min_1500) min_1500 = duration;
#endif
      }

      else
      {
        counter_fine_off = 0;
        preamble_done = false;
      }
#if defined(fineoff_rssi_read)
      if (counter_fine_off == 21 && preamble_done == true) get_rssi = true;
#endif

      if (counter_fine_off == 40)
      {
        if (fine_off_packet_done == false)
        {
#if defined(fineoff_rssi_read)
          if (get_rssi == false)rssi_packet = rssi;
          else rssi_packet = 0;
#endif
          code_fine_off_RX_done = code_fine_off_RX;
          fine_off_packet_done = true;
        }
        preamble_done = false;
        counter_fine_off = 0;
        wait_next_fine_off = false;
        return;
      }



    }

    duration_old = duration;
#if defined(fineoff_debug)
  }
#endif
}









void IRAM_ATTR HOT RemoteReceiverComponentStore::gpio_intr(RemoteReceiverComponentStore *arg)
{

  static unsigned long edgeTimeStamp[3] = {0, };  // Timestamp of edges
  static bool skip;
  // Filter out too short pulses. This method works as a low pass filter.
  edgeTimeStamp[1] = edgeTimeStamp[2];
  edgeTimeStamp[2] = micros();



  if (skip) {
    skip = false;
    return;
  }

  if (edgeTimeStamp[2] - edgeTimeStamp[1] < 120) { // filter out <120 pulses
    // Last edge was too short.
    // Skip this edge, and the next too.
    skip = true;
    return;
  }

  uint16_t duration = edgeTimeStamp[1] - edgeTimeStamp[0];
  edgeTimeStamp[0] = edgeTimeStamp[1];



  fine_off_rx(duration);


}






//finoffset decoding are from rtl_433


void RF_SensorsComponent::loop() {


  if (fine_off_packet_done == true)
  {
    uint8_t wh2_packet[5];
    uint8_t wh2_valid;

    wh2_packet[4] = code_fine_off_RX_done  & 0xff;
    wh2_packet[3] = code_fine_off_RX_done >> 8 & 0xff;
    wh2_packet[2] = code_fine_off_RX_done >> 16  & 0xff;
    wh2_packet[1] = code_fine_off_RX_done >> 24 & 0xff;
    wh2_packet[0] = code_fine_off_RX_done >> 32 & 0xff;

    int Sensor_ID = (wh2_packet[0] << 4) + (wh2_packet[1] >> 4);
    int humidity = wh2_packet[3];
    int temperature = ((wh2_packet[1] & 0x7) << 8) + wh2_packet[2];
    // make negative
    if (wh2_packet[1] & 0x8) {
      temperature = -temperature;
    }
    uint8_t crc = 0;
    uint8_t len = 4;
    uint8_t *addr = wh2_packet;
    // Indicated changes are from reference CRC-8 function in OneWire library
    while (len--) {
      uint8_t inbyte = *addr++;
      for (uint8_t i = 8; i; i--) {
        uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
        crc <<= 1; // changed from right shift
        if (mix) crc ^= 0x31;// changed from 0x8C;
        inbyte <<= 1; // changed from right shift
      }
    }
    if (crc == wh2_packet[4])
    {
      wh2_valid = true;
    }
    else
    {
      wh2_valid = false;

      ESP_LOGD("wh2", "--BAD crc-- ");
    }

    if (wh2_valid == true) 
    {
      temp = temperature / 10.0;
      hum = humidity;
      have_data_sensor = (Sensor_ID & 0xff);
    }





    if (have_data_sensor)
    {
#if defined (fineoff_rssi_read) && defined (fineoff_debug)

      ESP_LOGD("wh2", "id:%i |%i-%i|%i-%i|%i-%i| tmp %.1f hum %.0f rssi %i rssi_background %i" , have_data_sensor, max_1500, min_1500, max_1000, min_1000, max_500, min_500, temp, hum, rssi_packet, rssi_background);
#endif
# if defined(fineoff_rssi_read) && !defined(fineoff_debug)
      ESP_LOGD("wh2", "id:%i tmp %.1f hum %.0f rssi %i rssi_background %i", have_data_sensor, temp, hum, rssi, rssi_background);
#endif
# if !defined(fineoff_rssi_read) && defined(fineoff_debug)
      ESP_LOGD("wh2", "id:%i |%i-%i|%i-%i|%i-%i| tmp %.1f hum %.0f " , have_data_sensor, max_1500, min_1500, max_1000, min_1000, max_500, min_500, temp, hum);
#endif
# if !defined(fineoff_rssi_read) && !defined(fineoff_debug)
      //ESP_LOGD("wh2", "id:%i tmp %.1f hum %.0f ", have_data_sensor, temp, hum);
#endif



      RF_SensorsData data;
      data.id = have_data_sensor;
      data.temperature = temp;
      data.humidity = hum;


      this->data_callback_.call(data);
      have_data_sensor = 0;



    }
    fine_off_packet_done = false;
  }

}

void RF_SensorsComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RF433...");
  this->pin_->pin_mode(gpio::FLAG_NONE);
  this->pin_->pin_mode(gpio::FLAG_INPUT);
  this->pin_->attach_interrupt(RemoteReceiverComponentStore::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
  //this->pin_->setup();




}



void RF_SensorsComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "RF_Sensors:");

}
float RF_SensorsComponent::get_setup_priority() const {
  return setup_priority::AFTER_WIFI;
}

}  // namespace rf_sensors
}  // namespace esphome
