//made by swe-dude
#include "rf_raw.h"
#include "esphome/core/log.h"
#include <cinttypes>
#include <cstring>
#include "esphome/core/helpers.h"
namespace esphome {
namespace rf_raw {

static const char *const TAG = "rf_raw";

InternalGPIOPin *RX433_PIN;
bool start_int=false;

volatile VAR_ISR_ATTR int8_t  rc_switch_state=0;
volatile VAR_ISR_ATTR int8_t capture433 = 0;
volatile VAR_ISR_ATTR int8_t capture433_repeating = 0;
volatile VAR_ISR_ATTR int16_t nSeparationLimit = 2000;
volatile VAR_ISR_ATTR int16_t tolerance = 90;
volatile VAR_ISR_ATTR int16_t nSeparation_pulse_state;









//--------------------------------------------------------------RCSWITCH------------------------------------------------------

volatile VAR_ISR_ATTR uint32_t recieve_rc_adress = 0;
volatile VAR_ISR_ATTR int8_t recieve_rc_protocol = 0;
#define RCSWITCH_MAX_CHANGES 67
uint16_t timings[RCSWITCH_MAX_CHANGES];
struct HighLow {
  uint8_t high;
  uint8_t low;
};

struct Protocol {
  /** base pulse length in microseconds, e.g. 350 */
  uint16_t pulseLength;
  HighLow syncFactor;
  HighLow zero;
  HighLow one;
  bool invertedSignal;
};


static inline unsigned int diff(int A, int B) {
  return abs(A - B);
}

bool RECEIVE_ATTR receiveProtocol(const int p, unsigned int changeCount) {

  static const VAR_ISR_ATTR Protocol proto[] = {

    { 350, { 1, 31 }, { 1, 3 }, { 3, 1 }, false },     // protocol 1
    { 650, { 1, 10 }, { 1, 2 }, { 2, 1 }, false },     // protocol 2
    { 100, { 30, 71 }, { 4, 11 }, { 9, 6 }, false },   // protocol 3
    { 380, { 1, 6 }, { 1, 3 }, { 3, 1 }, false },      // protocol 4
    { 500, { 6, 14 }, { 1, 2 }, { 2, 1 }, false },     // protocol 5
    { 450, { 23, 1 }, { 1, 2 }, { 2, 1 }, true },      // protocol 6 (HT6P20B)
    { 150, { 2, 62 }, { 1, 6 }, { 6, 1 }, false },     // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
    { 200, { 3, 130 }, { 7, 16 }, { 3, 16 }, false },  // protocol 8 Conrad RS-200 RX
    { 200, { 130, 7 }, { 16, 7 }, { 16, 3 }, true },   // protocol 9 Conrad RS-200 TX
    { 365, { 18, 1 }, { 3, 1 }, { 1, 3 }, true },      // protocol 10 (1ByOne Doorbell)
    { 270, { 36, 1 }, { 1, 2 }, { 2, 1 }, true },      // protocol 11 (HT12E)
    { 320, { 36, 1 }, { 1, 2 }, { 2, 1 }, true }       // protocol 12 (SM5212)
  };

  const Protocol &pro = proto[p - 1];
  int16_t nReceiveTolerance = tolerance;  //adjust the tolerance here 60 original value, 120 might be too high decrease if you get strange signals...
  unsigned long code = 0;
  //Assuming the longer pulse length is the pulse captured in timings[0]
  const uint16_t syncLengthInPulses = ((pro.syncFactor.low) > (pro.syncFactor.high)) ? (pro.syncFactor.low) : (pro.syncFactor.high);
  const uint16_t delay = timings[0] / syncLengthInPulses;
  const uint16_t delayTolerance = delay * nReceiveTolerance / 100;

  /* For protocols that start low, the sync period looks like
                   _________
     _____________|         |XXXXXXXXXXXX|

     |--1st dur--|-2nd dur-|-Start data-|

     The 3rd saved duration starts the data.

     For protocols that start high, the sync period looks like

      ______________
     |              |____________|XXXXXXXXXXXXX|

     |-filtered out-|--1st dur--|--Start data--|

     The 2nd saved duration starts the data
  */
  const uint16_t firstDataTiming = (pro.invertedSignal) ? (2) : (1);

  for (uint16_t i = firstDataTiming; i < changeCount - 1; i += 2) {
    code <<= 1;
    if (diff(timings[i], delay * pro.zero.high) < delayTolerance && diff(timings[i + 1], delay * pro.zero.low) < delayTolerance) {
      // zero
    } else if (diff(timings[i], delay * pro.one.high) < delayTolerance && diff(timings[i + 1], delay * pro.one.low) < delayTolerance) {
      // one
      code |= 1;
    } else {
      // Failed
      return false;
    }
  }


  if (code > 10000)  // this will ignore 0 or 10 or ... sinals....
  {
    recieve_rc_adress = code;
    recieve_rc_protocol = p;
  }

  return true;
}


void RECEIVE_ATTR rc_sw_rx_(uint16_t duration) {
  static uint16_t changeCount = 0;
  static uint16_t repeatCount = 0;
if(!recieve_rc_adress)
{
  if (duration > nSeparationLimit) {
    // A long stretch without signal level change occurred. This could
    // be the gap between two transmission.
    if ((repeatCount == 0) || (diff(duration, timings[0]) < 200)) {
      // This long signal is close in length to the long signal which
      // started the previously recorded timings; this suggests that
      // it may indeed by a a gap between two transmissions (we assume
      // here that a sender will send the signal multiple times,
      // with roughly the same gap between them).
      repeatCount++;
      if (repeatCount == 2) {

        if (((changeCount - 1) / 2) > 22)  //only check for codes with lenght over 22, decrease if you need to catch shorter codes.
        {
            for (uint16_t i = 1; i <= 12; i++) {
            if (receiveProtocol(i, changeCount)) {
             // receive succeeded for protocol i
             break;
            }
            }

        }
        repeatCount = 0;
      }
    }
    changeCount = 0;
  }

  // detect overflow
  if (changeCount >= RCSWITCH_MAX_CHANGES) {
    changeCount = 0;
    repeatCount = 0;
  }
  timings[changeCount++] = duration;

}
}
















#define RAW_MAX_CHANGES 150
//--------------------------------------------------------------raw repeating find-- rc switch part of code from https://github.com/sui77/rc-switch------------------------------------------------------------------------------------------------

unsigned int RECEIVE_ATTR diff_raw(int A, int B) {
  return abs(A - B);
}

int32_t timings_raw[RAW_MAX_CHANGES];
volatile VAR_ISR_ATTR int raw_lenght_done = 0;
void RECEIVE_ATTR raw_data(uint16_t duration) {

  static uint16_t changeCount = 0;
  static uint16_t repeatCount = 0;

  static uint16_t lenght = 0;
  static uint16_t counter = 1;
  if (raw_lenght_done == 0) {

    if (duration > nSeparationLimit) {
      nSeparation_pulse_state = RX433_PIN->digital_read();
      // A long stretch without signal level change occurred. This could
      // be the gap between two transmission.
      if ((repeatCount == 0) || (diff_raw(duration, timings_raw[0]) < (tolerance*3))) {
        // This long signal is close in length to the long signal which
        // started the previously recorded timings; this suggests that
        // it may indeed by a a gap between two transmissions (we assume
        // here that a sender will send the signal multiple times,
        // with roughly the same gap between them).
        repeatCount++;
        if (repeatCount == 2) {

          if ((changeCount - 1) > 8)  //ignore short transmissions
          {
            raw_lenght_done = 1;
            lenght = changeCount - 1;
            repeatCount = 0;
            changeCount = 0;
            return;
          }
          repeatCount = 0;
        }
      }
      changeCount = 0;
    }

    // detect overflow
    if (changeCount >= RAW_MAX_CHANGES) {
      changeCount = 0;
      repeatCount = 0;
    }


    timings_raw[changeCount++] = duration;
  }


  if (raw_lenght_done == 1) {
    if ((diff_raw(duration, timings_raw[counter]) < tolerance)) {
      counter++;

      if (counter == lenght) {
        raw_lenght_done = lenght+1;
        counter = 1;
        return;
      }
    } else {
      raw_lenght_done = 0;
      counter = 1;
    }
  }
}


#define RAW_CAPTURE_MAX_CHANGES 1000
//--------------------------------------------------------------raw repeating find-- rc switch part of code from https://github.com/sui77/rc-switch------------------------------------------------------------------------------------------------

int32_t timings_raw_capture[RAW_CAPTURE_MAX_CHANGES];
boolean raw_capture_done=0;

void RECEIVE_ATTR raw_data_capture(uint16_t duration) {

  static uint16_t counter = 0;
  
  if(raw_capture_done==0&&capture433==1)
  {
	if(counter<RAW_CAPTURE_MAX_CHANGES)
	{		
	 timings_raw_capture[counter]=duration; 
	 counter++;
	}
else
{
counter=0;	
capture433=0;	
raw_capture_done=1;	
	
}	
	
  }

}





void IRAM_ATTR HOT RemoteReceiverComponentStore::gpio_intr(RemoteReceiverComponentStore *arg)
{

  static unsigned long edgeTimeStamp = 0;  // Timestamp of edge
  unsigned long micros_tmp=micros();
  uint16_t duration = micros_tmp - edgeTimeStamp;
  edgeTimeStamp = micros_tmp;
  if(capture433_repeating)raw_data(duration);
  raw_data_capture(duration);
  if(rc_switch_state) rc_sw_rx_(duration);
  
}


void RF_RawComponent::loop() {
if(start_int==true)
{
this->pin_->attach_interrupt(RemoteReceiverComponentStore::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
ESP_LOGD("rx", "int on");	
start_int=false;
}



if(recieve_rc_adress)
{
  ESP_LOGD("rc_", "adress:%i protocol %i  ", recieve_rc_adress, recieve_rc_protocol);
  recieve_rc_adress=0;
}



      if (raw_lenght_done > 1)
      {
        //below modefied code from esphome raw_protocol.cpp
        static const char *const TAG = "diy raw";
        char buffer[256];
        uint32_t buffer_offset = 0;
        buffer_offset += sprintf(buffer, "size: %i idle pulse %i pulses: [", (raw_lenght_done), timings_raw[0]);
        for (int32_t i = 0; i < raw_lenght_done; i++)
        {
          const int32_t value = timings_raw[i]; //-->> dont write the idle pulse here
          const uint32_t remaining_length = sizeof(buffer) - buffer_offset;
          int written;

          if (nSeparation_pulse_state)
          {
            if (i < raw_lenght_done-1) {
              written = snprintf(buffer + buffer_offset, remaining_length, "-%d, ", value);
            } else {
              written = snprintf(buffer + buffer_offset, remaining_length, "-%d]", value);
            }

            if (written < 0 || written >= int(remaining_length)) {
              // write failed, flush...
              buffer[buffer_offset] = '\0';
              ESP_LOGD(TAG, "%s", buffer);
              buffer_offset = 0;
              written = sprintf(buffer, "  ");
              if (i < raw_lenght_done-1) {
                written += sprintf(buffer + written, "-%d, ", value);
              } else {
                written += sprintf(buffer + written, "-%d]", value);
              }
            }
          }
          else
          {

            if (i < raw_lenght_done-1) {
              written = snprintf(buffer + buffer_offset, remaining_length, "%d, ", value);
            } else {
              written = snprintf(buffer + buffer_offset, remaining_length, "%d]", value);
            }

            if (written < 0 || written >= int(remaining_length)) {
              // write failed, flush...
              buffer[buffer_offset] = '\0';
              ESP_LOGD(TAG, "%s", buffer);
              buffer_offset = 0;
              written = sprintf(buffer, "  ");
              if (i < raw_lenght_done-1) {
                written += sprintf(buffer + written, "%d, ", value);
              } else {
                written += sprintf(buffer + written, "%d]", value);
              }
            }
          }

          nSeparation_pulse_state = !nSeparation_pulse_state;
          buffer_offset += written;
        }
        if (buffer_offset != 0) {
          ESP_LOGD(TAG, "%s", buffer);
        }

        ESP_LOGD(TAG, "signal done ");
        ESP_LOGD("", "");
        raw_lenght_done = 0;
      }






      if (raw_capture_done == 1)
      {
        //below modefied code from esphome raw_protocol.cpp
        static const char *const TAG = "diy raw capture";
        char buffer[256];
        uint32_t buffer_offset = 0;
        buffer_offset += sprintf(buffer, "size: %i  ", (RAW_CAPTURE_MAX_CHANGES - 1));
        for (int32_t i = 0; i < RAW_CAPTURE_MAX_CHANGES - 1; i++)
        {
          const int32_t value = timings_raw_capture[i]; //-->> dont write the idle pulse here
          const uint32_t remaining_length = sizeof(buffer) - buffer_offset;
          int written;

            if (i < RAW_CAPTURE_MAX_CHANGES - 1) {
              written = snprintf(buffer + buffer_offset, remaining_length, "%d,", value);
            } else {
              written = snprintf(buffer + buffer_offset, remaining_length, "%d", value);
            }

            if (written < 0 || written >= int(remaining_length)) {
              // write failed, flush...
              buffer[buffer_offset] = '\0';
              ESP_LOGD(TAG, "%s", buffer);
              buffer_offset = 0;
              written = sprintf(buffer, "  ");
              if (i < RAW_CAPTURE_MAX_CHANGES) {
                written += sprintf(buffer + written, "%d,", value);
              } else {
                written += sprintf(buffer + written, "%d", value);
              }
            }
          


          buffer_offset += written;
        }
        if (buffer_offset != 0) {
          ESP_LOGD(TAG, "%s", buffer);
        }

        ESP_LOGD(TAG, "signal done ");
        ESP_LOGD("", "");
        raw_capture_done = 0;
      }






}







void set_capture433(int8_t value)
{
capture433=value;
}

void set_capture433_repeating(int8_t value)
{
capture433_repeating=value;
}

void set_nSeparationLimit(int16_t value)
{
nSeparationLimit=value;
}

void set_tolerance(int16_t value)
{
tolerance=value;
}

void set_rc_switch(int8_t value)
{
rc_switch_state=value;  
}



void rx_int_off()
{
RX433_PIN->detach_interrupt();
ESP_LOGD("rx", "int off");

}

void rx_int_on()
{
start_int=true;
}



void RF_RawComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RF433...");
  this->pin_->pin_mode(gpio::FLAG_INPUT);
  this->pin_->digital_write(false);
  RX433_PIN=pin_;
  this->high_freq_.start();
  this->pin_->attach_interrupt(RemoteReceiverComponentStore::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
  //this->pin_->setup();

}



void RF_RawComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "RF_Raw:");

}
float RF_RawComponent::get_setup_priority() const {
  return setup_priority::AFTER_CONNECTION;
}

}  // namespace rf_raw
}  // namespace esphome
