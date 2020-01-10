## Si1133

The Silicon Labs 1133 is a light sensor with sensors for ambient, infrared and UV light. The chip uses I2C and offers a 
sparate open drain interrupt pin. It is highly configurable and comes with a timer that can be programmed from 800us to
3.7h.

This library provides an easy to use API for Arduino that supports pretty much all the sensor can do.

Dependencies: Arduino-Util

Example:
~~~~C++
#include <Si1133.h>

Si1133 light_sensor(Si1133::kSi1133AddrAPinLow);

void setup() {
  Wire.begin(4, 5);
  delay(100);
  
  // Setup light sensor
  if(light_sensor.begin() == 0) {
    Serial.println("Si1133 found");
    // Initialize measurement rate (10s / 800us, 800us being minumum measurement rate of the sensor)
    // as well as one counter
    uint32_t measuerment_rate_1s = 1e4 / 8;
    if(light_sensor.setMeasurementRate(uint16_t(measuerment_rate_1s)) != 0) {
        Serial.println("Si1133: Could not set measurement rate"); 
    }
    if(light_sensor.setMeasurementCounter(Si1133::Counter::kIndex0, 30) != 0) {
        Serial.println("Si1133: Could not configure measurement counter"); 
    }

    Si1133::ChannelConfig chan_ir, chan_ambi, chan_uv;
    chan_ambi.channel = Si1133::Channel::kChannel0;
    chan_ambi.adc_mux = Si1133::ADCMux::kLargeWhite;
    chan_ambi.counter = Si1133::Counter::kIndex0;
    chan_ir.channel = Si1133::Channel::kChannel1;
    chan_ir.adc_mux = Si1133::ADCMux::kLargeIR;
    chan_ir.counter = Si1133::Counter::kIndex0;
    chan_uv.channel = Si1133::Channel::kChannel2;
    chan_uv.adc_mux = Si1133::ADCMux::kUV;
    chan_uv.counter = Si1133::Counter::kIndex0;

    if(light_sensor.configureChannel(chan_ambi) != 0) {
        Serial.println("Si1133: Failed to configure ambient light channel");
    }
    if(light_sensor.configureChannel(chan_ir) != 0) {
        Serial.println("Si1133: Failed to configure infrared light channel");
    }
    if(light_sensor.configureChannel(chan_uv) != 0) {
        Serial.println("Si1133: Failed to configure UV light channel");
    }
    if(light_sensor.enableChannels(uint8_t(Si1133::Channel::kChannel0) | uint8_t(Si1133::Channel::kChannel1) | uint8_t(Si1133::Channel::kChannel2)) != 0 ||
        light_sensor.enableInterrupts(uint8_t(Si1133::Channel::kChannel0) | uint8_t(Si1133::Channel::kChannel1) | uint8_t(Si1133::Channel::kChannel2)) != 0) {
        Serial.println("Si1133: Could not enable channels");
    }
    
    light_sensor.startMeasurements();
  } else {
    Serial.println("Si1133 not available");
  }
}

int last_run = 0;

void loop() {
  int now = millis();
  if(now >= last_run + 1000) {
    last_run = now;
    
    uint8_t light_avail = 0;
    light_sensor.readAvailableChannels(light_avail);
    if((light_avail & uint8_t(Si1133::Channel::kChannel0)) != 0) {
        Serial.print("Ambient value: ");
        Serial.println(light_sensor.getData(Si1133::Channel::kChannel0));
    }
    if((light_avail & uint8_t(Si1133::Channel::kChannel1)) != 0) {
        Serial.print("IR value: ");
        Serial.println(light_sensor.getData(Si1133::Channel::kChannel1));
    }
    if((light_avail & uint8_t(Si1133::Channel::kChannel2)) != 0) {
        Serial.print("UV value: ");
        Serial.println(light_sensor.getData(Si1133::Channel::kChannel2));
    }
  }
}
~~~~
