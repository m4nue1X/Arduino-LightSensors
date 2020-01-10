#include "Si1133.h"
#include <WireUtil.h>
#include <TimeUtil.h>

Si1133::Si1133(uint8_t i2c_addr) : address(i2c_addr) { }

int Si1133::writeRegister(uint8_t register_addr, uint8_t data) {
  return WireUtil::writeRegister(address, register_addr, data);
}

int Si1133::executeCommand(uint8_t command, uint32_t timeout_us) {
  uint8_t cmd_ctr_prev = 0;
  uint8_t cmd_ctr_aft = 0;
  if(WireUtil::readRegister(address, kSi1133RegResponse0, cmd_ctr_prev) != 0 ||
     WireUtil::writeRegister(address, kSi1133RegCommnad, command) != 0) {
    return -1;
  }
  uint32_t start_time = micros();
  while(WireUtil::readRegister(address, kSi1133RegResponse0, cmd_ctr_aft) == 0) {
    if((cmd_ctr_aft & 0x10) != 0) { // we encountered an error
      return -1;
    }
    if((((cmd_ctr_aft & 0xF) - (cmd_ctr_prev & 0xF)) & 0xF) == 1) {
      // the command has finished executing
      return 0;
    }
    if(TimeUtil::deltaMicroseconds(start_time, micros()) >= timeout_us) {
      return -2;
    }
    delayMicroseconds(8);
  }
  return -1;
}

int Si1133::setParameter(uint8_t param, uint8_t val) {
  return (writeRegister(kSi1133RegHostIn0, val) != 0 || executeCommand(kSi1133CmdParameterSet | param) != 0) ? -1 : 0;
}

int Si1133::reset() {
  if(!ready) {
    return -1;
  }
  // NOTE: we are not using executeCommand here because the counter will be reset
  int r = writeRegister(kSi1133RegCommnad, kSi1133CmdResetSoftware);
  delayMicroseconds(100); // the reset takes a bit
  return r;
}

int Si1133::setThreshold(Threshold index, uint16_t value) {
  if(!ready) {
    return -1;
  }

  uint8_t threshold_offset;
  switch(index) {
    case Threshold::kThreshold0: threshold_offset = 0; break;
    case Threshold::kThreshold1: threshold_offset = 2; break;
    case Threshold::kThreshold2: threshold_offset = 4; break;
    default: return -3;
  }
  
  if(setParameter(kSi1133ParamThreshold0High + threshold_offset, (value >> 8) & 0xFF) != 0 ||
     setParameter(kSi1133ParamThreshold0Low + threshold_offset, value & 0xFF) != 0) {
    return -2;
  }
  
  return 0;
}

int Si1133::setMeasurementCounter(Counter index, uint8_t value) {
  if(!ready) {
    return -1;
  }
  
  uint8_t measurement_counter_offset;
  switch(index) {
    case Counter::kIndex0: measurement_counter_offset = 0; break;
    case Counter::kIndex1: measurement_counter_offset = 1; break;
    case Counter::kIndex2: measurement_counter_offset = 2; break;
    default: return -3;
  }
  
  if(setParameter(kSi1133ParamMeasurementCount0 + measurement_counter_offset, value) != 0) {
    return -2;
  }
  
  return 0;
}

int Si1133::setMeasurementRate(uint16_t measurement_rate) {
  if(!ready) {
    return -1;
  }
    
  if(setParameter(kSi1133ParamMeasurementRateHigh, (measurement_rate >> 8) & 0xFF) != 0 || 
     setParameter(kSi1133ParamMeasurementRateLow, measurement_rate & 0xFF) != 0) {
    return -2;
  }
}

int Si1133::configureChannel(const ChannelConfig& config) {
  if(!ready) {
    return -1;
  }
  
  uint8_t channel_offset;
  switch(config.channel) {
    case Channel::kChannel1: channel_offset = 4; break;
    case Channel::kChannel2: channel_offset = 8; break;
    case Channel::kChannel3: channel_offset = 12; break;
    case Channel::kChannel4: channel_offset = 16; break;
    case Channel::kChannel5: channel_offset = 20; break;
    default: channel_offset = 0; break; // i. e. channel 0
  }
  
  uint8_t config_params[4], config_data[4];
  config_data[0] = uint8_t(config.decimations_rate) | uint8_t(config.adc_mux);
  config_params[0] = kSi1133ParamADCCconfig0 + channel_offset;
  config_data[1] = uint8_t(config.range) | uint8_t(config.software_gain) | uint8_t(config.hardware_gain);
  config_params[1] = kSi1133ParamADCSens0 + channel_offset;
  config_data[2] = uint8_t(config.output_format) | uint8_t(config.threshold);
  config_params[2] = kSi1133ParamADCPost0 + channel_offset;
  config_data[3] = uint8_t(config.counter);
  config_params[3] = kSi1133ParamMeasurementConfig0 + channel_offset;
  
  for(int i=0; i<4; ++i) {
    if(setParameter(config_params[i], config_data[i]) != 0) {
      return -2;
    }
  }
  
  // store if this channel is configured to deliver the output in 24bit mode to
  // be able to figure out the layout of the HostOut registers while reading
  if(config.output_format == OutputFormat::kInt24) {
    chan_24bits |= uint8_t(config.channel);
  } else {
    chan_24bits &= ~(uint8_t(config.channel));
  }
  
  return 0;
}

int Si1133::enableChannels(uint8_t channels) {
  if(!ready) {
    return -1;
  }
  
  if(setParameter(kSi1133ParamChannelList, channels & 0x3f) != 0) {
    return -2;
  }
  chan_list = channels;
  return 0;
}

int Si1133::enableInterrupts(uint8_t channels) {
  if(!ready) {
    return -1;
  }
  return writeRegister(kSi1133RegIRQEnable, channels & 0x3f);
}

int Si1133::forceMeasurment() {
  if(!ready) {
    return -1;
  }
  return executeCommand(kSi1133CmdForceMeasurement);
}

int Si1133::startMeasurements() {
  if(!ready) {
    return -1;
  }
  return executeCommand(kSi1133CmdStartMeasurements);
}

int Si1133::pauseMeasurements() {
  if(!ready) {
    return -1;
  }
  return executeCommand(kSi1133CmdPauseMeasurements);
}

int Si1133::readAvailableChannels(uint8_t& channels) {
  if(!ready) {
    return -1;
  }
    
  // check which data is available
  if(WireUtil::readRegister(address, kSi1133RegIRQStatus, channels) != 0) {
    return -1;
  }
  
  // loop over all channels and read the available ones
  uint8_t out_reg = uint8_t(kSi1133RegHostOut0);
  for(int i=0; i<6; ++i) {
    uint8_t cur_channel = 0x1 << i;
    // check if the current channel is activated
    if((cur_channel & chan_list) != 0) {
      // determine the output size
      uint8_t cur_channel_n_out = ((chan_24bits & cur_channel) != 0) ? 3 : 2;
      // check if we have data for the current channel
      if((cur_channel & channels) != 0) {
        // we have new data for the currnet channel -> read it
        uint32_t tmp_res = 0;
        if(WireUtil::readRegister(address, out_reg, cur_channel_n_out, tmp_res, false) != 0) {
          return -1;
        }
        // convert & store the data
        if(cur_channel_n_out == 3) { // this is a 24 bit signed value
          chan_data[i] = int32_t(tmp_res) << 8 >> 8; // bring the signed bit to the front
        } else {
          chan_data[i] = int32_t(tmp_res); // tmp_res is 16bit unsigned, so there won't be an issue with the sign
        }
      }
      out_reg += cur_channel_n_out;
    }
    cur_channel <<= 1;
  }
  return 0;
}

int32_t Si1133::getData(Channel channel) {
  uint8_t channel_index = log2(uint8_t(channel));
  if(channel_index >= 0 && channel_index < 6) {
    return chan_data[channel_index];
  }
  return 0x80000000; // return invalid value
}

int Si1133::begin() {    
  end();
  
  uint8_t res = 0;
  if(WireUtil::readRegister(address, kSi1133RegPartId, res, 800) < 0) {
    return -1;
  } else if(res !=  0x33) {
    return -2;
  } // else -> found sensor
  
  ready = true;
  // reset (IMPORTANT: set ready to true first)
  reset();
  return 0;
}

void Si1133::end() {
  ready = false;
}
