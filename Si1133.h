
/**********************************************************
 * Si1133.h                                          *
 *                                                        *
 * Author: Manuel Schoch <info@manuel-schoch.net>         *
 *                                                        *
 **********************************************************/

#ifndef SI1133_H
#define SI1133_H

#include <Arduino.h>

class Si1133 {
public:
  // Device addresses
  static constexpr uint8_t kSi1133AddrAPinLow = 0x52;
  static constexpr uint8_t kSi1133AddrAPinHigh = 0x55;
  // Resister addresses
  static constexpr uint8_t kSi1133RegPartId = 0x00;
  static constexpr uint8_t kSi1133RegHardwareId = 0x01;
  static constexpr uint8_t kSi1133RegRevision = 0x02;
  static constexpr uint8_t kSi1133RegHostIn3 = 0x07;
  static constexpr uint8_t kSi1133RegHostIn2 = 0x08;
  static constexpr uint8_t kSi1133RegHostIn1 = 0x09;
  static constexpr uint8_t kSi1133RegHostIn0 = 0x0A;
  static constexpr uint8_t kSi1133RegCommnad = 0x0B;
  static constexpr uint8_t kSi1133RegIRQEnable = 0x0F;
  static constexpr uint8_t kSi1133RegResponse1 = 0x10;
  static constexpr uint8_t kSi1133RegResponse0 = 0x11;
  static constexpr uint8_t kSi1133RegIRQStatus = 0x12;
  static constexpr uint8_t kSi1133RegHostOut0 = 0x13;
  static constexpr uint8_t kSi1133RegHostOut1 = 0x14;
  static constexpr uint8_t kSi1133RegHostOut2 = 0x15;
  static constexpr uint8_t kSi1133RegHostOut3 = 0x16;
  static constexpr uint8_t kSi1133RegHostOut4 = 0x17;
  static constexpr uint8_t kSi1133RegHostOut5 = 0x18;
  static constexpr uint8_t kSi1133RegHostOut6 = 0x19;
  static constexpr uint8_t kSi1133RegHostOut7 = 0x1A;
  static constexpr uint8_t kSi1133RegHostOut8 = 0x1B;
  // Commands
  static constexpr uint8_t kSi1133CmdResetCommandCounter = 0x0;
  static constexpr uint8_t kSi1133CmdResetSoftware = 0x1;
  static constexpr uint8_t kSi1133CmdForceMeasurement = 0x11;
  static constexpr uint8_t kSi1133CmdPauseMeasurements = 0x12;
  static constexpr uint8_t kSi1133CmdStartMeasurements = 0x13;
  static constexpr uint8_t kSi1133CmdParameterQuery = 0x40;
  static constexpr uint8_t kSi1133CmdParameterSet = 0x80;
  // Parameters
  static constexpr uint8_t kSi1133ParamI2CAddress = 0x0;
  static constexpr uint8_t kSi1133ParamChannelList = 0x1;
  static constexpr uint8_t kSi1133ParamADCCconfig0 = 0x2;
  static constexpr uint8_t kSi1133ParamADCSens0 = 0x3;
  static constexpr uint8_t kSi1133ParamADCPost0 = 0x4;
  static constexpr uint8_t kSi1133ParamMeasurementConfig0 = 0x5;
  static constexpr uint8_t kSi1133ParamADCConfig1 = (kSi1133ParamADCCconfig0 + 4);
  static constexpr uint8_t kSi1133ParamADCSens1 = (kSi1133ParamADCSens0 + 4);
  static constexpr uint8_t kSi1133ParamADCPost1 = (kSi1133ParamADCPost0 + 4);
  static constexpr uint8_t kSi1133ParamMeasurementConfig1 = (kSi1133ParamMeasurementConfig0 + 4);
  static constexpr uint8_t kSi1133ParamADCConfig2 = (kSi1133ParamADCCconfig0 + 8);
  static constexpr uint8_t kSi1133ParamADCSens2 = (kSi1133ParamADCSens0 + 8);
  static constexpr uint8_t kSi1133ParamADCPost2 = (kSi1133ParamADCPost0 + 8);
  static constexpr uint8_t kSi1133ParamMeasurementConfig2 = (kSi1133ParamMeasurementConfig0 + 8);
  static constexpr uint8_t kSi1133ParamADCConfig3 = (kSi1133ParamADCCconfig0 + 12);
  static constexpr uint8_t kSi1133ParamADCSens3 = (kSi1133ParamADCSens0 + 12);
  static constexpr uint8_t kSi1133ParamADCPost3 = (kSi1133ParamADCPost0 + 12);
  static constexpr uint8_t kSi1133ParamMeasurementConfig3 = (kSi1133ParamMeasurementConfig0 + 12);
  static constexpr uint8_t kSi1133ParamADCConfig4 = (kSi1133ParamADCCconfig0 + 16);
  static constexpr uint8_t kSi1133ParamADCSens4 = (kSi1133ParamADCSens0 + 16);
  static constexpr uint8_t kSi1133ParamADCPost4 = (kSi1133ParamADCPost0 + 16);
  static constexpr uint8_t kSi1133ParamMeasurementConfig4 = (kSi1133ParamMeasurementConfig0 + 16);
  static constexpr uint8_t kSi1133ParamADCConfig5 = (kSi1133ParamADCCconfig0 + 20);
  static constexpr uint8_t kSi1133ParamADCSens5 = (kSi1133ParamADCSens0 + 20);
  static constexpr uint8_t kSi1133ParamADCPost5 = (kSi1133ParamADCPost0 + 20);
  static constexpr uint8_t kSi1133ParamMeasurementConfig5 = (kSi1133ParamMeasurementConfig0 + 20);
  static constexpr uint8_t kSi1133ParamMeasurementRateHigh = 0x1A;
  static constexpr uint8_t kSi1133ParamMeasurementRateLow = 0x1B;
  static constexpr uint8_t kSi1133ParamMeasurementCount0 = 0x1C;
  static constexpr uint8_t kSi1133ParamMeasurementCount1 = kSi1133ParamMeasurementCount0 + 1;
  static constexpr uint8_t kSi1133ParamMeasurementCount2 = kSi1133ParamMeasurementCount0 + 2;
  static constexpr uint8_t kSi1133ParamThreshold0High = 0x25;
  static constexpr uint8_t kSi1133ParamThreshold0Low = 0x26;
  static constexpr uint8_t kSi1133ParamThreshold1High = kSi1133ParamThreshold0High + 2;
  static constexpr uint8_t kSi1133ParamThreshold1Low = kSi1133ParamThreshold0Low + 2;
  static constexpr uint8_t kSi1133ParamThreshold2High = kSi1133ParamThreshold0High + 4;
  static constexpr uint8_t kSi1133ParamThreshold2Low = kSi1133ParamThreshold0Low + 4;
  
  enum class OperationMode : uint8_t {
      kForcedMeasurement = 1,
      kPeriodicMesaturment
  };
  
  enum class Channel : uint8_t {
    kChannel0 = 0x1,
    kChannel1 = 0x2,
    kChannel2 = 0x4,
    kChannel3 = 0x8,
    kChannel4 = 0x10,
    kChannel5 = 0x20
  };
  
  /* ADCCONFIGx enums */
  /* - DECIM_RATE */
  enum class DecimationsRate : uint8_t {
    k48Dot8Us = (0x0 << 5),
    k97Dot6Us = (0x1 << 5),
    k195Us = (0x2 << 5),
    k24Dot4Us = (0x3 << 5)
  };

  /* - ADC_MUX */
  enum class ADCMux : uint8_t {
    kSmallIR = 0x0,
    kMediumIR = 0x1,
    kLargeIR = 0x2,
    kWhite = 0xB,
    kLargeWhite = 0xD,
    kUV = 0x18,
    kUVDeep = 0x19
  };

  /* ADCSENSx enums */
  /* - HSIG */
  enum class HSIG : uint8_t {
    kNormalRange = (0x0 << 7),
    kHighRange = (0x1 << 7)
  };

  /* - SW_GAIN */
  enum class SoftwareGain : uint8_t {
    k1Measurment = (0x0 << 4),
    k2Measurments = (0x1 << 4),
    k4Measurments = (0x2 << 4),
    k8Measurments = (0x3 << 4),
    k16Measurments = (0x4 << 4),
    k32Measurments = (0x5 << 4),
    k64Measurments = (0x6 << 4),
    k128Measurments = (0x7 << 4)
  };

  /* - HW_GAIN */
  enum class HardwareGain : uint8_t {
    k24Us = (0x0),
    k49Us = (0x1),
    k98Us = (0x2),
    k195Us = (0x3),
    k390Us = (0x4),
    k781Us = (0x5),
    k1562Us = (0x6),
    k3123Us = (0x7),
    k6246Us = (0x8),
    k12493Us = (0x9),
    k25Ms = (0xA),
    k50Ms = (0xB)
  };

  /* ADCPOSTx */
  /* - 24BIT_OUT */
  enum class OutputFormat : uint8_t {
    kUint16 = (0x0 << 6),
    kInt24 = (0x1 << 6)
  };

  /* - THRESH_EN */
  enum class Threshold : uint8_t {
    kOff = 0x0,
    kThreshold0 = 0x1,
    kThreshold1 = 0x2,
    kThreshold2 = 0x3
  };
  
  /* COUNTER_INDEX */
  enum class Counter : uint8_t {
    kNone = (0x0 << 6),
    kIndex0 = (0x1 << 6),
    kIndex1 = (0x2 << 6),
    kIndex2 = (0x3 << 6)
  };
  
  struct ChannelConfig {
    Channel channel;
    ADCMux adc_mux;
    DecimationsRate decimations_rate = DecimationsRate::k48Dot8Us;
    HSIG range = HSIG::kNormalRange;
    SoftwareGain software_gain = SoftwareGain::k2Measurments;
    HardwareGain hardware_gain = HardwareGain::k49Us;
    OutputFormat output_format = OutputFormat::kInt24;
    Counter counter=Counter::kNone;
    Threshold threshold=Threshold::kOff;
  };
  
  Si1133(uint8_t i2c_addr);
  
  /**
   * Set specified threshold register to specified value.
   * A thresold can be used to specify a value and interrup should be
   * generated when exceeded. To use a threshold, the channel config must be
   * pointing to the threshold field.
   * @return 0 on success, < 0 on error
   */
  int setThreshold(Threshold index, uint16_t value);
  
  /**
   * Set specified measurment counter. In automatic measuing mode, interval how often
   * the measurment of a channel is performed is determined by measurement rate *
   * measurement counter. To use a measuement counter, the channel config must be pointing
   * to the respective measurment counter field.
   * @return 0 on success, < 0 on error
   */
  int setMeasurementCounter(Counter index, uint8_t value);
  
  /**
   * Set global measurment rate in automatic measurment mode. The measurment rate is set
   * in 800 micosecons steps. I. e. filling the measurment rate field with 100, the measurment
   * counters per channel will be incremented every 80ms. As soon as the configured count for
   * a channel is reached, a measurment will be triggered (assuming the previous measurment
   * has been read).
   * @return 0 on success, < 0 on error
   */
  int setMeasurementRate(uint16_t measurement_rate);
  
  /**
   * Configure one measurment channel. The two main fileds to configure here are the
   * channle index and which sensor to use for this channel. E. g. you can configure
   * to use the large IR sensor for channel 0. Additionally the measurment counter field
   * and threshold filed to use for automatic mode can be configured here. All other
   * settings are mainly for fine tuning (gains, range, output format...)
   * @return 0 on success, < 0 on error
   */
  int configureChannel(const ChannelConfig& config);
  
  /**
   * Activate channels. To perform any measurement (in automatic and forced mode) the desired
   * channels must be activated first.
   * @return 0 on success, < 0 on error
   */
  int enableChannels(uint8_t channels);
  
  /**
   * Configure which channles should trigger the intterrupt in automatic mode. If the channel
   * has a threshold configured, the interrupt will only be triggered if the theshold is
   * exceeded. If not, the interrup will be triggered as soon as a new measuement has been
   * performed.
   * @return 0 on success, < 0 on error
   */
  int enableInterrupts(uint8_t channels);
  
  /**
   * Start a measurment in manual mode for all activated channles. The measurements take some
   * time. readAvailableChannels will only read data from channels where new data is available.
   * @return 0 on success, < 0 on error
   */
  int forceMeasurment();
  
  /**
   * Start measurments in automatic mode. This requires the channels to be set up, activated
   * and a measurment rate to be set.
   * @return 0 on success, < 0 on error
   */
  int startMeasurements();
  
  /**
   * Pause automatic measurment mode.
   * @return 0 on success, < 0 on error
   */
  int pauseMeasurements();
  
  /**
   * Read data from all channels where a new measurment has been finished. The Parameter
   * channles provides a bit field which channels have been read.
   * @return 0 on success, < 0 on error
   */
  int readAvailableChannels(uint8_t& channels);
  
  /**
   * Get the currnet data for a specific channel. The output is always converted into a
   * 32 bit signed integer, considering the output format.
   * @return The last measurment data from the specified channel. If the channel has never
   * been read by readAvailableChannels, the result is undefined.
   */
  int32_t getData(Channel channel);
  
  /**
   * Trigger a software reset of the chip.
   * @return 0 on success, < 0 on error
   */
  int reset();
  
  /**
   * Start the driver. This will check if the sensor responds on I2C.
   * @return 0 on success, < 0 on error
   */
  int begin();
  
  /**
   * Shutdown the driver.
   */
  void end();
  
protected:
  int writeRegister(uint8_t register_addr, uint8_t data);
  int executeCommand(uint8_t command, uint32_t timeout_us=800);
  int setParameter(uint8_t param, uint8_t val);
private:
  bool ready = false;
  
  uint8_t address = kSi1133AddrAPinLow;
  uint8_t chan_list = 0;
  uint8_t chan_24bits = 0;
  int32_t chan_data[6];
};


#endif /* SI1133 */
