
#include "driver_stim300.h"
#include <chrono>
#include <iostream>
#include <thread>

DriverStim300::DriverStim300(SerialDriver &serial_driver,
                             DatagramIdentifier datagram_id,
                             GyroOutputUnit gyro_output_unit,
                             AccOutputUnit acc_output_unit,
                             InclOutputUnit incl_output_unit,
                             AccRange acc_range, SampleFreq freq)
    : serial_driver_(serial_driver),
      datagram_parser_(datagram_id, gyro_output_unit, acc_output_unit,
                       incl_output_unit, acc_range),
      datagram_id_(datagramIdentifierToRaw(datagram_id)),
      crc_dummy_bytes_(numberOfPaddingBytes(datagram_id)),
      datagram_size_(calculateDatagramSize(datagram_id)), sensor_config_{
                                                              '0',
                                                              0,
                                                              freq,
                                                              datagram_id,
                                                              false,
                                                              gyro_output_unit,
                                                              acc_output_unit,
                                                              incl_output_unit,
                                                              acc_range} {}

DriverStim300::DriverStim300(SerialDriver &serial_driver)
    : DriverStim300(serial_driver, DatagramIdentifier::CONFIGURATION,
                    GyroOutputUnit::AVERAGE_ANGULAR_RATE,
                    AccOutputUnit::AVERAGE_ACCELERATION,
                    InclOutputUnit::AVERAGE_ACCELERATION, AccRange::G5,
                    SampleFreq::S125) {}
double DriverStim300::getAccX() const noexcept { return sensor_data_.acc[0]; }
double DriverStim300::getAccY() const noexcept { return sensor_data_.acc[1]; }
double DriverStim300::getAccZ() const noexcept { return sensor_data_.acc[2]; }
double DriverStim300::getGyroX() const noexcept { return sensor_data_.gyro[0]; }
double DriverStim300::getGyroY() const noexcept { return sensor_data_.gyro[1]; }
double DriverStim300::getGyroZ() const noexcept { return sensor_data_.gyro[2]; }
double DriverStim300::getIncX() const noexcept { return sensor_data_.incl[0]; }
double DriverStim300::getIncY() const noexcept { return sensor_data_.incl[1]; }
double DriverStim300::getIncZ() const noexcept { return sensor_data_.incl[2]; }
uint16_t DriverStim300::getSampleRate() const noexcept {
  return stim_const::sampleFreq2int(sensor_config_.sample_freq);
}

uint16_t DriverStim300::getLatency_us() const noexcept {
  return sensor_data_.latency_us;
}
bool DriverStim300::isSensorStatusGood() const noexcept {
  return sensor_status_ == 0;
}
uint8_t DriverStim300::getInternalMeasurementCounter() const noexcept {
  return sensor_data_.counter;
}

double DriverStim300::getAverageTemp() const noexcept {
  double sum{0};
  uint8_t count{0};

  return count != 0 ? sum / count : std::numeric_limits<double>::quiet_NaN();
}


Stim300Status DriverStim300::readDataStream() {
  // ROLLING WINDOW SYNCHRONIZATION - Fixes false sync vulnerability
  // Read all available bytes into sync buffer first (no immediate processing)
  uint8_t byte;
  while (serial_driver_.readByte(byte)) {
    sync_buffer_.push_back(byte);
    
    // Prevent buffer from growing too large
    if (sync_buffer_.size() > 2048) {
      sync_buffer_.erase(sync_buffer_.begin(), sync_buffer_.begin() + 1024);
      std::cerr << "STIM300: Sync buffer trimmed to prevent overflow" << std::endl;
    }
  }

  // If we don't have enough data for a complete packet, wait for more
  if (sync_buffer_.size() < datagram_size_) {
    return Stim300Status::NORMAL;
  }

  // ROLLING WINDOW SCAN - Check every possible sync position with peek-ahead
  for (size_t offset = 0; offset <= sync_buffer_.size() - datagram_size_; offset++) {
    
    uint8_t potential_header = sync_buffer_[offset];
    
    // Check if this could be a valid header byte (all possible formats)
    if (potential_header == datagram_id_ ||
        potential_header == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION) ||
        potential_header == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION_CRLF)) {
      
      // Handle CRLF terminated packets (config packets)
      if (potential_header == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION_CRLF) ||
          (sensor_config_.normal_datagram_CRLF && potential_header == datagram_id_ && potential_header != 0xBC)) {
        
        // For CRLF packets, look for CR LF terminator
        size_t search_end = std::min(offset + datagram_size_ + 10, sync_buffer_.size());
        for (size_t i = offset + 1; i < search_end - 1; i++) {
          if (sync_buffer_[i] == 0x0D && sync_buffer_[i + 1] == 0x0A) {
            // Found CRLF terminator
            size_t packet_length = i - offset + 2;
            
            // Copy CRLF packet to buffer for verification
            buffer_.clear();
            buffer_.assign(sync_buffer_.begin() + offset, 
                          sync_buffer_.begin() + offset + packet_length);
            
            // Verify CRC (excluding CRLF)
            if (verifyChecksum(buffer_.cbegin(), buffer_.cend() - 2, crc_dummy_bytes_)) {
              std::cerr << "STIM300: Valid CRLF packet found at offset " << offset << std::endl;
              
              // Update format if needed
              if (potential_header != datagram_id_) {
                if (potential_header == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION_CRLF)) {
                  setDatagramFormat(DatagramIdentifier::CONFIGURATION_CRLF);
                } else if (potential_header == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION)) {
                  setDatagramFormat(DatagramIdentifier::CONFIGURATION);
                }
              }
              
              // Consume packet from sync buffer
              sync_buffer_.erase(sync_buffer_.begin(), sync_buffer_.begin() + offset + packet_length);
              n_checked_bytes = 0;
              
              // Process config packet
              stim_300::SensorConfig sensor_config = datagram_parser_.parseConfig(buffer_.cbegin());
              Stim300Status status = (sensor_config != sensor_config_) ? Stim300Status::CONFIG_CHANGED : Stim300Status::NORMAL;
              sensor_config_ = sensor_config;
              setDatagramFormat(sensor_config_.datagram_id);
              datagram_parser_.setDataParameters(sensor_config_);
              read_config_from_sensor_ = false;
              return status;
            }
            break; // Try next offset if CRC failed
          }
        }
        continue; // Try next offset if no CRLF found
      }
      
      // Handle normal CRC terminated packets (measurement data)
      if (offset + datagram_size_ <= sync_buffer_.size()) {
        
        // PEEK-AHEAD: Copy potential packet WITHOUT consuming data yet
        buffer_.clear();
        buffer_.assign(sync_buffer_.begin() + offset, 
                      sync_buffer_.begin() + offset + datagram_size_);
        
        // Verify CRC before committing to this sync point
        if (verifyChecksum(buffer_.cbegin(), buffer_.cend(), crc_dummy_bytes_)) { 
          // std::cerr << "STIM300: CRC passed for header 0x" << std::hex << (int)potential_header << std::dec << std::endl;
          
          // std::cerr << "STIM300: Valid packet found! Header=0x" << std::hex << (int)potential_header << std::dec << " at offset " << offset << std::endl;
          
          // Valid packet found! Update format if needed
          if (potential_header != datagram_id_) {
            if (potential_header == datagramIdentifierToRaw(DatagramIdentifier::CONFIGURATION)) {
              setDatagramFormat(DatagramIdentifier::CONFIGURATION);
            }
          }
          
          // NOW consume the data (all bytes up to and including this packet)
          sync_buffer_.erase(sync_buffer_.begin(), sync_buffer_.begin() + offset + datagram_size_);
          
          // Reset sync error counter
          n_checked_bytes = 0;
          
          // Check if this is a CONFIG packet or MEASUREMENT packet
          if (potential_header == 0xBC || potential_header == 0xBD) {
            // Process CONFIG packet
            std::cerr << "STIM300: Processing config packet..." << std::endl;
            stim_300::SensorConfig sensor_config = datagram_parser_.parseConfig(buffer_.cbegin());
            Stim300Status status = (sensor_config != sensor_config_) ? Stim300Status::CONFIG_CHANGED : Stim300Status::NORMAL;
            sensor_config_ = sensor_config;
            setDatagramFormat(sensor_config_.datagram_id);  // Switch to detected format!
            datagram_parser_.setDataParameters(sensor_config_);
            read_config_from_sensor_ = false;  // Stop requesting config
            std::cerr << "STIM300: Configuration received! Switching to format: 0x" 
                      << std::hex << (int)datagram_id_ << std::dec << std::endl;
            return status;
          } else {
            // Process MEASUREMENT packet
            sensor_status_ = datagram_parser_.parseData(buffer_.cbegin(), sensor_data_);
            
            // Return appropriate status based on sensor status bits
            if (sensor_status_ == 0)
              return Stim300Status::NEW_MEASURMENT;
            else if (sensor_status_ & (1u << 6u))
              return Stim300Status::STARTING_SENSOR;
            else if (sensor_status_ & (1u << 7u))
              return Stim300Status::SYSTEM_INTEGRITY_ERROR;
            else if (sensor_status_ & (1u << 5u))
              return Stim300Status::OUTSIDE_OPERATING_CONDITIONS;
            else if (sensor_status_ & (1u << 4u))
              return Stim300Status::OVERLOAD;
            else if (sensor_status_ & (1u << 3u))
              return Stim300Status::ERROR_IN_MEASUREMENT_CHANNEL;
            else
              return Stim300Status::ERROR;
          }
        }
        std::cerr << "STIM300: CRC FAILED for header 0x" << std::hex << (int)potential_header << std::dec << std::endl; // If CRC failed, continue to next offset (don't consume any data)
      }
    }
  }
  
  // No valid packet found in current buffer
  if (!sync_buffer_.empty()) {
    // Remove only the oldest byte to make progress (preserve potential true headers)
    sync_buffer_.erase(sync_buffer_.begin());
    n_checked_bytes++;
    
    // If we've discarded too many bytes without finding sync, request config
    if (n_checked_bytes > 500) {
      std::cerr << "STIM300: Lost synchronization after " << n_checked_bytes 
                << " bytes, requesting config" << std::endl;
      if (read_config_from_sensor_) {
        askForConfigDatagram();
      }
      n_checked_bytes = 0;
    }
  }
  
  return Stim300Status::NORMAL;
}

void DriverStim300::askForConfigDatagram() {
  serial_driver_.writeByte('C');
  serial_driver_.writeByte('\r');
}


Stim300Status DriverStim300::update() noexcept {
  switch (mode_) {
  case Mode::Init:
    // Initialize serial and request sensor configuration
    serial_driver_.flush();
    if (read_config_from_sensor_) {
      std::cerr << "STIM300: Requesting sensor configuration..." << std::endl;
      askForConfigDatagram();
      mode_ = Mode::WaitingForConfig;
    } else {
      // Config already available, go straight to normal operation
      mode_ = Mode::Normal; std::cerr << "STIM300: Switched to Normal mode for data packets" << std::endl;
    }
    return Stim300Status::NORMAL;
    
  case Mode::WaitingForConfig:
    // Process incoming data looking specifically for config response
    {
      Stim300Status status = readDataStream();
      if (status == Stim300Status::CONFIG_CHANGED || status == Stim300Status::NORMAL) {
        // Check if we received and processed a config packet
        if (!read_config_from_sensor_) {
          std::cerr << "STIM300: Configuration received! Switching to format: 0x" 
                    << std::hex << (int)datagram_id_ << std::dec << std::endl;
          mode_ = Mode::Normal; std::cerr << "STIM300: Switched to Normal mode for data packets" << std::endl;
        }
      }
      return status;
    }
    
  case Mode::Normal:
    return readDataStream();
    
  case Mode::Service:
    // std::string s(buffer_.begin(), buffer_.end());
    // std::cout << s << "\n";
    mode_ = Mode::Normal; std::cerr << "STIM300: Switched to Normal mode for data packets" << std::endl;
    return Stim300Status::NORMAL;
  }
  return Stim300Status::NORMAL;
}


namespace {
// Reads ASCII service mode output until `idle` has passed without a new byte
// (after at least one byte), or `total` has passed.
std::string readResponse(SerialDriver &serial, std::chrono::milliseconds total,
                         std::chrono::milliseconds idle) {
  using Clock = std::chrono::steady_clock;
  std::string response;
  const auto start = Clock::now();
  auto last_byte = start;
  uint8_t byte;
  while (Clock::now() - start < total) {
    if (serial.readByte(byte)) {
      response.push_back(static_cast<char>(byte));
      last_byte = Clock::now();
    } else {
      if (!response.empty() && Clock::now() - last_byte > idle) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return response;
}

void writeCommand(SerialDriver &serial, const std::string &command) {
  for (char c : command) {
    serial.writeByte(static_cast<uint8_t>(c));
  }
  serial.writeByte('\r');
}
} // namespace

bool DriverStim300::configureSensor(const SensorSettings &settings,
                                    std::string &log) {
  using std::chrono::milliseconds;
  int rate_index{-1};
  switch (settings.sample_rate_hz) {
  case 0:
    break;
  case 125:
    rate_index = 0;
    break;
  case 250:
    rate_index = 1;
    break;
  case 500:
    rate_index = 2;
    break;
  case 1000:
    rate_index = 3;
    break;
  case 2000:
    rate_index = 4;
    break;
  default:
    log = "Unsupported sample rate, use 125, 250, 500, 1000 or 2000 Hz";
    return false;
  }
  int filter_index{-1};
  switch (settings.lp_filter_hz) {
  case 0:
    break;
  case 16:
    filter_index = 0;
    break;
  case 33:
    filter_index = 1;
    break;
  case 66:
    filter_index = 2;
    break;
  case 131:
    filter_index = 3;
    break;
  case 262:
    filter_index = 4;
    break;
  default:
    log = "Unsupported LP filter, use 16, 33, 66, 131 or 262 Hz";
    return false;
  }
  if (rate_index < 0 && filter_index < 0) {
    log = "Nothing to configure";
    return true;
  }

  // The sensor completes the datagram in flight before it enters Service Mode,
  // so the reply is preceded by binary measurement data.
  serial_driver_.flush();
  writeCommand(serial_driver_, "SERVICEMODE");
  std::string response =
      readResponse(serial_driver_, milliseconds(2000), milliseconds(300));
  log = response;
  if (response.find("SERIAL NUMBER") == std::string::npos) {
    log = "Sensor did not enter Service Mode";
    return false;
  }

  // Sends a command and checks that the reply contains `expected` and no error.
  auto accepted = [&](const std::string &command, const std::string &expected) {
    writeCommand(serial_driver_, command);
    response =
        readResponse(serial_driver_, milliseconds(1000), milliseconds(200));
    log += response;
    return response.find(expected) != std::string::npos &&
           response.find("E00") == std::string::npos;
  };

  bool ok = true;
  if (ok && rate_index >= 0) {
    ok = accepted("m " + std::to_string(rate_index), "SAMPLE RATE");
  }
  if (ok && filter_index >= 0) {
    ok = accepted("f " + std::to_string(filter_index), "LP FILTER");
  }

  if (ok && settings.save_to_flash) {
    writeCommand(serial_driver_, "s");
    response = readResponse(serial_driver_, milliseconds(1000), milliseconds(200));
    log += response;
    if (response.find("CONFIRM SAVE") != std::string::npos) {
      writeCommand(serial_driver_, "Y");
      response =
          readResponse(serial_driver_, milliseconds(3000), milliseconds(300));
      log += response;
    }
    ok = response.find("SUCCESSFULLY STORED") != std::string::npos;
  }

  // N returns to Normal Mode at once, without asking about unsaved changes.
  writeCommand(serial_driver_, "x N");
  response = readResponse(serial_driver_, milliseconds(1000), milliseconds(200));
  log += response;

  // Start over: drop the service mode text, then request the configuration.
  serial_driver_.flush();
  sync_buffer_.clear();
  n_checked_bytes = 0;
  read_config_from_sensor_ = true;
  mode_ = Mode::Init;
  return ok;
}

bool DriverStim300::setDatagramFormat(DatagramIdentifier id) {
  datagram_id_ = datagramIdentifierToRaw(id);
  datagram_size_ = calculateDatagramSize(id);
  crc_dummy_bytes_ = numberOfPaddingBytes(id);
  return true;
}

bool DriverStim300::verifyChecksum(
    const std::vector<uint8_t>::const_iterator &begin,
    const std::vector<uint8_t>::const_iterator &end,
    const uint8_t &crc_dummy_bytes) {
  uint32_t crc = stim_300::DatagramParser::parseCRC(end - sizeof(uint32_t));

  boost::crc_basic<32> crc_32_calculator(0x04C11DB7, 0xFFFFFFFF, 0x00, false,
                                         false);
  uint8_t buffer_CRC[end - begin - sizeof(uint32_t) + crc_dummy_bytes];
  std::copy(begin, end - sizeof(uint32_t) + crc_dummy_bytes, buffer_CRC);

  /** Fill the Dummy bytes with 0x00. There are at the end of the buffer **/
  for (size_t i = 0; i < crc_dummy_bytes; ++i)
    buffer_CRC[sizeof(buffer_CRC) - (1 + i)] = 0x00;

  crc_32_calculator.process_bytes(buffer_CRC, sizeof(buffer_CRC));
  auto crc_calck = crc_32_calculator.checksum();
  return crc_calck == crc;
}

std::string DriverStim300::printSensorConfig() const noexcept {
  std::stringstream ss;
  ss << "\nFirmware: " << sensor_config_.revision
     << std::to_string(sensor_config_.firmvare_version) << std::endl;
  ss << "Sample_freq: ";
  switch (sensor_config_.sample_freq) {
  case SampleFreq::S125:
    ss << "125 Hz";
    break;
  case SampleFreq::S250:
    ss << "250 Hz";
    break;
  case SampleFreq::S500:
    ss << "500 Hz";
    break;
  case SampleFreq::S1000:
    ss << "1000 Hz";
    break;
  case SampleFreq::S2000:
    ss << "2000 Hz";
    break;
  case SampleFreq::TRG:
    ss << "External Trigger";
    break;
  }
  ss << std::endl;
  auto included_sensors = isIncluded(sensor_config_.datagram_id);
  ss << "Gyro:\t\t\t" << included_sensors[SensorIndx::GYRO] << std::endl;
  ss << "Accelerometer:\t" << included_sensors[SensorIndx::ACC] << std::endl;
  ss << "Inlcinometer:\t" << included_sensors[SensorIndx::INCL] << std::endl;
  ss << "Temprature:\t\t" << included_sensors[SensorIndx::TEMP] << std::endl;
  ss << "Aux:\t\t\t" << included_sensors[SensorIndx::AUX] << std::endl;
  ss << "Normal Datagram termination: " << sensor_config_.normal_datagram_CRLF
     << std::endl;
  ss << "Gyro output:\t\t\t";
  switch (sensor_config_.gyro_output_unit) {
  case GyroOutputUnit::ANGULAR_RATE:
    ss << "Angular rate";
    break;
  case GyroOutputUnit::AVERAGE_ANGULAR_RATE:
    ss << "Average angular rate";
    break;
  case GyroOutputUnit::INCREMENTAL_ANGLE:
    ss << "Incremental angle";
    break;
  case GyroOutputUnit::INTEGRATED_ANGLE:
    ss << "Integrated angle";
    break;
  }
  ss << std::endl;
  ss << "Accelerometer output:\t";
  switch (sensor_config_.acc_output_unit) {
  case AccOutputUnit::ACCELERATION:
    ss << "Acceleration";
    break;
  case AccOutputUnit::AVERAGE_ACCELERATION:
    ss << "Average acceleration";
    break;
  case AccOutputUnit::INCREMENTAL_VELOCITY:
    ss << "Incremental velocity";
    break;
  case AccOutputUnit::INTEGRATED_VELOCITY:
    ss << "Integrated velocity";
    break;
  }
  ss << std::endl;
  ss << "Inclinometer output:\t";
  switch (sensor_config_.incl_output_unit) {
  case InclOutputUnit::ACCELERATION:
    ss << "Acceleration";
    break;
  case InclOutputUnit::AVERAGE_ACCELERATION:
    ss << "Average acceleration";
    break;
  case InclOutputUnit::INCREMENTAL_VELOCITY:
    ss << "Incremental velocity";
    break;
  case InclOutputUnit::INTEGRATED_VELOCITY:
    ss << "Integrated velocity";
    break;
  }
  ss << std::endl;
  ss << "Acceleration range: ";
  switch (sensor_config_.acc_range) {
  case AccRange::G2:
    ss << "2";
    break;
  case AccRange::G5:
    ss << "5";
    break;
  case AccRange::G10:
    ss << "10";
    break;
  case AccRange::G30:
    ss << "30";
    break;
  case AccRange::G80:
    ss << "80";
    break;
  }
  ss << " g." << std::endl;
  return ss.str();
}
