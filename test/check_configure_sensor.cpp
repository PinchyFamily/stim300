#include "../include/driver_stim300.h"
#include "mock_serial_driver.h"
#include "gmock/gmock.h"

#include <deque>
#include <string>
#include <vector>

using ::testing::_;
using ::testing::ElementsAre;
using ::testing::Invoke;
using ::testing::IsEmpty;
using ::testing::NiceMock;
using ::testing::Return;

namespace {

// Behaves like the STIM300 service mode text interface (datasheet TS1524,
// section 10). Every command line written to it is recorded.
class FakeSensor {
public:
  bool enters_service_mode{true};
  bool rejects_rate{false};
  bool rejects_filter{false};
  bool rejects_save{false};
  std::vector<std::string> commands;

  bool readByte(uint8_t &byte) {
    if (out_.empty()) {
      return false;
    }
    byte = out_.front();
    out_.pop_front();
    return true;
  }

  bool writeByte(uint8_t byte) {
    if (byte != '\r') {
      pending_.push_back(static_cast<char>(byte));
      return true;
    }
    handle(pending_);
    pending_.clear();
    return true;
  }

  bool flush() {
    out_.clear();
    return true;
  }

private:
  void send(const std::string &text) {
    out_.insert(out_.end(), text.begin(), text.end());
  }

  void handle(const std::string &command) {
    commands.push_back(command);
    if (command == "SERVICEMODE") {
      if (enters_service_mode) {
        // The datagram in flight is completed first. Binary data may look like
        // anything, including a prompt.
        send(std::string("\x93\x3e\xbc\x00\x3e", 5));
        send("SERIAL NUMBER = N25581740269498\r\nPRODUCT = STIM300\r\n\r\n>");
      }
    } else if (command.rfind("m ", 0) == 0) {
      send(rejects_rate ? "E003 INVALID PARAMETER\r\n\r\n>"
                        : "SAMPLE RATE [samples/s] = " + command.substr(2) +
                              "\r\n\r\n>");
    } else if (command.rfind("f ", 0) == 0) {
      send(rejects_filter
               ? "E003 INVALID PARAMETER\r\n\r\n>"
               : "GYRO LP FILTER -3dB FREQUENCY, X-AXIS [Hz] = 66\r\n\r\n>");
    } else if (command == "s") {
      send("SYSTEM PARAMETERS WILL BE PERMANENTLY CHANGED. CONFIRM SAVE(Y/N): ");
    } else if (command == "Y") {
      send(rejects_save ? "E005 ERROR DURING SAVE\r\n\r\n>"
                        : "SYSTEM PARAMETERS SUCCESSFULLY STORED IN FLASH.\r\n"
                          "NUMBER OF SAVES = 26\r\n\r\n>");
    } else if (command == "x N") {
      send("SYSTEM RETURNING TO NORMAL MODE.\r\n");
    }
  }

  std::string pending_;
  std::deque<uint8_t> out_;
};

class ConfigureSensorTest : public ::testing::Test {
protected:
  ConfigureSensorTest() : driver_(serial_) {
    ON_CALL(serial_, readByte(_))
        .WillByDefault(Invoke(&sensor_, &FakeSensor::readByte));
    ON_CALL(serial_, writeByte(_))
        .WillByDefault(Invoke(&sensor_, &FakeSensor::writeByte));
    ON_CALL(serial_, flush()).WillByDefault(Invoke(&sensor_, &FakeSensor::flush));
  }

  NiceMock<MockStim300SerialDriver> serial_;
  FakeSensor sensor_;
  DriverStim300 driver_;
  std::string log_;
};

} // namespace

TEST_F(ConfigureSensorTest, SetsSampleRate) {
  DriverStim300::SensorSettings settings;
  settings.sample_rate_hz = 250;
  EXPECT_TRUE(driver_.configureSensor(settings, log_));
  EXPECT_THAT(sensor_.commands, ElementsAre("SERVICEMODE", "m 1", "x N"));
}

TEST_F(ConfigureSensorTest, MapsEverySampleRate) {
  const std::vector<std::pair<uint16_t, std::string>> expected{
      {125, "m 0"}, {250, "m 1"}, {500, "m 2"}, {1000, "m 3"}, {2000, "m 4"}};
  for (const auto &entry : expected) {
    sensor_.commands.clear();
    DriverStim300::SensorSettings settings;
    settings.sample_rate_hz = entry.first;
    EXPECT_TRUE(driver_.configureSensor(settings, log_)) << entry.first;
    EXPECT_EQ(entry.second, sensor_.commands.at(1)) << entry.first;
  }
}

TEST_F(ConfigureSensorTest, MapsEveryLpFilter) {
  const std::vector<std::pair<uint16_t, std::string>> expected{
      {16, "f 0"}, {33, "f 1"}, {66, "f 2"}, {131, "f 3"}, {262, "f 4"}};
  for (const auto &entry : expected) {
    sensor_.commands.clear();
    DriverStim300::SensorSettings settings;
    settings.lp_filter_hz = entry.first;
    EXPECT_TRUE(driver_.configureSensor(settings, log_)) << entry.first;
    EXPECT_EQ(entry.second, sensor_.commands.at(1)) << entry.first;
  }
}

TEST_F(ConfigureSensorTest, SetsSampleRateAndLpFilter) {
  DriverStim300::SensorSettings settings;
  settings.sample_rate_hz = 250;
  settings.lp_filter_hz = 66;
  EXPECT_TRUE(driver_.configureSensor(settings, log_));
  EXPECT_THAT(sensor_.commands,
              ElementsAre("SERVICEMODE", "m 1", "f 2", "x N"));
}

TEST_F(ConfigureSensorTest, DoesNotSaveToFlashByDefault) {
  DriverStim300::SensorSettings settings;
  settings.sample_rate_hz = 250;
  settings.lp_filter_hz = 66;
  EXPECT_TRUE(driver_.configureSensor(settings, log_));
  EXPECT_THAT(sensor_.commands, ::testing::Not(::testing::Contains("s")));
}

TEST_F(ConfigureSensorTest, SavesToFlashWhenAsked) {
  DriverStim300::SensorSettings settings;
  settings.sample_rate_hz = 250;
  settings.save_to_flash = true;
  EXPECT_TRUE(driver_.configureSensor(settings, log_));
  EXPECT_THAT(sensor_.commands,
              ElementsAre("SERVICEMODE", "m 1", "s", "Y", "x N"));
}

TEST_F(ConfigureSensorTest, FailsWhenFlashSaveFails) {
  sensor_.rejects_save = true;
  DriverStim300::SensorSettings settings;
  settings.sample_rate_hz = 250;
  settings.save_to_flash = true;
  EXPECT_FALSE(driver_.configureSensor(settings, log_));
  EXPECT_EQ("x N", sensor_.commands.back());
}

TEST_F(ConfigureSensorTest, FailsWhenSensorDoesNotEnterServiceMode) {
  sensor_.enters_service_mode = false;
  DriverStim300::SensorSettings settings;
  settings.sample_rate_hz = 250;
  EXPECT_FALSE(driver_.configureSensor(settings, log_));
  EXPECT_THAT(sensor_.commands, ElementsAre("SERVICEMODE"));
  EXPECT_NE(std::string::npos, log_.find("did not enter Service Mode"));
}

TEST_F(ConfigureSensorTest, RejectedRateSkipsLaterSettingsButLeavesServiceMode) {
  sensor_.rejects_rate = true;
  DriverStim300::SensorSettings settings;
  settings.sample_rate_hz = 250;
  settings.lp_filter_hz = 66;
  settings.save_to_flash = true;
  EXPECT_FALSE(driver_.configureSensor(settings, log_));
  EXPECT_THAT(sensor_.commands, ElementsAre("SERVICEMODE", "m 1", "x N"));
  EXPECT_NE(std::string::npos, log_.find("E003"));
}

TEST_F(ConfigureSensorTest, RejectedFilterLeavesServiceMode) {
  sensor_.rejects_filter = true;
  DriverStim300::SensorSettings settings;
  settings.lp_filter_hz = 66;
  EXPECT_FALSE(driver_.configureSensor(settings, log_));
  EXPECT_THAT(sensor_.commands, ElementsAre("SERVICEMODE", "f 2", "x N"));
}

TEST_F(ConfigureSensorTest, UnsupportedValuesNeverTouchTheSensor) {
  DriverStim300::SensorSettings bad_rate;
  bad_rate.sample_rate_hz = 300;
  EXPECT_FALSE(driver_.configureSensor(bad_rate, log_));

  DriverStim300::SensorSettings bad_filter;
  bad_filter.lp_filter_hz = 100;
  EXPECT_FALSE(driver_.configureSensor(bad_filter, log_));

  EXPECT_THAT(sensor_.commands, IsEmpty());
}

TEST_F(ConfigureSensorTest, NothingToConfigureNeverTouchesTheSensor) {
  EXPECT_TRUE(driver_.configureSensor(DriverStim300::SensorSettings{}, log_));
  EXPECT_THAT(sensor_.commands, IsEmpty());
}

TEST_F(ConfigureSensorTest, RequestsTheConfigurationAfterwards) {
  DriverStim300::SensorSettings settings;
  settings.sample_rate_hz = 250;
  ASSERT_TRUE(driver_.configureSensor(settings, log_));
  sensor_.commands.clear();

  driver_.update();
  EXPECT_THAT(sensor_.commands, ElementsAre("C"));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
