#pragma once

#include <stdint.h>

#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

class Channel {
public:
  typedef std::false_type fixed_size;
  static const char* descriptor;

  int32_t id = 0;
  std::string name;
  std::string subsystem;

  inline bool operator==(const Channel& _rhs) const {
    return std::tie(id, name, subsystem) == std::tie(_rhs.id, _rhs.name, _rhs.subsystem);
  }
  inline bool operator<(const Channel& _rhs) const {
    return std::tie(id, name, subsystem) < std::tie(_rhs.id, _rhs.name, _rhs.subsystem);
  }
  inline bool operator!=(const Channel& _rhs) const {
    return !(*this == _rhs);
  }
  inline bool operator>(const Channel& _rhs) const {
    return _rhs < *this;
  }
  inline bool operator>=(const Channel& _rhs) const {
    return !(*this < _rhs);
  }
  inline bool operator<=(const Channel& _rhs) const {
    return !(_rhs < *this);
  }

  inline std::string toString() const {
    std::ostringstream _aidl_os;
    _aidl_os << "Channel{";
    _aidl_os << "id: " << id;
    _aidl_os << ", name: " << name;
    _aidl_os << ", subsystem: " << subsystem;
    _aidl_os << "}";
    return _aidl_os.str();
  }
};

class EnergyMeasurement {
public:
  typedef std::false_type fixed_size;
  static const char* descriptor;

  int32_t id = 0;
  int64_t timestampMs = 0L;
  int64_t durationMs = 0L;
  int64_t energyUWs = 0L;

  inline bool operator==(const EnergyMeasurement& _rhs) const {
    return std::tie(id, timestampMs, durationMs, energyUWs) == std::tie(_rhs.id, _rhs.timestampMs, _rhs.durationMs, _rhs.energyUWs);
  }
  inline bool operator<(const EnergyMeasurement& _rhs) const {
    return std::tie(id, timestampMs, durationMs, energyUWs) < std::tie(_rhs.id, _rhs.timestampMs, _rhs.durationMs, _rhs.energyUWs);
  }
  inline bool operator!=(const EnergyMeasurement& _rhs) const {
    return !(*this == _rhs);
  }
  inline bool operator>(const EnergyMeasurement& _rhs) const {
    return _rhs < *this;
  }
  inline bool operator>=(const EnergyMeasurement& _rhs) const {
    return !(*this < _rhs);
  }
  inline bool operator<=(const EnergyMeasurement& _rhs) const {
    return !(_rhs < *this);
  }

  inline std::string toString() const {
    std::ostringstream _aidl_os;
    _aidl_os << "EnergyMeasurement{";
    _aidl_os << "id: " << id;
    _aidl_os << ", timestampMs: " << timestampMs;
    _aidl_os << ", durationMs: " << durationMs;
    _aidl_os << ", energyUWs: " << energyUWs;
    _aidl_os << "}";
    return _aidl_os.str();
  }
};

class IioEnergyMeterDataProvider {
  public:
    IioEnergyMeterDataProvider(const std::vector<std::string> &deviceNames);

    void StartAll();

    void StopAll();

    void PrintColumnNames(std::ostringstream* output_string);

    void PrintStats(std::ostringstream* output_string);

  private:
    // Methods from PowerStats::IRailEnergyDataProvider
    int readEnergyMeter(const std::vector<int32_t> &in_channelIds,
                                       std::vector<EnergyMeasurement> *_aidl_return);
    int getEnergyMeterInfo(std::vector<Channel> *_aidl_return);


    void findIioEnergyMeterNodes();
    void parseEnabledRails();
    int parseEnergyValue(std::string path);
    int parseEnergyContents(const std::string &contents);

    std::mutex mLock;
    std::unordered_map<std::string, std::string> mDevicePaths;  // key: path, value: device name
    std::unordered_map<std::string, int32_t> mChannelIds;  // key: name, value: id
    std::vector<Channel> mChannelInfos;
    std::vector<EnergyMeasurement> mReading;
    std::vector<EnergyMeasurement> mLastReading;

    const std::vector<std::string> kDeviceNames;
    const std::string kDeviceType = "iio:device";
    const std::string kIioRootDir = "/sys/bus/iio/devices/";
    const std::string kNameNode = "/name";
    const std::string kEnabledRailsNode = "/enabled_rails";
    const std::string kEnergyValueNode = "/energy_value";
};
