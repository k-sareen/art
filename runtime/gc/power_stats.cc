#include "power_stats.h"

#include "android-base/file.h"
#include "android-base/logging.h"
#include "android-base/stringprintf.h"
#include "android-base/strings.h"

#include <dirent.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sstream>

#define MAX_RAIL_NAME_LEN 50
#define STR(s) #s
#define XSTR(s) STR(s)

void IioEnergyMeterDataProvider::findIioEnergyMeterNodes() {
    struct dirent *ent;

    DIR *iioDir = opendir(kIioRootDir.c_str());
    if (!iioDir) {
        LOG(ERROR) << "Error opening directory" << kIioRootDir;
        return;
    }

    // Find any iio:devices that match the given kDeviceNames
    while (ent = readdir(iioDir), ent) {
        std::string devTypeDir = ent->d_name;
        if (devTypeDir.find(kDeviceType) != std::string::npos) {
            std::string devicePath = kIioRootDir + devTypeDir;
            std::string deviceNameContents;

            if (!android::base::ReadFileToString(devicePath + kNameNode, &deviceNameContents)) {
                LOG(ERROR) << "Failed to read device name from " << devicePath;
            } else {
                for (const auto &deviceName : kDeviceNames) {
                    if (deviceNameContents.find(deviceName) != std::string::npos) {
                        mDevicePaths.emplace(devicePath, deviceName);
                    }
                }
            }
        }
    }

    closedir(iioDir);
    return;
}

void IioEnergyMeterDataProvider::parseEnabledRails() {
    std::string data;
    int32_t id = 0;
    for (const auto &path : mDevicePaths) {
        // Get list of enabled rails
        if (!android::base::ReadFileToString(path.first + kEnabledRailsNode, &data)) {
            LOG(ERROR) << "Error reading enabled rails from " << path.first;
            continue;
        }

        // Build RailInfos from list of enabled rails
        std::istringstream railNames(data);
        std::string line;
        while (std::getline(railNames, line)) {
            /* Format example: CH2[VSYS_PWR_RFFE]:Cellular */
            std::vector<std::string> words = android::base::Split(line, ":][");
            if (words.size() == 4) {
                const std::string channelName = words[1];
                const std::string subsystemName = words[3];
                if (mChannelIds.count(channelName) == 0) {
                    mChannelInfos.push_back(
                            {.id = id, .name = channelName, .subsystem = subsystemName});
                    mChannelIds.emplace(channelName, id);
                    id++;
                } else {
                    LOG(ERROR) << "There exists rails with the same name (not supported): "
                                 << channelName << ". Only the last occurrence of rail energy will "
                                 << "be provided.";
                }
            } else {
                LOG(ERROR) << "Unexpected enabled rail format in " << path.first;
            }
        }
    }
}

IioEnergyMeterDataProvider::IioEnergyMeterDataProvider(const std::vector<std::string> &deviceNames)
    : kDeviceNames(std::move(deviceNames)) {
    findIioEnergyMeterNodes();
    parseEnabledRails();
    mReading.resize(mChannelInfos.size());
}

int IioEnergyMeterDataProvider::parseEnergyContents(const std::string &contents) {
    std::istringstream energyData(contents);
    std::string line;

    int ret = 0;
    uint64_t timestamp = 0;
    bool timestampRead = false;

    while (std::getline(energyData, line)) {
        bool parseLineSuccess = false;

        if (timestampRead == false) {
            /* Read timestamp from boot (ms) */
            if (sscanf(line.c_str(), "t=%" PRIu64, &timestamp) == 1) {
                if (timestamp == 0 || timestamp == ULLONG_MAX) {
                    LOG(ERROR) << "Potentially wrong timestamp: " << timestamp;
                }
                timestampRead = true;
                parseLineSuccess = true;
            }

        } else {
            /* Read rail energy */
            uint64_t energy = 0;
            uint64_t duration = 0;
            char railNameRaw[MAX_RAIL_NAME_LEN + 1];

            /* Format example: CH3(T=358356)[S2M_VDD_CPUCL2], 761330 */
            if (sscanf(line.c_str(),
                       "CH%*d(T=%" PRIu64 ")[%" XSTR(MAX_RAIL_NAME_LEN) "[^]]], %" PRIu64,
                       &duration, railNameRaw, &energy) == 3) {
                std::string railName(railNameRaw);

                /* If the count == 0, the rail may not be enabled */
                /* The count cannot be > 1; mChannelIds is a map */
                if (mChannelIds.count(railName) == 1) {
                    size_t index = mChannelIds[railName];
                    mReading[index].id = index;
                    mReading[index].timestampMs = timestamp;
                    mReading[index].durationMs = duration;
                    mReading[index].energyUWs = energy;
                    if ((size_t) mReading[index].energyUWs == ULLONG_MAX) {
                        LOG(ERROR) << "Potentially wrong energy value on rail: " << railName;
                    }
                    // ATRACE_INT(railNameRaw, energy);
                }
                parseLineSuccess = true;
            }
        }

        if (parseLineSuccess == false) {
            ret = -1;
            break;
        }
    }

    return ret;
}

int IioEnergyMeterDataProvider::parseEnergyValue(std::string path) {
    int ret = 0;
    std::string data;
    if (!android::base::ReadFileToString(path + kEnergyValueNode, &data)) {
        LOG(ERROR) << "Error reading energy value in " << path;
        return -1;
    }

    ret = parseEnergyContents(data);
    if (ret != 0) {
        LOG(ERROR) << "Unexpected format in " << path;
    }
    return ret;
}

int IioEnergyMeterDataProvider::readEnergyMeter(
        const std::vector<int32_t> &in_channelIds, std::vector<EnergyMeasurement> *_aidl_return) {
    std::scoped_lock lock(mLock);

    for (const auto &devicePath : mDevicePaths) {
        if (parseEnergyValue(devicePath.first) < 0) {
            LOG(ERROR) << "Error in parsing " << devicePath.first;
            return -1;
        }
    }

    if (in_channelIds.empty()) {
        *_aidl_return = mReading;
    } else {
        _aidl_return->reserve(in_channelIds.size());
        for (const auto &id : in_channelIds) {
            // check for invalid ids
            if (id < 0 || (size_t) id >= mChannelInfos.size()) {
                return -1;
            }

            _aidl_return->emplace_back(mReading[id]);
        }
    }

    return 0;
}

int IioEnergyMeterDataProvider::getEnergyMeterInfo(
        std::vector<Channel> *_aidl_return) {
    std::scoped_lock lk(mLock);
    *_aidl_return = mChannelInfos;

    return 0;
}

void IioEnergyMeterDataProvider::StartAll() {
    readEnergyMeter({}, &mLastReading);
}

void IioEnergyMeterDataProvider::StopAll() {
    std::vector<EnergyMeasurement> currentReading;
    readEnergyMeter({}, &currentReading);
}

void IioEnergyMeterDataProvider::PrintColumnNames(std::ostringstream* output_string) {
    for (auto channel_info : mChannelInfos) {
        *output_string << channel_info.name << ".energy\t" << channel_info.name << ".power\t";
    }
    *output_string << "total.energy\ttotal.power\t";
}

void IioEnergyMeterDataProvider::PrintStats(std::ostringstream* output_string) {
    double total_power = 0;
    size_t total_energy = 0;
    for (auto channel_info : mChannelInfos) {
        size_t index = mChannelIds[channel_info.name];
        size_t energy = mReading[index].energyUWs - mLastReading[index].energyUWs;
        size_t duration = mReading[index].durationMs - mLastReading[index].durationMs;
        double power = energy / (duration / 1000.0);
        total_power += power;
        total_energy += energy;
        *output_string << energy << "\t" << power << "\t";
    }
    *output_string << total_energy << "\t" << total_power << "\t";
}
