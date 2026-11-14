#pragma once

#include <iomanip>
#include <map>
#include <ostream>
#include <string>

#include <Eigen/Core>

#include "core/SatID.h"
#include "TimeStruct.h"
#include "TimeStruct.h"

typedef std::map<std::string, double> TypeValueMap;

inline std::ostream &operator<<(std::ostream &os, const TypeValueMap &typeValueMap) {
    for (const auto &[type, value]: typeValueMap) {
        os << "  Type: " << type
                << ", Value: " << std::fixed << std::setprecision(6) << value << "\n";
    }
    return os;
}

typedef std::map<SatID, double> SatValueMap;

inline std::ostream &operator<<(std::ostream &os, const SatValueMap &satValueMap) {
    for (const auto &[sat, value]: satValueMap) {
        os << "  sat: " << sat
                << ", Value: " << std::fixed << std::setprecision(6) << value << "\n";
    }
    return os;
}


typedef std::map<SatID, TypeValueMap> SatTypeValueMap;
typedef std::map<SatID, std::map<std::string, int> > SatTypeLLIMap;

inline std::ostream &operator<<(std::ostream &os, const SatTypeValueMap &satTypeValueMap) {
    for (const auto &[sat, tv]: satTypeValueMap) {
        os << "Satellite ID: " << sat << "\n";
        os << tv;
    }
    return os;
}

struct ObsData {
    Eigen::Vector3d antennaPosition{0, 0, 0};

    std::string antType;

    std::string station;

    CommonTime epoch;

    WeekSecond weekSecond;
    SatTypeValueMap satTypeValueData;

    // 相位观测的 RINEX LLI
    SatTypeLLIMap satTypeLLI;
};

inline std::ostream &operator<<(std::ostream &os, const ObsData &data) {
    os << "Epoch: " << data.epoch.toCivilTime() << "\n";

    if (!data.satTypeValueData.empty()) {
        os << "Satellite Data:\n";
        os << data.satTypeValueData;
    } else {
        os << "No satellite data available.\n";
    }

    return os;
}
