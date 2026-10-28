#pragma once
#include <string>

#include "StringUtils.h"

namespace GnssTask {

enum class ObsFormat {
    Rinex,
    Oem7,
    Unknown
};

inline ObsFormat classifyObsPath(const std::string &path) {
    if (path.size() < 4) return ObsFormat::Unknown;
    if (endsWithIC(path, ".rnx") || endsWithIC(path, ".obs")) return ObsFormat::Rinex;

    if (std::isdigit(static_cast<unsigned char>(path[path.size() - 3])) &&
        std::toupper(static_cast<unsigned char>(path.back())) == 'O')
        return ObsFormat::Rinex;
    if (endsWithIC(path, ".log") || endsWithIC(path, ".oem7")) return ObsFormat::Oem7;
    return ObsFormat::Unknown;
}

}
