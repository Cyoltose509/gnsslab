#include "rinex/RINEXObsReader.h"

#include "StringUtils.h"
#include "TimeConvert.h"

#include <sstream>

CommonTime RINEX3ObsReader::parseTime(const std::string &line) {
    if (Str::sub(line, 2, 27) == std::string(27, ' '))
        return CommonTime();
    const auto year = Str::toInt(Str::sub(line, 2, 4));
    const auto month = Str::toInt(Str::sub(line, 7, 2));
    const auto day = Str::toInt(Str::sub(line, 10, 2));
    const auto hour = Str::toInt(Str::sub(line, 13, 2));
    const auto minute = Str::toInt(Str::sub(line, 16, 2));
    const auto second = Str::toDouble(Str::sub(line, 19, 11));
    return Time::CivilToCommon(CivilTime(year, month, day, hour, minute, second));
}

bool RINEX3ObsReader::isEpochLine(const std::string &line) const {
    return line.size() >= 2 && line[0] == '>' && line[1] == ' ' &&
           Str::sub(line, 2, 27) != std::string(27, ' ');
}

CommonTime RINEX3ObsReader::epochTime(const std::string &line) const {
    return parseTime(line);
}

void RINEX3ObsReader::readEpochHead(std::string &line, int &flag, int &numSats) {
    flag = Str::toInt(Str::sub(line, 31, 1));
    numSats = Str::toInt(Str::sub(line, 32, 3));
    if (flag >= 2 && flag <= 5) {
        for (int i = 0; i < numSats; ++i) {
            if (!nextLine(line)) throw EndOfFile("EOF encountered!");
        }
    }
}

void RINEX3ObsReader::readHeaderLabel(const std::string &label, const std::string &line) {
    if (label == "SYS / # / OBS TYPES") {
        // A1 系统 + I3 个数 + 每行 13 个 A4
        const char sysStr = line.empty() ? ' ' : line[0];
        const int numObs = Str::toInt(Str::sub(line, 3, 3));
        constexpr int maxObsPerLine = 13;
        for (int i = 0; i < maxObsPerLine && static_cast<int>(mObsTypes[sysStr].size()) < numObs; i++) {
            std::string typeStr = Str::sub(line, 4 * i + 7, 3);
            mObsTypes[sysStr].push_back(typeStr);
        }
        rinexHeader.mapObsTypes = mObsTypes;
    } else if (label == "SYS / PHASE SHIFT") {
        std::istringstream ss(Str::strip(Str::sub(line, 0, 60)));
        if (std::string sysStr, obsType, valStr; ss >> sysStr >> obsType >> valStr) {
            const char sys = sysStr.empty() ? ' ' : sysStr[0];
            rinexHeader.phaseShift[std::string(1, sys) + obsType] = Str::toDouble(valStr);
        }
    }
}
