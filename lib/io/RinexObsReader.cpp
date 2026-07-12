#include "StringUtils.h"
#include "RinexObsReader.h"

#include "Log.h"
#include "TimeConvert.h"


void RinexObsReader::parseRinexHeader() {
    XYZ antennaPosition;
    char satSys;
    std::map<char, std::vector<string> > mapObsTypes;
    while (true) {
        string line;
        getline(*pFileStream, line);
        if (pFileStream->eof()) {
            throw FFStreamError("Unexpected EOF in RINEX header");
        }

        string label;
        label = strip(safeSubstr(line, 60, 20));

        if (label == "END OF HEADER") {
            break;
        }
        if (label == "MARKER NAME") {
            string markerName = strip(safeSubstr(line, 0, 60));
            std::replace(markerName.begin(), markerName.end(), ' ', '_');
            rinexHeader.station = strip(markerName);
        } else if (label == "RINEX VERSION / TYPE") {
            const double version = safeStod(safeSubstr(line, 0, 20));
            if (version < 3.0) {
                throw FFStreamError("don't support RINEX version < 3.0");
            }
            rinexHeader.version = version;
        } else if (label == "APPROX POSITION XYZ") {
            antennaPosition[0] = safeStod(safeSubstr(line, 0, 14));
            antennaPosition[1] = safeStod(safeSubstr(line, 14, 14));
            antennaPosition[2] = safeStod(safeSubstr(line, 28, 14));
            rinexHeader.antennaPosition = antennaPosition;
        } else if (label == "SYS / # / OBS TYPES") {
            const char sysStr= line[0];
            const int numObs = safeStoi(safeSubstr(line, 3, 3));
            constexpr int maxObsPerLine = 13;
            for (int i = 0; i < maxObsPerLine && static_cast<int>(mapObsTypes[sysStr].size()) < numObs; i++) {
                std::string typeStr = safeSubstr(line, 4 * i + 7, 3);
                mapObsTypes[sysStr].push_back(typeStr);
            }
            rinexHeader.mapObsTypes = mapObsTypes;
        }
    }
}

ObsData RinexObsReader::parseRinexObs() {
    if (!isHeaderRead) {
        parseRinexHeader();
        isHeaderRead = true;
    }

    // 读取观测值
    std::string line;
    getline(*pFileStream, line);

    if (pFileStream->eof()) {
        throw EndOfFile("EOF encountered!");
    }

    if (line[0] != '>' || line[1] != ' ') {
        throw FFStreamError("Bad epoch line: >" + line + "<");
    }

    int epochFlag = safeStoi(safeSubstr(line, 31, 1));
    if (epochFlag < 0 || epochFlag > 6) {
        throw FFStreamError("Invalid epoch flag: " + std::to_string(epochFlag));
    }

    CommonTime currEpoch = parseTime(line);
    currEpoch.setTimeSystem(TimeSystem::GPS);

    int numSats = safeStoi(safeSubstr(line, 32, 3));
    SatTypeValueMap stvData;
    if (epochFlag == 0 || epochFlag == 1 || epochFlag == 6) {
        std::vector<SatID> satIndex(numSats);
        for (int isv = 0; isv < numSats; ++isv) {
            getline(*pFileStream, line);
            if (pFileStream->eof()) {
                throw EndOfFile("EOF encountered!");
            }
            try {
                satIndex[isv] = SatID(safeSubstr(line, 0, 3));
            } catch (std::exception &e) {
                throw FFStreamError(e.what());
            }

            auto sat = SatID(satIndex[isv]);
            // 仅解析已实现定位的星座：GPS/BDS
            if (sat.system != 'G' && sat.system != 'C') {
                continue;
            }

            auto size = static_cast<int>(rinexHeader.mapObsTypes.at(sat.system).size());

            if (size_t minSize = 3 + 16 * size; line.size() < minSize) {
                line += std::string(minSize - line.size(), ' ');
            }

            TypeValueMap typeObs;
            for (int i = 0; i < size; ++i) {
                size_t pos = 3 + 16 * i;
                std::string str = safeSubstr(line, pos, 16);
                std::string obsTypeStr = rinexHeader.mapObsTypes.at(sat.system)[i];

                std::string tmpStr = safeSubstr(str, 0, 14);
                double data = safeStod(tmpStr);

                if (obsTypeStr[0] == 'L') {
                    double wavelength = 0.0;
                    int n;
                    if (obsTypeStr[1] == 'A') {
                        n = 1;
                    } else {
                        n = safeStoi(safeSubstr(obsTypeStr, 1, 1));
                    }
                    wavelength = getWavelength(sat.system, n);
                    if (wavelength == 0.0) continue;
                    data = data * wavelength;
                }

                if (std::abs(data) == 0.0) {
                    continue;
                }
                typeObs[obsTypeStr] = data;
            }

            stvData[satIndex[isv]] = typeObs;
        }
    }

    ObsData obsData;
    obsData.station = rinexHeader.station;
    obsData.epoch = currEpoch;
    CommonTime2WeekSecond(currEpoch, obsData.weekSecond);
    obsData.satTypeValueData = stvData;
    obsData.antennaPosition = rinexHeader.antennaPosition;

    chooseObs(obsData);
    return obsData;
}

CommonTime RinexObsReader::parseTime(const string &line) {
    if (safeSubstr(line, 2, 27) == string(27, ' '))
        return CommonTime();
    const auto year   = safeStoi(safeSubstr(line, 2, 4));
    const auto month  = safeStoi(safeSubstr(line, 7, 2));
    const auto day    = safeStoi(safeSubstr(line, 10, 2));
    const auto hour   = safeStoi(safeSubstr(line, 13, 2));
    const auto minute = safeStoi(safeSubstr(line, 16, 2));
    const auto second = safeStod(safeSubstr(line, 19, 11));
    return CivilTime2CommonTime(CivilTime(year, month, day, hour, minute, second));
}

void RinexObsReader::chooseObs(ObsData &obsData) {
    if (sysTypes.empty()) return;

    for (auto it = obsData.satTypeValueData.begin(); it != obsData.satTypeValueData.end(); ) {
        const char sys = it->first.system;
        if (sysTypes.find(sys) == sysTypes.end()) {
            it = obsData.satTypeValueData.erase(it);
            continue;
        }
        const auto &wantedTypes = sysTypes.at(sys);
        TypeValueMap filtered;
        for (const auto &[type, value] : it->second) {
            if (wantedTypes.count(type)) filtered[type] = value;
        }
        if (filtered.empty()) {
            it = obsData.satTypeValueData.erase(it);
        } else {
            it->second = std::move(filtered);
            ++it;
        }
    }
}
