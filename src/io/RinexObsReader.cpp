#include "StringUtils.h"
#include "Const.h"
#include "RinexObsReader.h"

#include "TimeConvert.h"
#include "CoordConvert.h"


static bool looksLikeSatLine(const std::string &line) {
    if (line.size() < 3) return false;
    if (const char sys = line[0]; sys != 'G' && sys != 'R' && sys != 'E' && sys != 'C' &&
                                  sys != 'J' && sys != 'I' && sys != 'S')
        return false;
    return std::isdigit(static_cast<unsigned char>(line[1])) != 0 &&
           std::isdigit(static_cast<unsigned char>(line[2])) != 0;
}


void RinexObsReader::parseRinexHeader() {
    syncStream();
    XYZ antennaPosition;
    std::map<char, std::vector<string> > mapObsTypes;

    bool haveAntDelta = false;
    double antDh = 0.0, antDe = 0.0, antDn = 0.0;
    scanLabeledHeader([&](const std::string &label, std::string &line) {
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
        } else if (label == "ANTENNA: DELTA H/E/N") {
            antDh = safeStod(safeSubstr(line, 0, 14));
            antDe = safeStod(safeSubstr(line, 14, 14));
            antDn = safeStod(safeSubstr(line, 28, 14));
            haveAntDelta = true;
        } else if (label == "SYS / # / OBS TYPES") {
            const char sysStr = line[0];
            const int numObs = safeStoi(safeSubstr(line, 3, 3));
            constexpr int maxObsPerLine = 13;
            for (int i = 0; i < maxObsPerLine && static_cast<int>(mapObsTypes[sysStr].size()) < numObs; i++) {
                std::string typeStr = safeSubstr(line, 4 * i + 7, 3);
                mapObsTypes[sysStr].push_back(typeStr);
            }
            rinexHeader.mapObsTypes = mapObsTypes;
        } else if (label == "ANT # / TYPE") {
            const std::string type = strip(safeSubstr(line, 20, 20));
            const std::string radome = strip(safeSubstr(line, 40, 20));
            if (!type.empty()) rinexHeader.antType = radome.empty() ? type : type + " " + radome;
        } else if (label == "SYS / PHASE SHIFT") {
            std::istringstream ss(strip(safeSubstr(line, 0, 60)));

            if (std::string sysStr, obsType, valStr; ss >> sysStr >> obsType >> valStr) {
                const char sys = sysStr.empty() ? ' ' : sysStr[0];
                rinexHeader.phaseShift[std::string(1, sys) + obsType] = safeStod(valStr);
            }
        } else if (label == "GLONASS SLOT / FRQ #") {
            std::istringstream ss(strip(safeSubstr(line, 0, 60)));
            std::string tok;
            while (ss >> tok) {
                if (tok.size() == 3 && tok[0] == 'R' &&
                    std::isdigit(static_cast<unsigned char>(tok[1])) &&
                    std::isdigit(static_cast<unsigned char>(tok[2]))) {
                    if (std::string chTok; ss >> chTok) {
                        try { mGloChan[tok] = std::stoi(chTok); } catch (...) {
                        }
                    }
                }
            }
        }
    });
    if (haveAntDelta) {
        rinexHeader.antDeltaENU = Eigen::Vector3d(antDe, antDn, antDh);
    }

    isHeaderRead = true;
}

ObsData RinexObsReader::parseRinexObs() {
    syncStream();
    if (!isHeaderRead) {
        parseRinexHeader();
        isHeaderRead = true;
    }


    std::string line;
    while (true) {
        if (!nextLine(line)) throw EndOfFile("EOF encountered!");

        if (strip(line).empty()) continue;
        bool isEpochLine = line.size() >= 2 && line[0] == '>' && line[1] == ' ' &&
                           safeSubstr(line, 2, 27) != std::string(27, ' ');
        if (isEpochLine) break;
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
        if (!nextLine(line)) throw EndOfFile("EOF encountered!");
        if (!looksLikeSatLine(line)) {
            throw EndOfFile("观测数据结束");
        }
            try {
                satIndex[isv] = SatID(safeSubstr(line, 0, 3));
            } catch (std::exception &e) {
                throw FFStreamError(e.what());
            }

            auto sat = satIndex[isv];
            if (sat.system == 'R') {
                if (auto it = mGloChan.find(sat.toString()); it != mGloChan.end()) {
                    satIndex[isv].chan = it->second;
                    sat.chan = it->second;
                }
            }

            if (sat.system != 'G' && sat.system != 'C' && sat.system != 'R' && sat.system != 'E') {
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

                    double freq = sat.system == 'R'
                                      ? getGlonassFreq(obsTypeStr[1], sat.chan)
                                      : getFreq(sat.system, obsTypeStr);
                    double wavelength = freq > 0.0 ? C_MPS / freq : 0.0;
                    if (wavelength == 0.0) continue;
                    const std::string key = std::string(1, sat.system) + obsTypeStr;
                    if (const auto it = rinexHeader.phaseShift.find(key); it != rinexHeader.phaseShift.end()) data = data + it->second;
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
    obsData.antType = rinexHeader.antType;

    chooseObs(obsData);
    return obsData;
}

void RinexObsReader::fastForwardToEpoch(const CommonTime &target) const {
    if (!pFileStream || !pFileStream->good()) return;
    const std::streamoff startOff = pFileStream->tellg();
    pFileStream->seekg(0, std::ios::end);
    const std::streamoff endOff = pFileStream->tellg();
    const std::streamoff remaining = endOff - startOff;
    if (remaining <= 0) return;
    std::string all;
    all.resize(static_cast<std::size_t>(remaining));
    pFileStream->seekg(startOff);
    pFileStream->read(&all[0], remaining);
    const auto readn = static_cast<std::size_t>(pFileStream->gcount());
    if (readn == 0) return;
    all.resize(readn);

    std::size_t lineStart = 0;
    while (lineStart < readn) {
        std::size_t nl = all.find('\n', lineStart);
        if (nl == std::string::npos) nl = readn;
        if (const std::size_t lineLen = nl - lineStart; lineLen >= 2 && all[lineStart] == '>' && all[lineStart + 1] == ' ') {
            std::size_t end = nl;
            if (end > lineStart && all[end - 1] == '\r') --end;
            bool timeEmpty = true;
            for (int k = 2; k < 29 && lineStart + k < end; ++k) {
                if (all[lineStart + k] != ' ') {
                    timeEmpty = false;
                    break;
                }
            }
            if (!timeEmpty) {
                int epochFlag = 0;
                if (lineStart + 31 < end) epochFlag = safeStoi(std::string(1, all[lineStart + 31]));
                if (epochFlag == 0 || epochFlag == 1 || epochFlag == 6) {
                    CommonTime t = parseTime(std::string(all, lineStart, end - lineStart));
                    t.setTimeSystem(TimeSystem::GPS);
                    if (t >= target) {
                        pFileStream->clear();
                        pFileStream->seekg(startOff + static_cast<std::streamoff>(lineStart));
                        return;
                    }
                }
            }
        }
        lineStart = nl + 1;
    }

}

CommonTime RinexObsReader::parseTime(const string &line) {
    if (safeSubstr(line, 2, 27) == string(27, ' '))
        return CommonTime();
    const auto year = safeStoi(safeSubstr(line, 2, 4));
    const auto month = safeStoi(safeSubstr(line, 7, 2));
    const auto day = safeStoi(safeSubstr(line, 10, 2));
    const auto hour = safeStoi(safeSubstr(line, 13, 2));
    const auto minute = safeStoi(safeSubstr(line, 16, 2));
    const auto second = safeStod(safeSubstr(line, 19, 11));
    return CivilTime2CommonTime(CivilTime(year, month, day, hour, minute, second));
}

void RinexObsReader::chooseObs(ObsData &obsData) {
    if (sysTypes.empty()) return;

    for (auto it = obsData.satTypeValueData.begin(); it != obsData.satTypeValueData.end();) {
        const char sys = it->first.system;
        if (sysTypes.find(sys) == sysTypes.end()) {
            it = obsData.satTypeValueData.erase(it);
            continue;
        }
        const auto &wantedTypes = sysTypes.at(sys);
        TypeValueMap filtered;
        for (const auto &[type, value]: it->second) {
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
