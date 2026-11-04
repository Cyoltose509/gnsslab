#include "rinex/RINEXObsReader.h"

#include "Const.h"
#include "CoordConvert.h"
#include "StringUtils.h"
#include "TimeConvert.h"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace {
    // 观测数据行首的卫星号：3 字符。RINEX3 是 "G01"，RINEX2 是 "  1"（GPS 系统位为空）或 "G 1"
    bool looksLikeSatLine(const std::string &line) {
        if (line.size() < 3) return false;
        if (std::isdigit(static_cast<unsigned char>(line[2])) == 0) return false;
        if (std::isdigit(static_cast<unsigned char>(line[1])) == 0 && line[1] != ' ') return false;
        const char sys = line[0];
        return sys == ' ' || sys == 'G' || sys == 'R' || sys == 'E' ||
               sys == 'C' || sys == 'J' || sys == 'I' || sys == 'S';
    }
}

void RINEXObsReader::parseRINEXHeader() {
    XYZ antennaPosition;
    bool haveAntDelta = false;
    double antDh = 0.0, antDe = 0.0, antDn = 0.0;

    scanLabeledHeader([&](const std::string &label, const std::string &line) {
        if (label == "MARKER NAME") {
            std::string markerName = Str::strip(Str::sub(line, 0, 60));
            std::replace(markerName.begin(), markerName.end(), ' ', '_');
            rinexHeader.station = Str::strip(markerName);
        } else if (label == "RINEX VERSION / TYPE") {
            rinexHeader.version = parseRinexVersionType(line).version;
        } else if (label == "APPROX POSITION XYZ") {
            antennaPosition[0] = Str::toDouble(Str::sub(line, 0, 14));
            antennaPosition[1] = Str::toDouble(Str::sub(line, 14, 14));
            antennaPosition[2] = Str::toDouble(Str::sub(line, 28, 14));
            rinexHeader.antennaPosition = antennaPosition;
        } else if (label == "ANTENNA: DELTA H/E/N") {
            antDh = Str::toDouble(Str::sub(line, 0, 14));
            antDe = Str::toDouble(Str::sub(line, 14, 14));
            antDn = Str::toDouble(Str::sub(line, 28, 14));
            haveAntDelta = true;
        } else if (label == "ANT # / TYPE") {
            const std::string type = Str::strip(Str::sub(line, 20, 20));
            const std::string radome = Str::strip(Str::sub(line, 40, 20));
            if (!type.empty()) rinexHeader.antType = radome.empty() ? type : type + " " + radome;
        } else if (label == "GLONASS SLOT / FRQ #") {
            // 两版格式一致：Rnn + 通道号成对出现
            std::istringstream ss(Str::strip(Str::sub(line, 0, 60)));
            std::string tok;
            while (ss >> tok) {
                if (tok.size() == 3 && tok[0] == 'R' &&
                    std::isdigit(static_cast<unsigned char>(tok[1])) != 0 &&
                    std::isdigit(static_cast<unsigned char>(tok[2])) != 0) {
                    if (std::string chTok; ss >> chTok) {
                        try { mGloChan[tok] = std::stoi(chTok); } catch (...) {
                        }
                    }
                }
            }
        } else {
            readHeaderLabel(label, line); // 版别特有
        }
    });

    if (haveAntDelta) {
        rinexHeader.antDeltaENU = Eigen::Vector3d(antDe, antDn, antDh);
    }
    afterHeader();
    isHeaderRead = true;
}

ObsData RINEXObsReader::parseRINEXObs() {
    if (!isHeaderRead) parseRINEXHeader();

    std::string line;
    while (true) {
        if (!nextLine(line)) throw EndOfFile("EOF encountered!");
        if (Str::strip(line).empty()) continue;
        if (isEpochLine(line)) break;
    }

    int epochFlag = 0, numSats = 0;
    const CommonTime currEpochRaw = epochTime(line);
    readEpochHead(line, epochFlag, numSats);
    if (epochFlag < 0 || epochFlag > 6) {
        throw FFStreamError("Invalid epoch flag: " + std::to_string(epochFlag));
    }

    CommonTime currEpoch = currEpochRaw;
    currEpoch.setTimeSystem(TimeSystem::GPS);

    SatTypeValueMap stvData;
    SatTypeLLIMap lliData;
    if (epochFlag == 0 || epochFlag == 1 || epochFlag == 6) {
        readSatObs(stvData, numSats, lliData);
    }

    ObsData obsData;
    obsData.station = rinexHeader.station;
    obsData.epoch = currEpoch;
    obsData.weekSecond = Time::CommonToWeekSecond(currEpoch);
    obsData.satTypeValueData = std::move(stvData);
    obsData.satTypeLLI = std::move(lliData);
    obsData.antennaPosition = rinexHeader.antennaPosition;
    obsData.antType = rinexHeader.antType;

    chooseObs(obsData);
    return obsData;
}

void RINEXObsReader::readSatObs(SatTypeValueMap &stvData, const int numSats,
                                SatTypeLLIMap &lliOut) {
    std::string line;
    for (int isv = 0; isv < numSats; ++isv) {
        if (!nextLine(line)) throw EndOfFile("EOF encountered!");
        if (!looksLikeSatLine(line)) {
            throw EndOfFile("观测数据结束");
        }

        SatID sat;
        try {
            sat = SatID(Str::sub(line, 0, 3));
        } catch (std::exception &e) {
            throw FFStreamError(e.what());
        }
        // RINEX2 的 GPS 卫星写作 "  1"（系统位空），归一成 'G'
        if (sat.system == ' ') sat.system = 'G';

        if (sat.system == 'R') {
            if (auto it = mGloChan.find(sat.toString()); it != mGloChan.end()) {
                sat.chan = it->second;
            }
        }

        if (sat.system != 'G' && sat.system != 'C' && sat.system != 'R' && sat.system != 'E') {
            continue;
        }

        std::string buf = line.size() > 3 ? line.substr(3) : std::string();
        if (const auto itTypes = rinexHeader.mapObsTypes.find(sat.system);
            itTypes != rinexHeader.mapObsTypes.end()) {
            buf.resize(16 * itTypes->second.size(), ' ');
        }
        decodeObsBlock(sat, buf, stvData, lliOut);
    }
}

void RINEXObsReader::decodeObsBlock(const SatID &sat, const std::string &buf,
                                    SatTypeValueMap &out, SatTypeLLIMap &lliOut) const {
    const auto itTypes = rinexHeader.mapObsTypes.find(sat.system);
    if (itTypes == rinexHeader.mapObsTypes.end()) return;
    const auto &types = itTypes->second;

    TypeValueMap typeObs;
    std::map<std::string, int> typeLLI;
    for (int i = 0; i < static_cast<int>(types.size()); ++i) {
        const std::size_t pos = 16 * i;
        if (pos >= buf.size()) break;
        const std::string &obsTypeStr = types[i];
        if (obsTypeStr.empty()) continue;

        const std::string tmpStr = Str::sub(buf, pos, 14);
        double data = Str::toDouble(tmpStr);

        if (obsTypeStr[0] == 'L') {
            // 第 15 个字符是 LLI：bit0 失锁、bit1 半周跳。空格＝未标注，0＝明确无标记。
            if (pos + 14 < buf.size()) {
                if (const char lliCh = buf[pos + 14]; lliCh >= '1' && lliCh <= '7')
                    typeLLI[obsTypeStr] = lliCh - '0';
            }

            const double freq = sat.system == 'R'
                                    ? getGlonassFreq(obsTypeStr[1], sat.chan)
                                    : getFreq(sat.system, obsTypeStr);
            const double wavelength = freq > 0.0 ? C_MPS / freq : 0.0;
            if (wavelength == 0.0) continue;
            // RINEX 的相位偏移头（v3 才有）单位是 cycle
            const std::string key = std::string(1, sat.system) + obsTypeStr;
            if (const auto it = rinexHeader.phaseShift.find(key); it != rinexHeader.phaseShift.end())
                data = data + it->second;
            data = data * wavelength;
        }

        if (std::abs(data) == 0.0) continue;
        typeObs[obsTypeStr] = data;
    }
    out[sat] = typeObs;
    if (!typeLLI.empty()) lliOut[sat] = std::move(typeLLI);
}

// 线性扫描到第一个不早于 target 的历元。只做定位，不解码数据。
void RINEXObsReader::fastForwardToEpoch(const CommonTime &target) const {
    if (!pStream || !pStream->good()) return;
    const std::streamoff startOff = pStream->tellg();
    pStream->seekg(0, std::ios::end);
    const std::streamoff endOff = pStream->tellg();
    const std::streamoff remaining = endOff - startOff;
    if (remaining <= 0) {
        pStream->clear();
        pStream->seekg(startOff);
        return;
    }
    std::string all;
    all.resize(static_cast<std::size_t>(remaining));
    pStream->seekg(startOff);
    pStream->read(&all[0], remaining);
    const auto readn = static_cast<std::size_t>(pStream->gcount());
    pStream->clear();
    if (readn == 0) {
        pStream->seekg(startOff);
        return;
    }
    all.resize(readn);

    std::size_t lineStart = 0;
    while (lineStart < readn) {
        std::size_t nl = all.find('\n', lineStart);
        if (nl == std::string::npos) nl = readn;
        std::size_t end = nl;
        if (end > lineStart && all[end - 1] == '\r') --end;
        if (end > lineStart) {
            if (const std::string one(all, lineStart, end - lineStart); isEpochLine(one)) {
                if (const CommonTime t = epochTime(one); t >= target) {
                    pStream->seekg(startOff + static_cast<std::streamoff>(lineStart));
                    return;
                }
            }
        }
        lineStart = nl + 1;
    }
    pStream->seekg(startOff);
}

void RINEXObsReader::chooseObs(ObsData &obsData) {
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
