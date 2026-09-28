#include "StringUtils.h"
#include "Const.h"
#include "RinexObsReader.h"

#include "TimeConvert.h"
#include "CoordConvert.h"

#include <cctype>

// 卫星观测行形如 "G12 ..."：首字符为星座标识、其后两位为 PRN 数字。
// 用于识别被追加在文件尾部的非观测内容（如 INI 段/统计块），避免其被当成卫星行、
// 进而触发 SatID 内部 std::stoi 抛错（"invalid stoi argument"）而毁掉整个任务。
static bool looksLikeSatLine(const std::string &line) {
    if (line.size() < 3) return false;
    const char sys = line[0];
    if (sys != 'G' && sys != 'R' && sys != 'E' && sys != 'C' &&
        sys != 'J' && sys != 'I' && sys != 'S')
        return false;
    return std::isdigit(static_cast<unsigned char>(line[1])) != 0 &&
           std::isdigit(static_cast<unsigned char>(line[2])) != 0;
}


void RinexObsReader::parseRinexHeader() {
    XYZ antennaPosition;
    char satSys;
    std::map<char, std::vector<string> > mapObsTypes;
    // 天线偏心（marker -> 天线参考点 ARP），RINEX: ANTENNA: DELTA H/E/N（本地 E/N/U 偏移）
    bool haveAntDelta = false;
    double antDh = 0.0, antDe = 0.0, antDn = 0.0;
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
        } else if (label == "ANTENNA: DELTA H/E/N") {

            antDh = safeStod(safeSubstr(line, 0, 14));
            antDe = safeStod(safeSubstr(line, 14, 14));
            antDn = safeStod(safeSubstr(line, 28, 14));
            haveAntDelta = true;
        } else if (label == "SYS / # / OBS TYPES") {
            const char sysStr= line[0];
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
        }
    }
    if (haveAntDelta) {
        rinexHeader.antDeltaENU = Eigen::Vector3d(antDe, antDn, antDh);
    }

    isHeaderRead = true;
}

ObsData RinexObsReader::parseRinexObs() {
    if (!isHeaderRead) {
        parseRinexHeader();
        isHeaderRead = true;
    }

    // 读取观测值（跳过空行以及可能被污染的尾部非 epoch 行）
    std::string line;
    while (true) {
        getline(*pFileStream, line);
        if (pFileStream->eof()) {
            throw EndOfFile("EOF encountered!");
        }
        // 去除 Windows 换行残留的 \r
        if (!line.empty() && line.back() == '\r') line.pop_back();
        // 跳过空行
        if (strip(line).empty()) continue;
        // 正常 epoch 行以 "> " 开头且时间字段(cols 2-28)非空；个别文件(HKWS 样本)尾部带有
        // ">     4212" 形式的统计块首行，时间字段全空，会被误判为 epoch，进而 parseTime 返回
        // day=0 的 CommonTime 触发 "before Epoch" 错误。仅当 "> " 开头且时间字段非空才视为 epoch 行，
        // 否则继续读取(走到 EOF 由调用方 EndOfFile 分支优雅结束)。
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
            getline(*pFileStream, line);
            if (pFileStream->eof()) {
                throw EndOfFile("EOF encountered!");
            }
            if (!line.empty() && line.back() == '\r') line.pop_back();
            // 历元头声明的卫星数比实际观测行多（尾部被追加了非观测内容）时，读到这里的内容
            // 不是卫星行。视为观测数据结束——与「历元块中途遇 EOF」同义，交给调用方的
            // EndOfFile 分支优雅收尾，而不是让 SatID 的 stoi 抛错把整个任务判为解析失败。
            if (!looksLikeSatLine(line)) {
                throw EndOfFile("观测数据结束（历元块后存在非观测内容）");
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
                    double freq = getFreq(sat.system, obsTypeStr);
                    double wavelength = freq > 0.0 ? C_MPS / freq : 0.0;
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
    obsData.antType = rinexHeader.antType;

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
