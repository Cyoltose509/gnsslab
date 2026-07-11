#include <fstream>
#include <vector>
#include <iomanip>
#include <algorithm>

#include "TimeConvert.h"
#include "GnssStruct.h"
#include "OEM7Reader.h"

#define ID_RANGE        43
#define ID_GPSEPHEM     7
#define ID_BDSEPHEMRIS  1696
#define ID_BESTPOS      42
#define POLYCRC32 0xEDB88320u

bool OEM7Reader::open(const std::string &filename) {
    std::ifstream ifs(filename, std::ios::binary | std::ios::ate);
    if (!ifs) return false;

    // 记录文件总大小用于进度显示
    fileSize_ = static_cast<size_t>(ifs.tellg());
    ifs.seekg(0, std::ios::beg);

    buffer.clear();
    bufferIndex = 0;
    uint8_t tmp[4096];
    while (ifs.read(reinterpret_cast<char *>(tmp), sizeof(tmp)) || ifs.gcount() > 0) {
        buffer.insert(buffer.end(), tmp, tmp + ifs.gcount());
    }
    ifs.close();
    return !buffer.empty();
}

void OEM7Reader::close() {
    buffer.clear();
    bufferIndex = 0;
}

void OEM7Reader::readAll(std::vector<ObsData> &epochs,
                         std::vector<EphemerisTable> &ephSnapshots,
                         std::atomic<float> *progress) {
    EphemerisTable currentTable;
    std::vector<uint8_t> message;
    while (getNextMessage(message)) {
        if (progress) progress->store(this->progress());
        if (!parseMessage(message)) continue;

        // 仅星历消息才更新星历表
        if (currentHeader.type == ID_GPSEPHEM) {
            for (const auto &[prn, eph] : latestGps)
                currentTable.gps[prn] = std::make_shared<GPSEphem>(eph);
        } else if (currentHeader.type == ID_BDSEPHEMRIS) {
            for (const auto &[prn, eph] : latestBds)
                currentTable.bds[prn] = std::make_shared<BDSEphem>(eph);
        }

        // RANGE 观测：存为历元，同时快照此刻的星历表
        if (!currentObs.satTypeValueData.empty()) {
            epochs.push_back(std::move(currentObs));
            ephSnapshots.push_back(currentTable);  // 逐历元快照
            currentObs.satTypeValueData.clear();
        }
    }
    if (progress) progress->store(1.0f);
}

bool OEM7Reader::getNextMessage(std::vector<uint8_t> &message) {
    while (buffer.size() - bufferIndex >= 3) {
        // 从 bufferIndex 开始扫描同步头 0xAA 0x44 0x12
        size_t start = bufferIndex;
        bool found = false;
        for (; start + 2 < buffer.size(); ++start) {
            if (buffer[start] == 0xAA && buffer[start + 1] == 0x44 && buffer[start + 2] == 0x12) {
                found = true;
                break;
            }
        }

        if (!found) {
            buffer.clear();
            bufferIndex = 0;
            return false;
        }

        // 跳过同步头之前的无效数据（移动指针替代 erase）
        bufferIndex = start;

        if (constexpr size_t headerLen = 28; buffer.size() - bufferIndex < headerLen)
            return false;

        const auto hlen = buffer[bufferIndex + 3];
        const auto msgLen = U2(&buffer[bufferIndex + 8]);
        const auto totalLen = hlen + msgLen + 4;

        if (buffer.size() - bufferIndex < totalLen) {
            // 同步头后数据不足 → 跳过此同步头继续搜索
            bufferIndex += 3;
            continue;
        }

        message.assign(buffer.begin() + bufferIndex, buffer.begin() + bufferIndex + totalLen);
        bufferIndex += totalLen;
        return true;
    }
    return false;
}

bool OEM7Reader::crcExam(const std::vector<uint8_t> &message) {
    if (message.size() < 4) return false;
    const auto msg_len = message.size() - 4;
    auto *buff = message.data();
    unsigned int calc = 0;
    for (int i = 0; i < msg_len; i++) {
        calc ^= buff[i];
        for (int j = 0; j < 8; j++) {
            if (calc & 1) calc = calc >> 1 ^ POLYCRC32;
            else calc >>= 1;
        }
    }
    return calc == U4(buff + msg_len);
}

bool OEM7Reader::parseMessage(const std::vector<uint8_t> &message) {
    if (!crcExam(message)) return false;

    currentHeader.hLen = message[3];
    currentHeader.type = U2(&message[4]);
    currentHeader.length = U2(&message[8]);
    currentHeader.week = U2(&message[14]);
    currentHeader.ms = I4(&message[16]);

    switch (currentHeader.type) {
        case ID_RANGE:
            return parseRange(message);
        case ID_GPSEPHEM:
            return parseGpsEphem(message);
        case ID_BDSEPHEMRIS:
            return parseBdsEphem(message);
        case ID_BESTPOS:
            return parseBestPos(message);
        default:
            break;
    }
    return false;
}

bool OEM7Reader::parseRange(const std::vector<uint8_t> &message) {
    currentObs.satTypeValueData.clear();
    WeekSecond ws(currentHeader.week, currentHeader.ms * 0.001, TimeSystem::GPS);
    WeekSecond2CommonTime(ws, currentObs.epoch);

    currentObs.weekSecond = std::move(ws);

    int off = currentHeader.hLen;
    const uint32_t numObs = U4(&message[off]);
    off += 4;

    for (uint32_t i = 0; i < numObs; ++i) {
        if (off + 44 > message.size()) break;
        const auto prn = U2(&message[off + 0]);
        const auto psr = R8(&message[off + 4]);
        const auto adr = R8(&message[off + 16]);
        const auto doppler = R4(&message[off + 28]);
        const auto cn0 = R4(&message[off + 32]);
        const auto trackingStatus = U4(&message[off + 40]);
        const auto sysInt = static_cast<int>(trackingStatus >> 16) & 7;
        const auto sigType = static_cast<int>(trackingStatus >> 21) & 0x1F;
        const auto codeLocked = static_cast<int>(trackingStatus >> 12) & 1;
        const auto phaseLocked = static_cast<int>(trackingStatus >> 10) & 1;

        char sys;
        int freqIdx;

        if (sysInt == 0) {
            // GPS
            sys = 'G';
            switch (sigType) {
                case 0: // L1 C/A
                    freqIdx = 1;
                    break;
                case 9: // L2 P(Y)
                    freqIdx = 2;
                    break;
                default:
                    freqIdx = -1;
                    break;
            }
        } else if (sysInt == 4) {
            // BDS
            sys = 'C';
            switch (sigType) {
                case 0: // B1I D1
                case 4: // B1I D2
                    freqIdx = 2; // B1
                    break;
                case 2: // B3I D1
                case 6: // B3I D2
                    freqIdx = 6; // B3
                    break;
                default:
                    freqIdx = -1;
                    break;
            }
        } else {
            off += 44;
            continue;
        }
        if (freqIdx != -1) {
            SatID sat(sys,prn);
            std::string s_f = std::to_string(freqIdx);
            currentObs.satTypeValueData[sat]["C" + s_f] = codeLocked ? psr : 0;
            currentObs.satTypeValueData[sat]["L" + s_f] = phaseLocked ? adr : 0;
            currentObs.satTypeValueData[sat]["D" + s_f] = static_cast<double>(doppler);
            currentObs.satTypeValueData[sat]["S" + s_f] = static_cast<double>(cn0);
        }
        off += 44;
    }
    return !currentObs.satTypeValueData.empty();
}

bool OEM7Reader::parseGpsEphem(const std::vector<uint8_t> &message) {
    GPSEphem eph;
    int off = currentHeader.hLen;
    eph.prn = static_cast<int>(U4(&message[off])) & 0xFF;
    off += 4;
    off += 8; // tow skip
    eph.health = U4(&message[off]);
    off += 4;
    off += 8; // iode skip
    eph.week = U4(&message[off]);
    off += 4;
    off += 4; // zWeek skip
    eph.toe = R8(&message[off]);
    off += 8;
    const double A = R8(&message[off]);
    eph.rootA = sqrt(A);
    off += 8;
    eph.dn = R8(&message[off]);
    off += 8;
    eph.m0 = R8(&message[off]);
    off += 8;
    eph.e = R8(&message[off]);
    off += 8;
    eph.omega = R8(&message[off]);
    off += 8;
    eph.cuc = R8(&message[off]);
    off += 8;
    eph.cus = R8(&message[off]);
    off += 8;
    eph.crc = R8(&message[off]);
    off += 8;
    eph.crs = R8(&message[off]);
    off += 8;
    eph.cic = R8(&message[off]);
    off += 8;
    eph.cis = R8(&message[off]);
    off += 8;
    eph.i0 = R8(&message[off]);
    off += 8;
    eph.idot = R8(&message[off]);
    off += 8;
    eph.omega0 = R8(&message[off]);
    off += 8;
    eph.omegaDot = R8(&message[off]);
    off += 12;
    eph.toc = R8(&message[off]);
    off += 8;
    eph.tgd = R8(&message[off]);
    off += 8;
    eph.a0 = R8(&message[off]);
    off += 8;
    eph.a1 = R8(&message[off]);
    off += 8;
    eph.a2 = R8(&message[off]);
    off += 8; //NOLINT
    latestGps[eph.prn] = std::move(eph);
    return true;
}

bool OEM7Reader::parseBdsEphem(const std::vector<uint8_t> &message) {
    BDSEphem eph;
    int off = currentHeader.hLen;
    eph.prn = static_cast<int>(U4(&message[off])) & 0xFF;
    off += 4;
    eph.week = U4(&message[off]);
    off += 4;
    eph.ura = R8(&message[off]);
    off += 8;
    eph.health = U4(&message[off]);
    off += 4;
    eph.tgd1 = R8(&message[off]);
    off += 8;
    eph.tgd2 = R8(&message[off]);
    off += 8;
    eph.AODC = U4(&message[off]);
    off += 4;
    eph.toc = U4(&message[off]);
    off += 4;
    eph.a0 = R8(&message[off]);
    off += 8;
    eph.a1 = R8(&message[off]);
    off += 8;
    eph.a2 = R8(&message[off]);
    off += 8;
    eph.AODE = U4(&message[off]);
    off += 4;
    eph.toe = U4(&message[off]);
    off += 4;
    eph.rootA = R8(&message[off]);
    off += 8;
    eph.e = R8(&message[off]);
    off += 8;
    eph.omega = R8(&message[off]);
    off += 8;
    eph.dn = R8(&message[off]);
    off += 8;
    eph.m0 = R8(&message[off]);
    off += 8;
    eph.omega0 = R8(&message[off]);
    off += 8;
    eph.omegaDot = R8(&message[off]);
    off += 8;
    eph.i0 = R8(&message[off]);
    off += 8;
    eph.idot = R8(&message[off]);
    off += 8;
    eph.cuc = R8(&message[off]);
    off += 8;
    eph.cus = R8(&message[off]);
    off += 8;
    eph.crc = R8(&message[off]);
    off += 8;
    eph.crs = R8(&message[off]);
    off += 8;
    eph.cic = R8(&message[off]);
    off += 8;
    eph.cis = R8(&message[off]);
    off += 8; //NOLINT
    latestBds[eph.prn] = std::move(eph);
    return true;
}

bool OEM7Reader::parseBestPos(const std::vector<uint8_t> &message) {
    int off = currentHeader.hLen;

    if (message.size() < off + 20) {
        return false;
    }

    const int solStatus = static_cast<int>(U4(&message[off]));
    off += 4;
    //const int posType = static_cast<int>(U4(&message[off]));
    off += 4;

    // Novatel 使用小端序存储 double
    const double lat_deg = R8(&message[off]);
    off += 8;
    const double lon_deg = R8(&message[off]);
    off += 8;
    const double hgt = R8(&message[off]);

    if (solStatus == 0 && lat_deg > 0 && lat_deg < 90 && lon_deg > 0 && lon_deg < 180) {
        const double lat_rad = lat_deg * DEG_TO_RAD;
        const double lon_rad = lon_deg * DEG_TO_RAD;
        currentObs.antennaPosition = BLHtoXYZ({lat_rad, lon_rad, hgt}, Frame::WGS84);
        return true;
    }

    return false;
}

bool OEM7Reader::getNextEpoch(ObsData &obs) {
    while (true) {
        std::vector<uint8_t> message;
        if (!getNextMessage(message)) return false;
        if (parseMessage(message)) {
            if (!currentObs.satTypeValueData.empty()) {
                obs = std::move(currentObs);
                for (const auto &[prn, eph]: latestGps) {
                    obs.satEphemerisData[SatID('G', prn)] = const_cast<GPSEphem *>(&eph);
                }
                for (const auto &[prn, eph]: latestBds) {
                    obs.satEphemerisData[SatID('C', prn)] = const_cast<BDSEphem *>(&eph);
                }
                // currentObs.satEphemerisData.clear();
                currentObs.satTypeValueData.clear(); // 交付后清空，防止重复触发
                return true;
            }
        }
    }
}
