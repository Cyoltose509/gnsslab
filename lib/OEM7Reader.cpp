#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <cstring>
#include <algorithm>
#include <cmath>

#include "TimeConvert.h"
#include "GnssStruct.h"
#include "NavEphGPS.h"
#include "OEM7Reader.h"

#define ID_RANGE        43
#define ID_GPSEPHEM     7
#define ID_BDSEPHEMRIS  1696
#define POLYCRC32 0xEDB88320u

bool OEM7Reader::open(const std::string &filename) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) return false;

    buffer.clear();
    uint8_t tmp[4096];
    while (ifs.read(reinterpret_cast<char *>(tmp), sizeof(tmp)) || ifs.gcount() > 0) {
        buffer.insert(buffer.end(), tmp, tmp + ifs.gcount());
    }
    ifs.close();
    return !buffer.empty();
}

void OEM7Reader::close() {
    buffer.clear();
}

bool OEM7Reader::getNextMessage(std::vector<uint8_t> &message) {
    while (buffer.size() >= 3) {
        constexpr size_t headerLen = 28;
        long long start = 0;
        bool found = false;
        for (; start <= buffer.size() - 3; ++start) {
            if (buffer[start] == 0xAA && buffer[start + 1] == 0x44 && buffer[start + 2] == 0x12) {
                found = true;
                break;
            }
        }

        if (!found) {
            buffer.clear();
            return false;
        }

        if (start > 0) {
            buffer.erase(buffer.begin(), buffer.begin() + start);
        }

        if (buffer.size() < headerLen) return false; // 剩余数据不足以读取头部

        const auto hlen = buffer[3];
        const auto msgLen = U2(&buffer[8]);
        const auto totalLen = hlen + msgLen + 4;

        if (buffer.size() < totalLen) {
            // 发现同步头但剩余数据不足一条完整消息，跳过此同步头继续搜索
            buffer.erase(buffer.begin(), buffer.begin() + 3);
            continue;
        }

        message.assign(buffer.begin(), buffer.begin() + totalLen);
        buffer.erase(buffer.begin(), buffer.begin() + totalLen);
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

        std::string sys;
        int freqIdx = -1;

        if (sysInt == 0) {
            // GPS
            sys = "G";
            if (sigType == 0) freqIdx = 1; // L1C/A
            else if (sigType == 9) freqIdx = 2; // L2P(Y)
        } else if (sysInt == 4) {
            // BDS
            sys = "C";
            if (sigType == 0 || sigType == 4) freqIdx = 2; // B1I (Reference uses freq2 as B1I)
            else if (sigType == 2 || sigType == 6) freqIdx = 6; // B3I
        } else {
            off += 44;
            continue;
        }
        if (freqIdx != -1) {
            SatID sat;
            sat.system = sys;
            sat.id = prn;
            std::string s_f = std::to_string(freqIdx);
            currentObs.satTypeValueData[sat]["C" + s_f] = psr;
            currentObs.satTypeValueData[sat]["L" + s_f] = adr;
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
    off += 8;
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
    off += 8;
    latestBds[eph.prn] = std::move(eph);
    return true;
}

bool OEM7Reader::getNextEpoch(ObsData &obs) {
    while (true) {
        std::vector<uint8_t> message;
        if (!getNextMessage(message)) return false;
        if (parseMessage(message)) {
            if (!currentObs.satTypeValueData.empty()) {
                obs = currentObs;
                currentObs.satTypeValueData.clear(); // 交付后清空，防止重复触发
                return true;
            }
        }
    }
}
