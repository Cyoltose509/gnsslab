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

    buffer_.clear();
    uint8_t tmp[4096];
    while (ifs.read(reinterpret_cast<char*>(tmp), sizeof(tmp)) || ifs.gcount() > 0) {
        auto readcnt = ifs.gcount();
        buffer_.insert(buffer_.end(), tmp, tmp + readcnt);
    }
    ifs.close();
    return !buffer_.empty();
}

void OEM7Reader::close() {
    buffer_.clear();
}

bool OEM7Reader::getNextMessage(std::vector<uint8_t>& message) {
    const size_t headerLen = 28;
    while (buffer_.size() >= 3) {
        size_t start = 0;
        bool found = false;
        for (; start <= buffer_.size() - 3; ++start) {
            if (buffer_[start] == 0xAA && buffer_[start + 1] == 0x44 && buffer_[start + 2] == 0x12) {
                found = true;
                break;
            }
        }

        if (!found) {
            buffer_.clear();
            return false;
        }

        if (start > 0) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + start);
        }

        if (buffer_.size() < headerLen) return false; // 剩余数据不足以读取头部

        size_t hlen = buffer_[3];
        size_t msgLen = U2(&buffer_[8]);
        size_t totalLen = hlen + msgLen + 4;

        if (buffer_.size() < totalLen) {
            // 发现同步头但剩余数据不足一条完整消息，跳过此同步头继续搜索
            buffer_.erase(buffer_.begin(), buffer_.begin() + 3);
            continue;
        }

        message.assign(buffer_.begin(), buffer_.begin() + totalLen);
        buffer_.erase(buffer_.begin(), buffer_.begin() + totalLen);
        return true;
    }
    return false;
}

unsigned int OEM7Reader::crc32_calc(const unsigned char *buff, int len) {
    unsigned int crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= buff[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
            else crc >>= 1;
        }
    }
    return crc;
}

bool OEM7Reader::crcExam(const std::vector<uint8_t>& message) {
    if (message.size() < 4) return false;
    unsigned int msg_len = message.size() - 4;
    unsigned int calc = crc32_calc(message.data(), msg_len);
    unsigned int expected = U4(message.data() + msg_len);
    return calc == expected;
}

bool OEM7Reader::parseMessage(const std::vector<uint8_t>& message) {
    if (!crcExam(message)) return false;

    currentHeader.hlen = message[3];
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

bool OEM7Reader::parseRange(const std::vector<uint8_t>& message) {
    currentObs_.satTypeValueData.clear();
    const WeekSecond ws(currentHeader.week, currentHeader.ms * 0.001, TimeSystem::GPS);
    WeekSecond2CommonTime(ws, currentObs_.epoch);

    int off = currentHeader.hlen;
    uint32_t numObs = U4(&message[off]);
    off += 4;

    for (uint32_t i = 0; i < numObs; ++i) {
        if (off + 44 > message.size()) break;
        uint16_t prn = U2(&message[off + 0]);
        double psr = R8(&message[off + 4]);
        double adr = R8(&message[off + 16]);
        float doppler = R4(&message[off + 28]);
        float cn0 = R4(&message[off + 32]);
        uint32_t trackingStatus = U4(&message[off + 40]);

        int sysInt = (trackingStatus >> 16) & 7;
        int sigType = (trackingStatus >> 21) & 0x1F;

        std::string sys;
        int freqIdx = -1;

        if (sysInt == 0) { // GPS
            sys = "G";
            if (sigType == 0) freqIdx = 1;      // L1C/A
            else if (sigType == 9) freqIdx = 2; // L2P(Y)
        } else if (sysInt == 4) { // BDS
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
            currentObs_.satTypeValueData[sat]["C" + s_f] = psr;
            currentObs_.satTypeValueData[sat]["L" + s_f] = adr;
            currentObs_.satTypeValueData[sat]["D" + s_f] = (double)doppler;
            currentObs_.satTypeValueData[sat]["S" + s_f] = (double)cn0;
        }
        off += 44;
    }
    return !currentObs_.satTypeValueData.empty();
}

bool OEM7Reader::parseGpsEphem(const std::vector<uint8_t>& message) {
    GPSEphem eph;
    int off = currentHeader.hlen;
    eph.PRN = U4(&message[off]) & 0xFF; off += 4;
    off += 8; // tow skip
    eph.health = U4(&message[off]); off += 4;
    off += 8; // iode skip
    eph.week = U4(&message[off]); off += 4;
    off += 4; // zWeek skip
    eph.toe = R8(&message[off]); off += 8;
    double A = R8(&message[off]); eph.RootA = sqrt(A); off += 8;
    eph.dn = R8(&message[off]); off += 8;
    eph.M0 = R8(&message[off]); off += 8;
    eph.e = R8(&message[off]); off += 8;
    eph.omega = R8(&message[off]); off += 8;
    eph.cuc = R8(&message[off]); off += 8;
    eph.cus = R8(&message[off]); off += 8;
    eph.crc = R8(&message[off]); off += 8;
    eph.crs = R8(&message[off]); off += 8;
    eph.cic = R8(&message[off]); off += 8;
    eph.cis = R8(&message[off]); off += 8;
    eph.i0 = R8(&message[off]); off += 8;
    eph.idot = R8(&message[off]); off += 8;
    eph.Omega0 = R8(&message[off]); off += 8;
    eph.Omegadot = R8(&message[off]); off += 12;
    eph.toc = R8(&message[off]); off += 8;
    eph.tgd = R8(&message[off]); off += 8;
    eph.a0 = R8(&message[off]); off += 8;
    eph.a1 = R8(&message[off]); off += 8;
    eph.a2 = R8(&message[off]); off += 8;

    latestGps.erase(eph.PRN);
    latestGps.emplace(eph.PRN, eph);
    return true;
}

bool OEM7Reader::parseBdsEphem(const std::vector<uint8_t>& message) {
    BDSEphem eph;
    int off = currentHeader.hlen;
    eph.PRN = U4(&message[off]) & 0xFF; off += 4;
    eph.week = U4(&message[off]); off += 4;
    eph.URA = R8(&message[off]); off += 8;
    eph.health = U4(&message[off]); off += 4;
    eph.tgd1 = R8(&message[off]); off += 8;
    eph.tgd2 = R8(&message[off]); off += 8;
    eph.AODC = U4(&message[off]); off += 4;
    eph.toc = U4(&message[off]); off += 4;
    eph.a0 = R8(&message[off]); off += 8;
    eph.a1 = R8(&message[off]); off += 8;
    eph.a2 = R8(&message[off]); off += 8;
    eph.AODE = U4(&message[off]); off += 4;
    eph.toe = U4(&message[off]); off += 4;
    eph.RootA = R8(&message[off]); off += 8;
    eph.e = R8(&message[off]); off += 8;
    eph.omega = R8(&message[off]); off += 8;
    eph.dn = R8(&message[off]); off += 8;
    eph.M0 = R8(&message[off]); off += 8;
    eph.Omega0 = R8(&message[off]); off += 8;
    eph.Omegadot = R8(&message[off]); off += 8;
    eph.i0 = R8(&message[off]); off += 8;
    eph.idot = R8(&message[off]); off += 8;
    eph.cuc = R8(&message[off]); off += 8;
    eph.cus = R8(&message[off]); off += 8;
    eph.crc = R8(&message[off]); off += 8;
    eph.crs = R8(&message[off]); off += 8;
    eph.cic = R8(&message[off]); off += 8;
    eph.cis = R8(&message[off]); off += 8;

    latestBds.erase(eph.PRN);
    latestBds.emplace(eph.PRN, eph);
    return true;
}

bool OEM7Reader::getNextEpoch(ObsData &obs) {
    while (true) {
        std::vector<uint8_t> message;
        if (!getNextMessage(message)) return false;
        if (parseMessage(message)) {
            if (!currentObs_.satTypeValueData.empty()) {
                obs = currentObs_;
                currentObs_.satTypeValueData.clear(); // 交付后清空，防止重复触发
                return true;
            }
        }
    }
}
