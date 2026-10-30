#include <fstream>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <iterator>


#include "TimeConvert.h"
#include "GnssStruct.h"
#include "OEM7Reader.h"
#include "Const.h"

#include "Log.h"
#include "MathUtils.h"
#include "StringUtils.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#define ID_RANGE        43
#define ID_GPSEPHEM     7
#define ID_BDSEPHEMRIS  1696
#define ID_GLOEPHEM     723
#define ID_GALEPHEM     1122
#define ID_BESTPOS      42
#define ID_RAWIMU       268
#define ID_RAWIMUS      325
#define ID_RAWIMUXA     1461
#define ID_RAWIMUSX     1462
#define ID_RAWIMU_COMPACT 0x1313
#define POLYCRC32 0xEDB88320u

bool OEM7Reader::open(const std::string &filename) {
    std::ifstream ifs(acpToWide(filename), std::ios::binary | std::ios::ate);
    if (!ifs) return false;


    fileSize = static_cast<size_t>(ifs.tellg());
    ifs.seekg(0, std::ios::beg);

    buffer.clear();
    bufferIndex = 0;
    mImuLog.clear();
    mInsLayout = InsRecordLayout();
    mImuType = -1;
    mInsDiag.clear();
    mInsDiagLogged.store(false);
    uint8_t tmp[4096];
    while (ifs.read(reinterpret_cast<char *>(tmp), sizeof(tmp)) || ifs.gcount() > 0) {
        buffer.insert(buffer.end(), tmp, tmp + ifs.gcount());
    }
    ifs.close();
    mEphSnap = std::make_shared<EphemerisTable>();


    probeInsLayout();
    if (mImuType < 0) probeImuTypeFromShortHeader();
    if (mImuType >= 0) mInsDiag.clear();
    return !buffer.empty();
}

void OEM7Reader::close() {
    buffer.clear();
    bufferIndex = 0;
    mEphSnap.reset();
    mInsLayout = InsRecordLayout();
}

static constexpr OEM7Reader::InsRecordLayout kInsLayoutCandidates[] = {

    {true, 56, 14, 13, 8, 16, 24, 28, "RAWIMUSX 字段集（含 IMU Type）"},

    {true, 56, 12, -1, 8, 16, 24, 28, "RAWIMUS 字段集（无 IMU Type）"},
};

bool OEM7Reader::acceptInsRecord(const size_t start, const InsRecordLayout &layout) const {
    if (start + static_cast<size_t>(layout.recLen) > buffer.size()) return false;

    const auto body = static_cast<size_t>(layout.recLen) - 4;
    unsigned int calc = 0;
    for (size_t i = 0; i < body; ++i) {
        calc ^= buffer[start + i];
        for (int j = 0; j < 8; ++j) {
            if (calc & 1) calc = calc >> 1 ^ POLYCRC32;
            else calc >>= 1;
        }
    }
    if (calc != Math::U4(&buffer[start + body])) return false;

    if (const int week = Math::U2(&buffer[start + layout.weekOffset]); week < 1000 || week > 3000) return false;

    double sec = 0.0;
    std::memcpy(&sec, &buffer[start + layout.secOffset], sizeof(double));
    if (!(sec >= 0.0 && sec <= FULL_WEEK)) return false;
    if (const double msSec = Math::U4(&buffer[start + layout.towMsOffset]) * 0.001; std::fabs(msSec - sec) > 2e-3) return false;

    if (layout.imuTypeOffset >= 0 && buffer[start + layout.imuTypeOffset] == 0) return false;

    return true;
}

bool OEM7Reader::probeInsLayout() {
    constexpr size_t kMaxProbe = 2000;
    constexpr int kCandidates = std::size(kInsLayoutCandidates);
    size_t probes[kCandidates] = {};
    int pass[kCandidates] = {};
    int firstType = -1;

    size_t i = 0;
    size_t considered = 0;
    while (i + 56 <= buffer.size() && considered < kMaxProbe) {
        if (buffer[i] == 0xAA && buffer[i + 1] == 0x44 && buffer[i + 2] == 0x13 && buffer[i + 3] == 0x28) {
            ++considered;
            for (int c = 0; c < kCandidates; ++c) {
                ++probes[c];
                if (acceptInsRecord(i, kInsLayoutCandidates[c])) {
                    ++pass[c];
                    if (firstType < 0 && kInsLayoutCandidates[c].imuTypeOffset >= 0)
                        firstType = buffer[i + kInsLayoutCandidates[c].imuTypeOffset];
                }
            }
            i += 56;
        } else {
            ++i;
        }
    }

    if (considered == 0) {
        mInsDiag = "文件中没有 0xAA4413 紧凑 IMU 记录（可能只有标准 OEM7 日志）";
        return false;
    }

    int best = -1;
    for (int c = 0; c < kCandidates; ++c) {
        if (probes[c] > 0 && pass[c] * 100 >= probes[c] * 95) {
            if (best < 0 || pass[c] > pass[best]) best = c;
        }
    }

    if (best < 0) {
        mInsDiag = "发现 " + std::to_string(considered) +
                   " 条 0xAA4413 候选记录，但没有一种已知布局能通过 CRC/时间一致性校验"
                   "（已试 " + std::to_string(kCandidates) + " 种）";
        return false;
    }

    mInsLayout = kInsLayoutCandidates[best];
    mInsLayout.valid = true;
    mImuType = firstType;
    return true;
}

void OEM7Reader::probeImuTypeFromShortHeader() {
    size_t scanned = 0;
    for (size_t i = 0; i + 12 <= buffer.size() && scanned < 2000; ++i) {
        if (!(buffer[i] == 0xAA && buffer[i + 1] == 0x44 && buffer[i + 2] == 0x12)) continue;
        if (const int msgId = Math::U2(&buffer[i + 4]); msgId != ID_RAWIMUXA && msgId != ID_RAWIMUSX) continue;
        const int hLen = buffer[i + 3] == 28 ? 28 : 12;
        if (i + static_cast<size_t>(hLen) + 2 > buffer.size()) continue;
        ++scanned;
        if (const uint8_t type = buffer[i + hLen + 1]; type != 0) {
            mImuType = type;
            return;
        }
    }
}

bool OEM7Reader::getNextMessage(std::vector<uint8_t> &message, [[maybe_unused]] const bool block) {
    while (buffer.size() - bufferIndex >= 3) {
        size_t start = bufferIndex;
        bool found = false;
        uint8_t sync = 0;
        for (; start + 2 < buffer.size(); ++start) {
            if (buffer[start] == 0xAA && buffer[start + 1] == 0x44 &&
                (buffer[start + 2] == 0x12 || buffer[start + 2] == 0x13)) {
                found = true;
                sync = buffer[start + 2];
                break;
            }
        }

        if (!found) {
            buffer.clear();
            bufferIndex = 0;
            return false;
        }

        bufferIndex = start;

        if (buffer.size() - bufferIndex < 12)
            return false;

        if (sync == 0x13) {
            if (mInsLayout.valid && acceptInsRecord(bufferIndex, mInsLayout)) {
                message.assign(buffer.begin() + static_cast<long long>(bufferIndex),
                               buffer.begin() + static_cast<long long>(bufferIndex) + mInsLayout.recLen);
                message[4] = static_cast<uint8_t>(ID_RAWIMU_COMPACT & 0xFF);
                message[5] = static_cast<uint8_t>(ID_RAWIMU_COMPACT >> 8 & 0xFF);
                bufferIndex += mInsLayout.recLen;
                return true;
            }
            if (!mInsLayout.valid && !mInsDiagLogged.exchange(true) && !mInsDiag.empty())
                LOG_ERROR << "OEM7Reader: " << mInsDiag;
            bufferIndex += 3;
            continue;
        }

        const auto b3 = static_cast<int>(buffer[bufferIndex + 3]);
        const bool longHeader = b3 == 28;
        const int hlen = longHeader ? 28 : 12;
        const int msgLen = longHeader ? static_cast<int>(Math::U2(&buffer[bufferIndex + 8])) : b3;
        const int totalLen = hlen + msgLen + 4;

        if (buffer.size() - bufferIndex < static_cast<size_t>(totalLen) || totalLen < hlen + 4) {
            bufferIndex += 3;
            continue;
        }

        message.assign(buffer.begin() + static_cast<long long>(bufferIndex),
                       buffer.begin() + static_cast<long long>(bufferIndex) + totalLen);
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
    return calc == Math::U4(buff + msg_len);
}

bool OEM7Reader::parseGNSSMessage(const std::vector<uint8_t> &message) {
    if (!crcExam(message)) return false;

    const bool longHeader = message[3] == 28;
    currentHeader.hLen = longHeader ? 28 : 12;
    currentHeader.type = Math::U2(&message[4]);
    currentHeader.length = longHeader ? static_cast<int>(Math::U2(&message[8])) : message[3];
    currentHeader.week = static_cast<int>(longHeader ? Math::U2(&message[14]) : Math::U2(&message[6]));
    currentHeader.ms = longHeader ? Math::I4(&message[16]) : Math::I4(&message[8]);

    switch (currentHeader.type) {
        case ID_RANGE:
            return parseRange(message);
        case ID_GPSEPHEM:
            return parseGpsEphem(message);
        case ID_BDSEPHEMRIS:
            return parseBdsEphem(message);
        case ID_GLOEPHEM:
            return parseGloEphem(message);
        case ID_GALEPHEM:
            return parseGalEph(message);
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
    const uint32_t numObs = Math::U4(&message[off]);
    off += 4;

    for (uint32_t i = 0; i < numObs; ++i) {
        if (off + 44 > message.size()) break;
        const auto prnRaw = Math::U2(&message[off + 0]);
        const auto gfrq = Math::U2(&message[off + 2]);
        const auto psr = Math::R8(&message[off + 4]);
        const auto adr = Math::R8(&message[off + 16]);
        const auto doppler = Math::R4(&message[off + 28]);
        const auto cn0 = Math::R4(&message[off + 32]);
        const auto trackingStatus = Math::U4(&message[off + 40]);
        const auto sysInt = static_cast<int>(trackingStatus >> 16) & 7;
        const auto sigType = static_cast<int>(trackingStatus >> 21) & 0x1F;
        const auto codeLocked = static_cast<int>(trackingStatus >> 12) & 1;
        const auto phaseLocked = static_cast<int>(trackingStatus >> 10) & 1;

        char sys = 0;
        int freqIdx;
        int chan = 0;
        int prn = prnRaw;

        switch (sysInt) {
            case 0:
                sys = 'G';
                switch (sigType) {
                    case 0: freqIdx = 1;
                        break;
                    case 9: freqIdx = 2;
                        break;
                    default: freqIdx = -1;
                        break;
                }
                break;
            case 1:
                sys = 'R';
                prn = prnRaw - 37;
                chan = gfrq - 8;
                switch (sigType) {
                    case 0: freqIdx = 1;
                        break;
                    case 1:
                    case 5: freqIdx = 2;
                        break;
                    case 6: freqIdx = 3;
                        break;
                    default: freqIdx = -1;
                        break;
                }
                break;
            case 3:
                sys = 'E';
                switch (sigType) {
                    case 2: freqIdx = 1;
                        break;
                    case 6:
                    case 7: freqIdx = 6;
                        break;
                    case 12: freqIdx = 5;
                        break;
                    case 17: freqIdx = 7;
                        break;
                    case 20: freqIdx = 8;
                        break;
                    default: freqIdx = -1;
                        break;
                }
                break;
            case 4:
                sys = 'C';
                switch (sigType) {
                    case 0:
                    case 4:
                        freqIdx = 2;
                        break;
                    case 2:
                    case 6:
                        freqIdx = 6;
                        break;
                    default: freqIdx = -1;
                        break;
                }
                break;
            default:
                off += 44;
                continue;
        }

        if (freqIdx != -1) {
            SatID sat(sys, prn, chan);
            std::string s_f = std::to_string(freqIdx);
            if (codeLocked) currentObs.satTypeValueData[sat]["C" + s_f] = psr;
            {
                const std::string ctype = "C" + s_f;
                if (std::vector<std::string> &vec = availableTypes[sys]; std::find(vec.begin(), vec.end(), ctype) == vec.end())
                    vec.push_back(ctype);
            }
            const double freq = sys == 'R'
                                    ? getGlonassFreq(static_cast<char>('0' + freqIdx), chan)
                                    : getFreq(sys, freqIdx);
            const double lam = freq > 0.0 ? C_MPS / freq : 0.0;
            currentObs.satTypeValueData[sat]["L" + s_f] = phaseLocked && lam > 0.0 ? -adr * lam : 0.0;
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
    eph.prn = static_cast<int>(Math::U4(&message[off])) & 0xFF;
    off += 4;
    off += 8;
    eph.health = Math::U4(&message[off]);
    off += 4;
    off += 8;
    eph.week = Math::U4(&message[off]);
    off += 4;
    off += 4;
    eph.toe = Math::R8(&message[off]);
    off += 8;
    const double A = Math::R8(&message[off]);
    eph.rootA = sqrt(A);
    off += 8;
    eph.dn = Math::R8(&message[off]);
    off += 8;
    eph.m0 = Math::R8(&message[off]);
    off += 8;
    eph.e = Math::R8(&message[off]);
    off += 8;
    eph.omega = Math::R8(&message[off]);
    off += 8;
    eph.cuc = Math::R8(&message[off]);
    off += 8;
    eph.cus = Math::R8(&message[off]);
    off += 8;
    eph.crc = Math::R8(&message[off]);
    off += 8;
    eph.crs = Math::R8(&message[off]);
    off += 8;
    eph.cic = Math::R8(&message[off]);
    off += 8;
    eph.cis = Math::R8(&message[off]);
    off += 8;
    eph.i0 = Math::R8(&message[off]);
    off += 8;
    eph.idot = Math::R8(&message[off]);
    off += 8;
    eph.omega0 = Math::R8(&message[off]);
    off += 8;
    eph.omegaDot = Math::R8(&message[off]);
    off += 12;
    eph.toc = Math::R8(&message[off]);
    off += 8;
    eph.tgd = Math::R8(&message[off]);
    off += 8;
    eph.a0 = Math::R8(&message[off]);
    off += 8;
    eph.a1 = Math::R8(&message[off]);
    off += 8;
    eph.a2 = Math::R8(&message[off]);
    off += 8;

    storeEph(&EphemerisTable::gps, eph.prn, std::move(eph));
    return true;
}

bool OEM7Reader::parseBdsEphem(const std::vector<uint8_t> &message) {
    BDSEphem eph;
    int off = currentHeader.hLen;
    eph.prn = static_cast<int>(Math::U4(&message[off])) & 0xFF;
    off += 4;
    eph.week = Math::U4(&message[off]);
    off += 4;
    eph.ura = Math::R8(&message[off]);
    off += 8;
    eph.health = Math::U4(&message[off]);
    off += 4;
    eph.tgd1 = Math::R8(&message[off]);
    off += 8;
    eph.tgd2 = Math::R8(&message[off]);
    off += 8;
    eph.AODC = Math::U4(&message[off]);
    off += 4;
    eph.toc = Math::U4(&message[off]);
    off += 4;
    eph.a0 = Math::R8(&message[off]);
    off += 8;
    eph.a1 = Math::R8(&message[off]);
    off += 8;
    eph.a2 = Math::R8(&message[off]);
    off += 8;
    eph.AODE = Math::U4(&message[off]);
    off += 4;
    eph.toe = Math::U4(&message[off]);
    off += 4;
    eph.rootA = Math::R8(&message[off]);
    off += 8;
    eph.e = Math::R8(&message[off]);
    off += 8;
    eph.omega = Math::R8(&message[off]);
    off += 8;
    eph.dn = Math::R8(&message[off]);
    off += 8;
    eph.m0 = Math::R8(&message[off]);
    off += 8;
    eph.omega0 = Math::R8(&message[off]);
    off += 8;
    eph.omegaDot = Math::R8(&message[off]);
    off += 8;
    eph.i0 = Math::R8(&message[off]);
    off += 8;
    eph.idot = Math::R8(&message[off]);
    off += 8;
    eph.cuc = Math::R8(&message[off]);
    off += 8;
    eph.cus = Math::R8(&message[off]);
    off += 8;
    eph.crc = Math::R8(&message[off]);
    off += 8;
    eph.crs = Math::R8(&message[off]);
    off += 8;
    eph.cic = Math::R8(&message[off]);
    off += 8;
    eph.cis = Math::R8(&message[off]);
    off += 8; //NOLINT
    storeEph(&EphemerisTable::bds, eph.prn, std::move(eph));
    return true;
}

bool OEM7Reader::parseGloEphem(const std::vector<uint8_t> &message) {
    if (message.size() < static_cast<size_t>(currentHeader.hLen) + 144) return false;

    GlonassEphem eph;
    int off = currentHeader.hLen;
    const int rawPrn = Math::U2(&message[off]);
    off += 2;
    const int frq = Math::U2(&message[off]) - 7;
    off += 2; // FCN
    off += 2; // (p+4 未用)
    const int week = Math::U2(&message[off]);
    off += 2;
    const double tow = std::floor(Math::U4(&message[off]) * 0.001 + 0.5);
    off += 4; // ms→s 取整
    off += 4; // toff
    off += 4; // (p+16 未用)
    off += 4; // iode（GlonassEph 无对应成员，跳过）
    eph.health = Math::U4(&message[off]) < 4 ? 0u : 1u;
    off += 4;
    eph.X = Math::R8(&message[off]) * 0.001;
    off += 8; // m→km
    eph.Y = Math::R8(&message[off]) * 0.001;
    off += 8;
    eph.Z = Math::R8(&message[off]) * 0.001;
    off += 8;
    eph.vX = Math::R8(&message[off]) * 0.001;
    off += 8;
    eph.vY = Math::R8(&message[off]) * 0.001;
    off += 8;
    eph.vZ = Math::R8(&message[off]) * 0.001;
    off += 8;
    eph.aX = Math::R8(&message[off]) * 0.001;
    off += 8;
    eph.aY = Math::R8(&message[off]) * 0.001;
    off += 8;
    eph.aZ = Math::R8(&message[off]) * 0.001;
    off += 8;
    eph.tauN = -Math::R8(&message[off]);
    off += 8; // 取负，与 RINEX 约定一致
    eph.tauD = Math::R8(&message[off]);
    off += 8;
    off += 8; // gamn（未用）
    off += 4; // tof（相对量，未用）
    off += 8; // (p+128 未用)NOLINT

    eph.prn = rawPrn - 37; // 37+slot → slot(1..24)
    eph.freqNum = frq;

    CommonTime toeGps;
    WeekSecond wsGps(week, tow);
    WeekSecond2CommonTime(wsGps, toeGps);
    convertTimeSystem(toeGps, TimeSystem::UTC);
    eph.toeCommon = toeGps;
    eph.week = week;
    eph.toe = tow;

    storeEph(&EphemerisTable::glo, eph.prn, std::move(eph));
    return true;
}

bool OEM7Reader::parseGalEph(const std::vector<uint8_t> &message) {
    if (message.size() < static_cast<size_t>(currentHeader.hLen) + 220) return false;

    GalileoEphem eph;
    int off = currentHeader.hLen;
    eph.prn = static_cast<int>(Math::U4(&message[off])) & 0xFF;
    off += 4;
    off += 4; // rcv_fnav（GAL 接收标志，下游未用）
    const int rcv_inav = static_cast<int>(Math::U4(&message[off])) & 1;
    off += 4;
    off += 1; // svh_e1b（下游未用）
    off += 1; // svh_e5a（下游未用）
    const uint8_t svh_e5b = message[off] & 3;
    off += 1;
    off += 1; // dvs_e1b（下游未用）
    off += 1; // dvs_e5a（下游未用）
    const uint8_t dvs_e5b = message[off] & 1;
    off += 1;
    eph.ura = static_cast<double>(message[off]);
    off += 1; // SISA 指数
    off += 1; // 填充
    off += 4; // IODNav（GalEph 无对应成员，跳过）
    eph.toe = Math::U4(&message[off]);
    off += 4; // Toe (SOW, GAL 时间)
    eph.rootA = Math::R8(&message[off]);
    off += 8; // sqrtA
    eph.dn = Math::R8(&message[off]);
    off += 8; // deltaN
    eph.m0 = Math::R8(&message[off]);
    off += 8;
    eph.e = Math::R8(&message[off]);
    off += 8;
    eph.omega = Math::R8(&message[off]);
    off += 8; // omg
    eph.cuc = Math::R8(&message[off]);
    off += 8;
    eph.cus = Math::R8(&message[off]);
    off += 8;
    eph.crc = Math::R8(&message[off]);
    off += 8;
    eph.crs = Math::R8(&message[off]);
    off += 8;
    eph.cic = Math::R8(&message[off]);
    off += 8;
    eph.cis = Math::R8(&message[off]);
    off += 8;
    eph.i0 = Math::R8(&message[off]);
    off += 8;
    eph.idot = Math::R8(&message[off]);
    off += 8;
    eph.omega0 = Math::R8(&message[off]);
    off += 8; // OMG0
    eph.omegaDot = Math::R8(&message[off]);
    off += 8; // OMGd
    const int toc_fnav = static_cast<int>(Math::U4(&message[off]));
    off += 4;
    const double af0_fnav = Math::R8(&message[off]);
    off += 8;
    const double af1_fnav = Math::R8(&message[off]);
    off += 8;
    const double af2_fnav = Math::R8(&message[off]);
    off += 8;
    const int toc_inav = static_cast<int>(Math::U4(&message[off]));
    off += 4;
    const double af0_inav = Math::R8(&message[off]);
    off += 8;
    const double af1_inav = Math::R8(&message[off]);
    off += 8;
    const double af2_inav = Math::R8(&message[off]);
    off += 8;
    off += 8; // tgd[0] BGD E5a-E1（下游未用）
    const double bgd_e5b_e1 = Math::R8(&message[off]); // tgd[1] BGD E5b-E1

    // 优先 I/NAV
    // 仅当接收机收不到 I/NAV 时退回 F/NAV
    const bool useInav = rcv_inav != 0;
    eph.a0 = useInav ? af0_inav : af0_fnav;
    eph.a1 = useInav ? af1_inav : af1_fnav;
    eph.a2 = useInav ? af2_inav : af2_fnav;
    eph.toc = static_cast<double>(useInav ? toc_inav : toc_fnav);
    eph.tgd = bgd_e5b_e1; // I/NAV 用 BGD E5b-E1
    eph.health = static_cast<unsigned int>(svh_e5b) << 1 | dvs_e5b;
    eph.week = currentHeader.week; // Galileo 周号 = GPS 周号

    storeEph(&EphemerisTable::gal, eph.prn, std::move(eph));
    return true;
}

bool OEM7Reader::parseBestPos(const std::vector<uint8_t> &message) {
    int off = currentHeader.hLen;

    if (message.size() < off + 20) {
        return false;
    }

    const int solStatus = static_cast<int>(Math::U4(&message[off]));
    off += 4;
    //const int posType = static_cast<int>(Math::U4(&message[off]));
    off += 4;

    const double lat_deg = Math::R8(&message[off]);
    off += 8;
    const double lon_deg = Math::R8(&message[off]);
    off += 8;
    const double hgt = Math::R8(&message[off]);

    if (solStatus == 0 && lat_deg > 0 && lat_deg < 90 && lon_deg > 0 && lon_deg < 180) {
        const double lat_rad = lat_deg * DEG_TO_RAD;
        const double lon_rad = lon_deg * DEG_TO_RAD;
        currentObs.antennaPosition = BLHToXYZ({lat_rad, lon_rad, hgt}, Frame::WGS84);
        return true;
    }

    return false;
}

bool OEM7Reader::getNextEpoch(ObsData &obs, const bool block) {
    while (true) {
        std::vector<uint8_t> message;
        if (!getNextMessage(message, block)) {
            return false;
        }
        parseImu(message);
        if (parseGNSSMessage(message)) {
            if (!currentObs.satTypeValueData.empty()) {
                obs = std::move(currentObs);
                currentObs.satTypeValueData.clear();
                return true;
            }
        }
    }
}

bool OEM7Reader::parseImu(const std::vector<uint8_t> &message) {
    if (message.size() < 6) return false;
    const int type = Math::U2(&message[4]);

    int week = 0;
    double towSec = 0.0;
    int32_t zacc = 0, nyacc = 0, xacc = 0, zgyro = 0, nygyro = 0, xgyro = 0;

    if (type == ID_RAWIMU || type == ID_RAWIMUS || type == ID_RAWIMUXA || type == ID_RAWIMUSX) {
        if (!crcExam(message)) return false;
        if (message.size() < 12) return false;
        const int hLen = message[3] == 28 ? 28 : 12;
        const size_t off = static_cast<size_t>(hLen) + 16;
        if (message.size() < off + 24) return false;
        if (type == ID_RAWIMUXA || type == ID_RAWIMUSX) {
            week = Math::U2(&message[hLen + 2]);
            if (mImuType < 0) {
                if (const int t = message[hLen + 1]; t > 0) mImuType = t;
            }
        } else {
            week = static_cast<int>(Math::U4(&message[hLen + 0]));
        }
        std::memcpy(&towSec, &message[hLen + 4], sizeof(double));

        zacc = Math::I4(&message[off + 0]);
        nyacc = Math::I4(&message[off + 4]);
        xacc = Math::I4(&message[off + 8]);
        zgyro = Math::I4(&message[off + 12]);
        nygyro = Math::I4(&message[off + 16]);
        xgyro = Math::I4(&message[off + 20]);
    } else if (type == ID_RAWIMU_COMPACT) {
        if (!mInsLayout.valid) return false;
        if (message.size() < static_cast<size_t>(mInsLayout.recLen)) return false;
        if (mImuType < 0 && mInsLayout.imuTypeOffset >= 0) mImuType = message[mInsLayout.imuTypeOffset];
        week = Math::U2(&message[mInsLayout.weekOffset]);
        std::memcpy(&towSec, &message[mInsLayout.secOffset], sizeof(double));
        const int c = mInsLayout.countsOffset;
        zacc = Math::I4(&message[c + 0]);
        nyacc = Math::I4(&message[c + 4]);
        xacc = Math::I4(&message[c + 8]);
        zgyro = Math::I4(&message[c + 12]);
        nygyro = Math::I4(&message[c + 16]);
        xgyro = Math::I4(&message[c + 20]);
    } else {
        return false;
    }

    ImuSample s;
    s.gnssWeek = week;
    s.tow = week * FULL_WEEK + towSec;
    s.ax = xacc;
    s.ay = -nyacc;
    s.az = zacc;
    s.gx = xgyro;
    s.gy = -nygyro;
    s.gz = zgyro;
    mImuLog.push_back(s);
    return true;
}
