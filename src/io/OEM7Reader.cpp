#include <fstream>
#include <vector>
#include <iomanip>
#include <algorithm>


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
#define ID_RAWIMUXA     1461
#define ID_RAWIMUSA     325
#define ID_RAWIMU_COMPACT 0x1313
#define POLYCRC32 0xEDB88320u

bool OEM7Reader::open(const std::string &filename) {
    std::ifstream ifs(acpToWide(filename), std::ios::binary | std::ios::ate);
    if (!ifs) return false;

    // 记录文件总大小用于进度显示
    fileSize = static_cast<size_t>(ifs.tellg());
    ifs.seekg(0, std::ios::beg);

    buffer.clear();
    bufferIndex = 0;
    mImuLog.clear(); // 新文件清空 IMU 累积，使流式 getNextEpoch 也能正确累积本文件 IMU（getImuLog 取之）
    uint8_t tmp[4096];
    while (ifs.read(reinterpret_cast<char *>(tmp), sizeof(tmp)) || ifs.gcount() > 0) {
        buffer.insert(buffer.end(), tmp, tmp + ifs.gcount());
    }
    ifs.close();
    mEphSnap = std::make_shared<EphemerisTable>(); // 流式/批量路径共用的写时复制快照
    return !buffer.empty();
}

void OEM7Reader::close() {
    buffer.clear();
    bufferIndex = 0;
    mEphSnap.reset();
}

bool OEM7Reader::getNextMessage(std::vector<uint8_t> &message) {
    while (buffer.size() - bufferIndex >= 3) {
        // 统一扫描同步头：短头 0xAA 0x44 0x12（标准 OEM7 帧）与长头 0xAA 0x44 0x13（紧凑 IMU 记录）。
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

        // 跳过同步头之前的无效数据（移动指针替代 erase）
        bufferIndex = start;

        if (constexpr size_t headerLen = 28; buffer.size() - bufferIndex < headerLen)
            return false;

        if (sync == 0x13) {
            // 长头只认「紧凑 IMU 记录」：固定 56 字节、无标准 OEM7 帧长/CRC，靠 week/秒范围门确认。
            if (isCompactImuRecord(buffer, bufferIndex)) {
                message.assign(buffer.begin() + static_cast<long long>(bufferIndex),
                               buffer.begin() + static_cast<long long>(bufferIndex) + 56);
                message[4] = static_cast<uint8_t>(ID_RAWIMU_COMPACT & 0xFF);
                message[5] = static_cast<uint8_t>(ID_RAWIMU_COMPACT >> 8 & 0xFF);
                bufferIndex += 56;
                return true;
            }
            bufferIndex += 3; // 非紧凑长头 → 跳过此同步头继续搜索
            continue;
        }

        // 标准短头 OEM7 帧（对齐段等）：totalLen = hLen + msgLen + 4
        const auto hlen = buffer[bufferIndex + 3];
        const auto msgLen = Math::U2(&buffer[bufferIndex + 8]);
        const auto totalLen = hlen + msgLen + 4;

        if (buffer.size() - bufferIndex < totalLen) {
            // 同步头后数据不足 → 跳过此同步头继续搜索
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

bool OEM7Reader::isCompactImuRecord(const std::vector<uint8_t> &buf, const size_t start) {
    if (start + 56 > buf.size()) return false;
    const int week = buf[start + 12] | buf[start + 13] << 8;
    double sec = 0.0;
    std::memcpy(&sec, &buf[start + 16], sizeof(double));
    return week >= 1000 && week <= 3000 && sec >= 0.0 && sec <= FULL_WEEK;
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

    currentHeader.hLen = message[3];
    currentHeader.type = Math::U2(&message[4]);
    currentHeader.length = Math::U2(&message[8]);
    currentHeader.week = Math::U2(&message[14]);
    currentHeader.ms = Math::I4(&message[16]);

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
        const auto gfrq = Math::U2(&message[off + 2]); // GLONASS FCN+8（FDMA 频率通道）
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
            case 0: // GPS
                sys = 'G';
                switch (sigType) {
                    case 0: freqIdx = 1;
                        break; // L1 C/A
                    case 9: freqIdx = 2;
                        break; // L2 P(Y)
                    default: freqIdx = -1;
                        break;
                }
                break;
            case 1: // GLONASS
                sys = 'R';
                prn = prnRaw - 37; // 37+slot → slot(1..24)
                chan = gfrq - 8; // FCN+8 → 频率通道号 k
                switch (sigType) {
                    case 0: freqIdx = 1;
                        break; // G1 C/A
                    case 1: // G2 C/A
                    case 5: freqIdx = 2;
                        break; // G2 P
                    case 6: freqIdx = 3;
                        break; // G3 (CDMA)
                    default: freqIdx = -1;
                        break;
                }
                break;
            case 3: // Galileo
                sys = 'E';
                switch (sigType) {
                    case 2: freqIdx = 1;
                        break; // E1 C
                    case 6: // E6 B
                    case 7: freqIdx = 6;
                        break; // E6 C
                    case 12: freqIdx = 5;
                        break; // E5a Q
                    case 17: freqIdx = 7;
                        break; // E5b Q
                    case 20: freqIdx = 8;
                        break; // AltBOC
                    default: freqIdx = -1;
                        break;
                }
                break;
            case 4: // BDS
                sys = 'C';
                switch (sigType) {
                    case 0: // B1I D1
                    case 4: // B1I D2
                        freqIdx = 2;
                        break; // B1
                    case 2: // B3I D1
                    case 6: // B3I D2
                        freqIdx = 6;
                        break; // B3
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
    off += 8; // tow skip
    eph.health = Math::U4(&message[off]);
    off += 4;
    off += 8; // iode skip
    eph.week = Math::U4(&message[off]);
    off += 4;
    off += 4; // zWeek skip
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
    off += 8; //NOLINT

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
        currentObs.antennaPosition = BLHtoXYZ({lat_rad, lon_rad, hgt}, Frame::WGS84);
        return true;
    }

    return false;
}

bool OEM7Reader::getNextEpoch(ObsData &obs) {
    while (true) {
        std::vector<uint8_t> message;
        if (!getNextMessage(message)) return false;
        parseImu(message); // 流式路径也累积 IMU，与 readAll 一致；避免对准 .DAT 中 GNSS/IMU 交错时静默丢 IMU 样本。非 IMU 帧返回 false 不累积。
        if (parseGNSSMessage(message)) {
            if (!currentObs.satTypeValueData.empty()) {
                obs = std::move(currentObs);
                currentObs.satTypeValueData.clear(); // 交付后清空，防止重复触发
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

    if (type == ID_RAWIMUXA || type == ID_RAWIMUSA) {
        // 标准短头 OEM7 帧带 CRC32 尾
        if (!crcExam(message)) return false;
        if (message.size() < 3 + 28) return false;
        const int off = message[3] + 16; // 6 个 IMU 计数从 hLen+16 起
        if (off + 24 > static_cast<int>(message.size())) return false;
        week = Math::U2(&message[14]);
        towSec = Math::I4(&message[16]) * 0.001; // OEM7 头的 week/ms
        zacc = Math::I4(&message[off + 0]);
        nyacc = Math::I4(&message[off + 4]);
        xacc = Math::I4(&message[off + 8]);
        zgyro = Math::I4(&message[off + 12]);
        nygyro = Math::I4(&message[off + 16]);
        xgyro = Math::I4(&message[off + 20]);
    } else if (type == ID_RAWIMU_COMPACT) {
        // 紧凑长头 56 字节记录无标准 OEM7 帧长/CRC
        if (message.size() < 56) return false;
        week = Math::U2(&message[12]);
        std::memcpy(&towSec, &message[16], sizeof(double));
        zacc = Math::I4(&message[28]);
        nyacc = Math::I4(&message[32]);
        xacc = Math::I4(&message[36]);
        zgyro = Math::I4(&message[40]);
        nygyro = Math::I4(&message[44]);
        xgyro = Math::I4(&message[48]);
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
