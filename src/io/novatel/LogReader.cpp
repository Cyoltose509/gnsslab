#include <vector>
#include <algorithm>
#include <cmath>

#include "core/TimeStruct.h"
#include "core/obs/ObsStruct.h"
#include "core/geom/CoordConvert.h"
#include "novatel/LogReader.h"
#include "core/Const.h"

#include "Log.h"
#include "MathUtils.h"
#include "StringUtils.h"

namespace NovAtel {
    constexpr std::uint32_t kPolyCrc32 = 0xEDB88320u;
    std::uint32_t crc32(const std::uint8_t *data, const std::size_t len) {
        unsigned int calc = 0;
        for (std::size_t i = 0; i < len; ++i) {
            calc ^= data[i];
            for (int j = 0; j < 8; ++j) {
                if (calc & 1) calc = calc >> 1 ^ kPolyCrc32;
                else calc >>= 1;
            }
        }
        return calc;
    }

    void LogReader::close() {
        mEphSnap.reset();
        mPending.reset();
        mEpochWeek = -1;
        mEpochMs = 0;
        mSeenFullRange = false;
        mSeenCmpRange = false;
        mPrevLock.clear();
        mCurLock.clear();
        mImuLog.clear();
        mImuType = -1;
    }

    void LogReader::noteRangeFormat(const bool full) {
        if (full) {
            if (mSeenFullRange.exchange(true)) return;
            LOG_INFO << "observation format: RANGE (unquantized)"
                    << (mSeenCmpRange ? "; RANGE wins when RANGECMP is also present" : "");
            return;
        }
        if (mSeenCmpRange.exchange(true)) return;
        LOG_INFO << "observation format: RANGECMP (pseudorange 1/128 m, phase 1/256 cycle)";
    }

    bool LogReader::crcExam(const std::vector<std::uint8_t> &message) {
        if (message.size() < 4) return false;
        const auto msg_len = message.size() - 4;
        return crc32(message.data(), msg_len) == Math::U4(message.data() + msg_len);
    }

    bool LogReader::parseGNSSMessage(const std::vector<std::uint8_t> &message) {
        if (!crcExam(message)) return false;

        const bool longHeader = message[3] == 28;
        currentHeader.hLen = longHeader ? 28 : 12;
        currentHeader.type = Math::U2(&message[4]);
        currentHeader.length = longHeader ? static_cast<int>(Math::U2(&message[8])) : message[3];
        currentHeader.week = static_cast<int>(longHeader ? Math::U2(&message[14]) : Math::U2(&message[6]));
        currentHeader.ms = longHeader ? Math::I4(&message[16]) : Math::I4(&message[8]);

        switch (currentHeader.type) {
            case LogID::RANGE:
                noteRangeFormat(true);
                return parseRange(message);
            case LogID::RANGECMP:
                noteRangeFormat(false);
                return parseRangeCmp(message);
            case LogID::GPSEPHEM:
                return parseGpsEphem(message);
            case LogID::BDSEPHEMERIS:
                return parseBdsEphem(message);
            case LogID::GLOEPHEM:
                return parseGloEphem(message);
            case LogID::GALEPHEM:
                return parseGalEph(message);
            case LogID::GALINAVEPHEM:
                return parseGalINavBinary(message);
            case LogID::GALFNAVEPHEM:
                return parseGalFNavBinary(message);
            case LogID::BESTPOS:
                return parseBestPos(message);
            default:
                break;
        }
        return false;
    }

    namespace {
        struct SignalRef {
            SatSystem sys = SatSystem::Unknown;
            int freqIdx = -1;
            int chan = 0;
            int prn = 0;
            double freqHz = 0.0;
        };

        bool decodeSignal(const std::uint32_t status, const int prnRaw, const int gfrq, SignalRef &out) {
            const int sysInt = static_cast<int>(status >> 16) & 7;
            const int sigType = static_cast<int>(status >> 21) & 0x1F;
            out = SignalRef();
            out.prn = prnRaw;
            switch (sysInt) {
                case 0:
                    out.sys = SatSystem::G;
                    switch (sigType) {
                        case 0: out.freqIdx = 1;
                            break;
                        case 9: out.freqIdx = 2;
                            break;
                        default: return false;
                    }
                    break;
                case 1:
                    out.sys = SatSystem::R;
                    out.prn = prnRaw - 37;
                    out.chan = gfrq - 7;
                    switch (sigType) {
                        case 0: out.freqIdx = 1;
                            break;
                        case 1:
                        case 5: out.freqIdx = 2;
                            break;
                        case 6: out.freqIdx = 3;
                            break;
                        default: return false;
                    }
                    break;
                case 3:
                    out.sys = SatSystem::E;
                    switch (sigType) {
                        case 2: out.freqIdx = 1;
                            break;
                        case 6:
                        case 7: out.freqIdx = 6;
                            break;
                        case 12: out.freqIdx = 5;
                            break;
                        case 17: out.freqIdx = 7;
                            break;
                        case 20: out.freqIdx = 8;
                            break;
                        default: return false;
                    }
                    break;
                case 4:
                    out.sys = SatSystem::C;
                    // NovAtel OEM7 RANGE/RANGECMP 的 BeiDou 信号类型表（docs.novatel.com RANGE.htm）：
                    // 0/4 = B1I 的 D1/D2 电文版（D2 仅 GEO），1/5 = B2I 的 D1/D2，11 = B2b，
                    // 三者同为 1207.14；2/6 = B3I 的 D1/D2(1268.52)；9 = B2a(1176.45)。
                    // 不收 sigType 7（B1C）：RINEX 相位偏移 0.25 周，emitObs 无相移机制，
                    // 收进来会给相位注入常偏破坏整周固定；B1C 也超出 IF 组合的频率表用途。
                    switch (sigType) {
                        case 0: // B1I (D1)
                        case 4: out.freqIdx = 2; // B1I (D2，GEO)
                            break;
                        case 1: // B2I (D1) / B2b
                        case 5: // B2I (D2，GEO)
                        case 11: out.freqIdx = 7; // 1207.14
                            break;
                        case 9: out.freqIdx = 5; // B2a
                            break;
                        case 2: // B3I (D1)
                        case 6: out.freqIdx = 6; // B3I (D2)
                            break;
                        default: return false;
                    }
                    break;
                default:
                    return false;
            }
            out.freqHz = out.sys == SatSystem::R
                             ? getGlonassFreq(static_cast<char>('0' + out.freqIdx), out.chan)
                             : getFreq(out.sys, out.freqIdx);
            return true;
        }

        std::uint64_t extractBits(const std::uint8_t *rec, const int startBit, const int len) {
            const int nBytes = (startBit % 8 + len + 7) / 8;
            std::uint64_t v = 0;
            for (int i = 0; i < nBytes; ++i) v |= static_cast<std::uint64_t>(rec[startBit / 8 + i]) << (8 * i);
            v >>= startBit % 8;
            return v & (std::uint64_t{1} << len) - 1;
        }

        double unwrapCompressedAdr(const double adr, const double psr, const double wavelength) {
            if (!(wavelength > 0.0)) return adr;
            constexpr double maxValue = 8388608.0;
            double rolls = (psr / wavelength + adr) / maxValue;
            rolls = rolls >= 0.0 ? std::floor(rolls + 0.5) : std::ceil(rolls - 0.5);
            return adr - maxValue * rolls;
        }

        constexpr int kGpsToGalWeek = static_cast<int>((GAL_EPOCH_MJD - GPS_EPOCH_MJD) / 7);


        GalileoEphem makeGalEph(const GalEphFields &g, const int gpsWeek) {
            GalileoEphem eph;
            eph.prn = g.prn & 0xFF;
            eph.toe = g.toe;
            eph.rootA = g.rootA;
            eph.dn = g.deltaN;
            eph.m0 = g.m0;
            eph.e = g.ecc;
            eph.omega = g.omega;
            eph.cuc = g.cuc;
            eph.cus = g.cus;
            eph.crc = g.crc;
            eph.crs = g.crs;
            eph.cic = g.cic;
            eph.cis = g.cis;
            eph.i0 = g.i0;
            eph.idot = g.idot;
            eph.omega0 = g.omega0;
            eph.omegaDot = g.omegaDot;
            eph.a0 = g.af0;
            eph.a1 = g.af1;
            eph.a2 = g.af2;
            eph.toc = static_cast<double>(g.toc);
            eph.ura = static_cast<double>(g.sisa);
            eph.health = g.inav
                             ? static_cast<unsigned int>(g.svhE5b) << 1 | static_cast<unsigned int>(g.dvsE5b)
                             : static_cast<unsigned int>(g.svhE5a) << 1 | static_cast<unsigned int>(g.dvsE5a);
            eph.tgd = g.inav ? g.bgdE5bE1 : g.bgdE5aE1;
            // 报文里给的是 GPS 周（与 header 同周），GalileoEphem 按 GST 纪元展开，折回 GST 周
            eph.week = gpsWeek - kGpsToGalWeek;
            return eph;
        }
    }

    void LogReader::beginEpoch() {
        // RANGE 与 RANGECMP 是同一批观测的两种编码，时间戳相同就必须并进一个历元，
        // 否则下游会收到两个同时间的历元。
        if (currentHeader.week == mEpochWeek && currentHeader.ms == mEpochMs) return;

        if (!currentObs.satTypeValueData.empty()) {
            ObsData done;
            done.antennaPosition = currentObs.antennaPosition;
            done.antType = currentObs.antType;
            done.station = currentObs.station;
            done.epoch = currentObs.epoch;
            done.weekSecond = currentObs.weekSecond;
            done.satTypeValueData.swap(currentObs.satTypeValueData);
            done.satTypeLLI.swap(currentObs.satTypeLLI);
            mPending = std::move(done);
        }

        mEpochWeek = currentHeader.week;
        mEpochMs = currentHeader.ms;
        mPrevLock.swap(mCurLock);
        mCurLock.clear();
        const WeekSecond ws(currentHeader.week, currentHeader.ms * 0.001, TimeSystem::GPS);
        currentObs.epoch = ws.toCommonTime();
        currentObs.weekSecond = ws;
    }

    void LogReader::emitObs(const int prnRaw, const int gfrq, const double psr, const double adr,
                            const double doppler, const double cn0, const double lockTime,
                            const std::uint32_t status, const bool fromFull) {
        SignalRef sig;
        if (!decodeSignal(status, prnRaw, gfrq, sig)) return;

        const auto sys = sig.sys;
        const std::string s_f = std::to_string(sig.freqIdx);
        const std::string ctype = "C" + s_f;
        if (std::vector<std::string> &vec = availableTypes[sys]; std::find(vec.begin(), vec.end(), ctype) == vec.end())
            vec.push_back(ctype);

        const bool codeLocked = static_cast<int>(status >> 12) & 1;
        const bool phaseLocked = static_cast<int>(status >> 10) & 1;
        const bool parityKnown = static_cast<int>(status >> 11) & 1;

        const SatID sat(sys, sig.prn, sig.chan);
        auto &values = currentObs.satTypeValueData[sat];
        const auto put = [&](const std::string &type, const double v) {
            if (fromFull || values.find(type) == values.end()) values[type] = v;
        };
        if (codeLocked) put(ctype, psr);
        const double lam = sig.freqHz > 0.0 ? C_MPS / sig.freqHz : 0.0;
        put("L" + s_f, phaseLocked && lam > 0.0 ? -adr * lam : 0.0);
        put("D" + s_f, doppler);
        put("S" + s_f, cn0);

        if (const int fi = sig.freqIdx; fi >= 0 && fi < 9) {
            bool regress = false;
            if (const auto it = mPrevLock.find(sat); it != mPrevLock.end() && it->second[fi] >= 0.0f)
                regress = lockTime + 0.05 < it->second[fi];

            const auto cur = mCurLock.find(sat);
            if (cur == mCurLock.end()) mCurLock[sat].fill(-1.0f);
            float &lock = mCurLock[sat][fi];
            lock = lock < 0.0f ? static_cast<float>(lockTime)
                               : std::min(lock, static_cast<float>(lockTime));

            if (phaseLocked) {
                int lli = 0;
                if (regress) lli |= 1;
                if (!parityKnown) lli |= 2;
                if (lli != 0) currentObs.satTypeLLI[sat]["L" + s_f] = lli;
            }
        }
    }

    bool LogReader::parseRange(const std::vector<std::uint8_t> &message) {
        beginEpoch();
        int off = currentHeader.hLen;
        if (static_cast<std::size_t>(off) + 4 > message.size()) return false;
        const std::uint32_t numObs = Math::U4(&message[off]);
        off += 4;

        for (std::uint32_t i = 0; i < numObs; ++i) {
            if (off + 44 > static_cast<int>(message.size())) break;
            emitObs(Math::U2(&message[off + 0]), Math::U2(&message[off + 2]),
                    Math::R8(&message[off + 4]), Math::R8(&message[off + 16]),
                    Math::R4(&message[off + 28]), Math::R4(&message[off + 32]),
                    Math::R4(&message[off + 36]), Math::U4(&message[off + 40]), true);
            off += 44;
        }
        return !currentObs.satTypeValueData.empty();
    }

    bool LogReader::parseRangeCmp(const std::vector<std::uint8_t> &message) {
        beginEpoch();
        int off = currentHeader.hLen;
        if (static_cast<std::size_t>(off) + 4 > message.size()) return false;
        const std::uint32_t numObs = Math::U4(&message[off]);
        off += 4;

        for (std::uint32_t i = 0; i < numObs; ++i) {
            if (off + 24 > static_cast<int>(message.size())) break;
            const std::uint8_t *rec = &message[off];
            const auto status = static_cast<std::uint32_t>(extractBits(rec, 0, 32));
            const auto dopplerRaw = static_cast<std::int32_t>(extractBits(rec, 32, 28) << 4) >> 4; // 28 位补符号
            const auto psr = static_cast<double>(extractBits(rec, 60, 36)) / 128.0;
            const auto adrRaw = static_cast<double>(static_cast<std::int32_t>(extractBits(rec, 96, 32))) / 256.0;
            const auto prnRaw = static_cast<int>(extractBits(rec, 136, 8));
            const auto cn0 = 20.0 + static_cast<double>(extractBits(rec, 165, 5));
            const auto gfrq = static_cast<int>(extractBits(rec, 170, 6));
            const auto lockTime = static_cast<double>(extractBits(rec, 144, 21)) / 32.0;

            double adr = adrRaw;
            if (SignalRef sig; decodeSignal(status, prnRaw, gfrq, sig))
                adr = unwrapCompressedAdr(adrRaw, psr, sig.freqHz > 0.0 ? C_MPS / sig.freqHz : 0.0);

            emitObs(prnRaw, gfrq, psr, adr, static_cast<double>(dopplerRaw) / 256.0, cn0, lockTime, status, false);
            off += 24;
        }
        return !currentObs.satTypeValueData.empty();
    }

    bool LogReader::parseGpsEphem(const std::vector<std::uint8_t> &message) {
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
        eph.rootA = std::sqrt(A);
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
        off += 8;//NOLINT

        storeEph(&EphemerisTable::gps, eph.prn, std::move(eph));
        return true;
    }

    bool LogReader::parseBdsEphem(const std::vector<std::uint8_t> &message) {
        BDSEphem eph;
        int off = currentHeader.hLen;
        eph.prn = static_cast<int>(Math::U4(&message[off])) & 0xFF;
        off += 4;
        eph.week = static_cast<int>(Math::U4(&message[off]));
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

    bool LogReader::parseGloEphem(const std::vector<std::uint8_t> &message) {
        if (message.size() < static_cast<std::size_t>(currentHeader.hLen) + 144) return false;

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
        eph.tauN = Math::R8(&message[off]);
        off += 8;
        eph.deltaTauN = Math::R8(&message[off]);
        off += 8;
        eph.gamma = Math::R8(&message[off]);
        off += 8;
        off += 4; // Tk（信息帧起始时刻，相对 GLONASS 日，未用）
        off += 8; // P / Ft（技术参数、用户距离，未用）NOLINT

        eph.prn = rawPrn - 37; // 37+slot → slot(1..24)
        eph.freqNum = frq;

        const WeekSecond wsGps(week, tow);
        CommonTime toeGps = wsGps.toCommonTime();
        toeGps.convertSystem(TimeSystem::UTC);
        eph.toeCommon = toeGps;
        eph.week = week;
        eph.toe = tow;

        storeEph(&EphemerisTable::glo, eph.prn, std::move(eph));
        return true;
    }

    bool LogReader::parseGalEph(const std::vector<std::uint8_t> &message) {
        const int h = currentHeader.hLen;
        if (message.size() < static_cast<std::size_t>(h) + 220) return false;
        const std::uint8_t *p = &message[h];

        GalEphFields g;
        g.prn = static_cast<int>(Math::U4(&p[0])) & 0xFF;
        g.svhE1b = p[12];
        g.svhE5a = p[13];
        g.svhE5b = p[14] & 3;
        g.dvsE1b = p[15];
        g.dvsE5a = p[16];
        g.dvsE5b = p[17] & 1;
        g.sisa = p[18];
        g.iodNav = static_cast<int>(Math::U4(&p[20]));
        g.toe = static_cast<int>(Math::U4(&p[24]));
        g.rootA = Math::R8(&p[28]);
        g.deltaN = Math::R8(&p[36]);
        g.m0 = Math::R8(&p[44]);
        g.ecc = Math::R8(&p[52]);
        g.omega = Math::R8(&p[60]);
        g.cuc = Math::R8(&p[68]);
        g.cus = Math::R8(&p[76]);
        g.crc = Math::R8(&p[84]);
        g.crs = Math::R8(&p[92]);
        g.cic = Math::R8(&p[100]);
        g.cis = Math::R8(&p[108]);
        g.i0 = Math::R8(&p[116]);
        g.idot = Math::R8(&p[124]);
        g.omega0 = Math::R8(&p[132]);
        g.omegaDot = Math::R8(&p[140]);

        g.inav = (Math::U4(&p[8]) & 1u) != 0u; // 收得到 I/NAV 就用 I/NAV，否则 F/NAV
        const int clk = g.inav ? 176 : 148;
        g.toc = static_cast<int>(Math::U4(&p[clk]));
        g.af0 = Math::R8(&p[clk + 4]);
        g.af1 = Math::R8(&p[clk + 12]);
        g.af2 = Math::R8(&p[clk + 20]);
        g.bgdE5aE1 = Math::R8(&p[204]);
        g.bgdE5bE1 = Math::R8(&p[212]);

        GalileoEphem eph = makeGalEph(g, currentHeader.week);
        storeEph(&EphemerisTable::gal, eph.prn, std::move(eph));
        return true;
    }

    void assembleGalBody(const GalEphFields &g, std::vector<std::uint8_t> &body) {
        body.assign(220, 0);
        Math::putU4(body, 0, static_cast<std::uint32_t>(g.prn));
        Math::putU4(body, 4, g.inav ? 0u : 1u); // rcv F/NAV
        Math::putU4(body, 8, g.inav ? 1u : 0u); // rcv I/NAV（parseGalEph 据此选钟差来源）
        body[12] = static_cast<std::uint8_t>(g.svhE1b);
        body[13] = static_cast<std::uint8_t>(g.svhE5a);
        body[14] = static_cast<std::uint8_t>(g.svhE5b);
        body[15] = static_cast<std::uint8_t>(g.dvsE1b);
        body[16] = static_cast<std::uint8_t>(g.dvsE5a);
        body[17] = static_cast<std::uint8_t>(g.dvsE5b);
        body[18] = static_cast<std::uint8_t>(g.sisa);
        body[19] = 0;
        Math::putU4(body, 20, static_cast<std::uint32_t>(g.iodNav));
        Math::putU4(body, 24, static_cast<std::uint32_t>(g.toe));
        const double orb[15] = {
            g.rootA, g.deltaN, g.m0, g.ecc, g.omega,
            g.cuc, g.cus, g.crc, g.crs, g.cic, g.cis,
            g.i0, g.idot, g.omega0, g.omegaDot
        };
        for (int k = 0; k < 15; ++k) Math::putR8(body, 28 + 8 * k, orb[k]);
        // F/NAV 钟差槽（148..175）与 I/NAV 钟差槽（176..203）只填其中一个，另一个留 0。
        Math::putU4(body, 148, static_cast<std::uint32_t>(g.inav ? 0 : g.toc));
        Math::putR8(body, 152, g.inav ? 0.0 : g.af0);
        Math::putR8(body, 160, g.inav ? 0.0 : g.af1);
        Math::putR8(body, 168, g.inav ? 0.0 : g.af2);
        Math::putU4(body, 176, static_cast<std::uint32_t>(g.inav ? g.toc : 0));
        Math::putR8(body, 180, g.inav ? g.af0 : 0.0);
        Math::putR8(body, 188, g.inav ? g.af1 : 0.0);
        Math::putR8(body, 196, g.inav ? g.af2 : 0.0);
        Math::putR8(body, 204, g.bgdE5aE1);
        Math::putR8(body, 212, g.bgdE5bE1);
    }

    bool LogReader::parseGalINavBinary(const std::vector<std::uint8_t> &message) {
        const int h = currentHeader.hLen;
        if (message.size() < static_cast<std::size_t>(h) + 184) return false;
        const std::uint8_t *b = &message[h];

        GalEphFields g;
        g.prn = static_cast<int>(Math::U4(b)); // SatId
        g.svhE5b = b[4];
        g.dvsE5b = b[5];
        g.svhE1b = b[8];
        g.dvsE1b = b[9];
        g.iodNav = static_cast<int>(Math::U2(b + 12));
        g.sisa = b[14];
        g.toe = static_cast<int>(Math::U4(b + 16)); // T0e
        g.toc = static_cast<int>(Math::U4(b + 20)); // T0c
        g.m0 = Math::R8(b + 24);
        g.deltaN = Math::R8(b + 32);
        g.ecc = Math::R8(b + 40);
        g.rootA = Math::R8(b + 48);
        g.i0 = Math::R8(b + 56);
        g.idot = Math::R8(b + 64);
        g.omega0 = Math::R8(b + 72);
        g.omega = Math::R8(b + 80);
        g.omegaDot = Math::R8(b + 88);
        g.cuc = Math::R8(b + 96);
        g.cus = Math::R8(b + 104);
        g.crc = Math::R8(b + 112);
        g.crs = Math::R8(b + 120);
        g.cic = Math::R8(b + 128);
        g.cis = Math::R8(b + 136);
        g.af0 = Math::R8(b + 144);
        g.af1 = Math::R8(b + 152);
        g.af2 = Math::R8(b + 160);
        g.bgdE5aE1 = Math::R8(b + 168);
        g.bgdE5bE1 = Math::R8(b + 176);
        g.inav = true;

        GalileoEphem eph = makeGalEph(g, currentHeader.week);
        storeEph(&EphemerisTable::gal, eph.prn, std::move(eph));
        return true;
    }

    bool LogReader::parseGalFNavBinary(const std::vector<std::uint8_t> &message) {
        const int h = currentHeader.hLen;
        if (message.size() < static_cast<std::size_t>(h) + 172) return false;
        const std::uint8_t *b = &message[h];

        GalEphFields g;
        g.prn = static_cast<int>(Math::U4(b)); // SatId
        g.svhE5a = b[4];
        g.dvsE5a = b[5];
        g.iodNav = static_cast<int>(Math::U2(b + 8));
        g.sisa = b[10];
        g.toe = static_cast<int>(Math::U4(b + 12)); // T0e
        g.toc = static_cast<int>(Math::U4(b + 16)); // T0c
        g.m0 = Math::R8(b + 20);
        g.deltaN = Math::R8(b + 28);
        g.ecc = Math::R8(b + 36);
        g.rootA = Math::R8(b + 44);
        g.i0 = Math::R8(b + 52);
        g.idot = Math::R8(b + 60);
        g.omega0 = Math::R8(b + 68);
        g.omega = Math::R8(b + 76);
        g.omegaDot = Math::R8(b + 84);
        g.cuc = Math::R8(b + 92);
        g.cus = Math::R8(b + 100);
        g.crc = Math::R8(b + 108);
        g.crs = Math::R8(b + 116);
        g.cic = Math::R8(b + 124);
        g.cis = Math::R8(b + 132);
        g.af0 = Math::R8(b + 140);
        g.af1 = Math::R8(b + 148);
        g.af2 = Math::R8(b + 156);
        g.bgdE5aE1 = Math::R8(b + 164);
        g.bgdE5bE1 = 0.0;
        g.inav = false;

        GalileoEphem eph = makeGalEph(g, currentHeader.week);
        storeEph(&EphemerisTable::gal, eph.prn, std::move(eph));
        return true;
    }


    bool LogReader::parseBestPos(const std::vector<std::uint8_t> &message) {
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
            currentObs.antennaPosition = Coord::BLHToXYZ({lat_rad, lon_rad, hgt}, Frame::WGS84);
            return true;
        }

        return false;
    }

    bool LogReader::getNextEpoch(ObsData &obs, const bool block) {
        while (true) {
            if (mPending) {
                obs = std::move(*mPending);
                mPending.reset();
                return true;
            }
            std::vector<std::uint8_t> message;
            if (!getNextMessage(message, block)) {
                if (currentObs.satTypeValueData.empty()) return false;
                obs = std::move(currentObs);
                currentObs.satTypeValueData.clear();
                currentObs.satTypeLLI.clear();
                return true;
            }
            parseImu(message);
            parseGNSSMessage(message);
        }
    }

    bool LogReader::parseImu(const std::vector<std::uint8_t> &message) {
        if (message.size() < 6) return false;
        const int type = Math::U2(&message[4]);
        if (type != LogID::RAWIMU && type != LogID::RAWIMUS &&
            type != LogID::RAWIMUX && type != LogID::RAWIMUSX) return false;
        if (!crcExam(message) || message.size() < 12) return false;

        const int hLen = message[3] == 28 ? 28 : 12;
        const std::size_t off = static_cast<std::size_t>(hLen) + 16;
        if (message.size() < off + 24) return false;

        int week = 0;
        if (type == LogID::RAWIMUX || type == LogID::RAWIMUSX) {
            week = Math::U2(&message[hLen + 2]);
            if (mImuType < 0) {
                if (const int t = message[hLen + 1]; t > 0) mImuType = t;
            }
        } else {
            week = static_cast<int>(Math::U4(&message[hLen + 0]));
        }

        double towSec = 0.0;
        std::memcpy(&towSec, &message[hLen + 4], sizeof(double));

        ImuSample s;
        s.gnssWeek = week;
        s.tow = week * FULL_WEEK + towSec;
        s.ax = Math::I4(&message[off + 8]);
        s.ay = -Math::I4(&message[off + 4]);
        s.az = Math::I4(&message[off + 0]);
        s.gx = Math::I4(&message[off + 20]);
        s.gy = -Math::I4(&message[off + 16]);
        s.gz = Math::I4(&message[off + 12]);
        mImuLog.push_back(s);
        return true;
    }
}
