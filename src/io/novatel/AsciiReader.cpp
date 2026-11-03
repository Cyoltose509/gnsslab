#include "AsciiReader.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>

#include "StringUtils.h"
#include "MathUtils.h"

namespace NovAtel {
    namespace {
        constexpr int kShortHeader = 12;

        double field(const std::vector<std::string> &f, const std::size_t i) {
            return i < f.size() ? Str::toDouble(f[i]) : 0.0;
        }

        std::uint32_t intField(const std::vector<std::string> &f, const std::size_t i) {
            return i < f.size() ? static_cast<std::uint32_t>(Str::toInt(f[i])) : 0u;
        }

        std::int32_t signedField(const std::vector<std::string> &f, const std::size_t i) {
            return i < f.size() ? Str::toInt(f[i]) : 0;
        }

        std::uint32_t flagField(const std::vector<std::string> &f, const std::size_t i) {
            if (i >= f.size()) return 0u;
            return f[i] == "TRUE" || f[i] == "true" || f[i] == "1" ? 1u : 0u;
        }

        std::uint32_t hexField(const std::vector<std::string> &f, const std::size_t i) {
            if (i >= f.size()) return 0u;
            return static_cast<std::uint32_t>(std::strtoul(f[i].c_str(), nullptr, 16));
        }

        int hexDigit(const char c) {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        }

        bool hexToBytes(const std::string &s, std::uint8_t *out, const std::size_t n) {
            if (s.size() != n * 2) return false;
            for (std::size_t i = 0; i < n; ++i) {
                const int hi = hexDigit(s[2 * i]);
                const int lo = hexDigit(s[2 * i + 1]);
                if (hi < 0 || lo < 0) return false;
                out[i] = static_cast<std::uint8_t>(hi << 4 | lo);
            }
            return true;
        }

        bool parseHead(const std::string &head, std::string &name, int &week, int &ms) {
            const std::vector<std::string> f = Str::split(head, ',', false);
            if (f.empty() || f[0].empty()) return false;
            name = Str::strip(f[0].substr(1)); // 去掉前导 '#'/'%'
            if (f.size() >= 7) {
                week = Str::toInt(f[5]);
                ms = static_cast<int>(std::llround(Str::toDouble(f[6]) * 1000.0));
            } else if (f.size() >= 3) {
                week = Str::toInt(f[1]);
                ms = static_cast<int>(std::llround(Str::toDouble(f[2]) * 1000.0));
            } else {
                return false;
            }
            return !name.empty();
        }

        std::uint32_t solStatus(const std::vector<std::string> &f, const std::size_t i) {
            return i < f.size() && f[i] == "SOL_COMPUTED" ? 0u : 1u;
        }

        std::vector<std::uint8_t> sealMessage(const std::uint16_t id, const int week, const int ms,
                                              const std::vector<std::uint8_t> &body) {
            std::vector<std::uint8_t> msg(static_cast<std::size_t>(kShortHeader) + body.size() + 4, 0);
            msg[0] = 0xAA;
            msg[1] = 0x44;
            msg[2] = 0x12;
            msg[3] = kShortHeader;
            Math::putU2(msg, 4, id);
            Math::putU2(msg, 6, static_cast<std::uint32_t>(week));
            Math::putI4(msg, 8, ms);
            if (!body.empty()) std::memcpy(msg.data() + kShortHeader, body.data(), body.size());
            const auto bodyEnd = static_cast<std::size_t>(kShortHeader) + body.size();
            Math::putU4(msg, bodyEnd, crc32(msg.data(), bodyEnd));
            return msg;
        }

        void buildRangeBody(const std::vector<std::string> &f, std::vector<std::uint8_t> &body) {
            std::size_t n = f.empty() ? 0 : static_cast<std::size_t>(Str::toInt(f[0]));
            n = std::min(n, (f.size() - 1) / 10);
            body.assign(4 + 44 * n, 0);
            Math::putU4(body, 0, static_cast<std::uint32_t>(n));
            for (std::size_t i = 0; i < n; ++i) {
                const std::size_t o = 4 + 44 * i;
                const std::size_t b = 1 + 10 * i;
                Math::putU2(body, o + 0, intField(f, b + 0));
                Math::putU2(body, o + 2, intField(f, b + 1));
                Math::putR8(body, o + 4, field(f, b + 2));
                Math::putR4(body, o + 12, field(f, b + 3));
                Math::putR8(body, o + 16, field(f, b + 4));
                Math::putR4(body, o + 24, field(f, b + 5));
                Math::putR4(body, o + 28, field(f, b + 6));
                Math::putR4(body, o + 32, field(f, b + 7));
                Math::putR4(body, o + 36, field(f, b + 8));
                Math::putU4(body, o + 40, hexField(f, b + 9));
            }
        }

        bool buildRangeCmpBody(const std::vector<std::string> &f, std::vector<std::uint8_t> &body) {
            const std::size_t n = f.empty() ? 0 : static_cast<std::size_t>(Str::toInt(f[0]));
            if (n == 0 || f.size() < n + 1) return false;
            body.assign(4 + 24 * n, 0);
            Math::putU4(body, 0, static_cast<std::uint32_t>(n));
            for (std::size_t i = 0; i < n; ++i)
                if (!hexToBytes(f[1 + i], &body[4 + 24 * i], 24)) return false;
            return true;
        }

        void buildGpsEphemBody(const std::vector<std::string> &f, std::vector<std::uint8_t> &body) {
            body.assign(228, 0);
            Math::putU4(body, 0, intField(f, 0));
            Math::putR8(body, 4, field(f, 1)); // tow
            Math::putU4(body, 12, intField(f, 2)); // health
            Math::putU4(body, 16, intField(f, 3)); // IODE1
            Math::putU4(body, 20, intField(f, 4)); // IODE2
            Math::putU4(body, 24, intField(f, 5)); // week
            Math::putU4(body, 28, intField(f, 6)); // z week
            Math::putR8(body, 32, field(f, 7)); // toe
            for (int k = 0; k < 15; ++k) Math::putR8(body, 40 + 8 * k, field(f, 8 + k)); // A … ώ
            Math::putU4(body, 160, intField(f, 23)); // iodc
            Math::putR8(body, 164, field(f, 24)); // toc
            Math::putR8(body, 172, field(f, 25)); // tgd
            Math::putR8(body, 180, field(f, 26)); // af0
            Math::putR8(body, 188, field(f, 27)); // af1
            Math::putR8(body, 196, field(f, 28)); // af2
            Math::putU4(body, 204, flagField(f, 29)); // AS
            Math::putR8(body, 208, field(f, 30)); // N
            Math::putR8(body, 216, field(f, 31)); // URA
        }

        void buildBdsEphemBody(const std::vector<std::string> &f, std::vector<std::uint8_t> &body) {
            body.assign(196, 0);
            Math::putU4(body, 0, intField(f, 0)); // PRN
            Math::putU4(body, 4, intField(f, 1)); // week
            Math::putR8(body, 8, field(f, 2)); // URA
            Math::putU4(body, 16, intField(f, 3)); // health
            Math::putR8(body, 20, field(f, 4)); // TGD1
            Math::putR8(body, 28, field(f, 5)); // TGD2
            Math::putU4(body, 36, intField(f, 6)); // AODC
            Math::putU4(body, 40, intField(f, 7)); // toc
            Math::putR8(body, 44, field(f, 8)); // a0
            Math::putR8(body, 52, field(f, 9)); // a1
            Math::putR8(body, 60, field(f, 10)); // a2
            Math::putU4(body, 68, intField(f, 11)); // AODE
            Math::putU4(body, 72, intField(f, 12)); // toe
            for (int k = 0; k < 10; ++k) Math::putR8(body, 76 + 8 * k, field(f, 13 + k)); // 根号A … cuc
            Math::putR8(body, 156, field(f, 23)); // cus
            Math::putR8(body, 164, field(f, 24)); // crc
            Math::putR8(body, 172, field(f, 25)); // crs
            Math::putR8(body, 180, field(f, 26)); // cic
            Math::putR8(body, 188, field(f, 27)); // cis
        }

        void buildGalEphemBody(const std::vector<std::string> &f, std::vector<std::uint8_t> &body) {
            GalEphFields g;
            g.prn = static_cast<int>(intField(f, 0));
            g.inav = flagField(f, 2) != 0; // rcv I/NAV
            g.svhE1b = static_cast<int>(intField(f, 3));
            g.svhE5a = static_cast<int>(intField(f, 4));
            g.svhE5b = static_cast<int>(intField(f, 5));
            g.dvsE1b = static_cast<int>(intField(f, 6));
            g.dvsE5a = static_cast<int>(intField(f, 7));
            g.dvsE5b = static_cast<int>(intField(f, 8));
            g.sisa = static_cast<int>(intField(f, 9));
            g.iodNav = static_cast<int>(intField(f, 11));
            g.toe = static_cast<int>(intField(f, 12));
            g.rootA = field(f, 13);
            g.deltaN = field(f, 14);
            g.m0 = field(f, 15);
            g.ecc = field(f, 16);
            g.omega = field(f, 17);
            g.cuc = field(f, 18);
            g.cus = field(f, 19);
            g.crc = field(f, 20);
            g.crs = field(f, 21);
            g.cic = field(f, 22);
            g.cis = field(f, 23);
            g.i0 = field(f, 24);
            g.idot = field(f, 25);
            g.omega0 = field(f, 26);
            g.omegaDot = field(f, 27);
            const int clk = g.inav ? 32 : 28; // f28..31 是 F/NAV 钟差，f32..35 是 I/NAV
            g.toc = static_cast<int>(intField(f, clk));
            g.af0 = field(f, clk + 1);
            g.af1 = field(f, clk + 2);
            g.af2 = field(f, clk + 3);
            g.bgdE5aE1 = field(f, 36);
            g.bgdE5bE1 = field(f, 37);
            assembleGalBody(g, body);
        }

        void buildGalInavEphemBody(const std::vector<std::string> &f, std::vector<std::uint8_t> &body) {
            GalEphFields g;
            g.prn = static_cast<int>(intField(f, 0));
            g.svhE5b = static_cast<int>(intField(f, 1));
            g.dvsE5b = static_cast<int>(intField(f, 2));
            g.svhE1b = static_cast<int>(intField(f, 5));
            g.dvsE1b = static_cast<int>(intField(f, 6));
            g.iodNav = static_cast<int>(intField(f, 9));
            g.sisa = static_cast<int>(intField(f, 10));
            g.toe = static_cast<int>(intField(f, 12)); // T0e
            g.toc = static_cast<int>(intField(f, 13)); // T0c
            g.m0 = field(f, 14);
            g.deltaN = field(f, 15);
            g.ecc = field(f, 16);
            g.rootA = field(f, 17);
            g.i0 = field(f, 18);
            g.idot = field(f, 19);
            g.omega0 = field(f, 20);
            g.omega = field(f, 21);
            g.omegaDot = field(f, 22);
            g.cuc = field(f, 23);
            g.cus = field(f, 24);
            g.crc = field(f, 25);
            g.crs = field(f, 26);
            g.cic = field(f, 27);
            g.cis = field(f, 28);
            g.af0 = field(f, 29);
            g.af1 = field(f, 30);
            g.af2 = field(f, 31);
            g.bgdE5aE1 = field(f, 32);
            g.bgdE5bE1 = field(f, 33);
            g.inav = true;
            assembleGalBody(g, body);
        }

        void buildGalFnavEphemBody(const std::vector<std::string> &f, std::vector<std::uint8_t> &body) {
            GalEphFields g;
            g.prn = static_cast<int>(intField(f, 0));
            g.svhE5a = static_cast<int>(intField(f, 1));
            g.dvsE5a = static_cast<int>(intField(f, 2));
            g.iodNav = static_cast<int>(intField(f, 5));
            g.sisa = static_cast<int>(intField(f, 6));
            g.toe = static_cast<int>(intField(f, 9));
            g.toc = static_cast<int>(intField(f, 9));
            g.m0 = field(f, 10);
            g.deltaN = field(f, 11);
            g.ecc = field(f, 12);
            g.rootA = field(f, 13);
            g.i0 = field(f, 14);
            g.idot = field(f, 15);
            g.omega0 = field(f, 16);
            g.omega = field(f, 17);
            g.omegaDot = field(f, 18);
            g.cuc = field(f, 19);
            g.cus = field(f, 20);
            g.crc = field(f, 21);
            g.crs = field(f, 22);
            g.cic = field(f, 23);
            g.cis = field(f, 24);
            g.af0 = field(f, 25);
            g.af1 = field(f, 26);
            g.af2 = field(f, 27);
            g.bgdE5aE1 = field(f, 28);
            g.bgdE5bE1 = 0.0; // F/NAV 只播 BGD E5a-E1
            g.inav = false;
            assembleGalBody(g, body);
        }

        void buildGloEphemBody(const std::vector<std::string> &f, std::vector<std::uint8_t> &body) {
            body.assign(144, 0);
            Math::putU2(body, 0, intField(f, 0)); // slot+37（解析侧 -37 得 1..24 槽位）
            Math::putU2(body, 2, intField(f, 1)); // FCN+7（解析侧 -7 得 -7..+6）
            if (f.size() > 3) {
                body[4] = static_cast<std::uint8_t>(intField(f, 2)); // satType
                body[5] = static_cast<std::uint8_t>(intField(f, 3)); // 保留
            }
            Math::putU2(body, 6, intField(f, 4)); // week
            Math::putU4(body, 8, intField(f, 5)); // tow（ms）
            Math::putU4(body, 12, intField(f, 6)); // toff
            Math::putU4(body, 16, intField(f, 7));
            Math::putU4(body, 20, intField(f, 10));
            Math::putU4(body, 24, intField(f, 11));
            for (int k = 0; k < 12; ++k) Math::putR8(body, 28 + 8 * k, field(f, 12 + k)); // X..τG
            Math::putU4(body, 124, intField(f, 24)); // E
            Math::putU4(body, 128, intField(f, 25)); // P
            Math::putU4(body, 132, intField(f, 26)); // FT
            Math::putU4(body, 136, intField(f, 27));
            Math::putU4(body, 140, intField(f, 28)); // 健康相关字
        }

        void buildBestPosBody(const std::vector<std::string> &f, std::vector<std::uint8_t> &body) {
            body.assign(32, 0);
            Math::putU4(body, 0, solStatus(f, 0));
            Math::putU4(body, 4, intField(f, 1)); // posType
            Math::putR8(body, 8, field(f, 2)); // 纬度
            Math::putR8(body, 16, field(f, 3)); // 经度
            Math::putR8(body, 24, field(f, 4)); // 高程
        }

        void buildImuBody(const std::vector<std::string> &f, std::vector<std::uint8_t> &body) {
            body.assign(40, 0);
            Math::putU4(body, 0, intField(f, 0)); // GPS 周
            Math::putR8(body, 4, field(f, 1)); // 周内秒
            static constexpr int kIdx[6] = {3, 4, 5, 6, 7, 8};
            for (int k = 0; k < 6; ++k) Math::putI4(body, 16 + 4 * k, signedField(f, kIdx[k]));
        }

        void buildImuXBody(const std::vector<std::string> &f, std::vector<std::uint8_t> &body) {
            body.assign(40, 0);
            Math::putU1(body, 1, intField(f, 1)); // IMU 型号（十进制）
            Math::putU2(body, 2, intField(f, 2));
            Math::putR8(body, 4, field(f, 3));
            static constexpr int kIdx[6] = {5, 6, 7, 8, 9, 10};
            for (int k = 0; k < 6; ++k) Math::putI4(body, 16 + 4 * k, signedField(f, kIdx[k]));
        }
    }

    bool AsciiReader::open(const std::string &filename) {
        close();
        mIn.open(Str::ACPToWide(filename), std::ios::in | std::ios::binary);
        if (!mIn) return false;

        mEphSnap = std::make_shared<EphemerisTable>();
        mImuLog.clear();
        mInsLayout = InsRecordLayout();
        mImuType = -1;

        probeRangeKind();
        return true;
    }

    void AsciiReader::close() {
        if (mIn.is_open()) mIn.close();
        LogReader::close();
    }

    void AsciiReader::restartStream() {
        mIn.clear();
        if (mIn.is_open()) mIn.seekg(0);
    }

    bool AsciiReader::getNextMessage(std::vector<std::uint8_t> &message, [[maybe_unused]] bool block) {
        std::string line;
        while (std::getline(mIn, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.size() < 8) continue;
            if (line.front() != '#' && line.front() != '%') continue;
            if (buildMessage(line, message)) return true;
        }
        return false;
    }

    bool AsciiReader::buildMessage(const std::string &line, std::vector<std::uint8_t> &message) const {
        const auto semi = line.find(';');
        if (semi == std::string::npos) return false;

        std::string name;
        int week = 0, ms = 0;
        if (!parseHead(line.substr(0, semi), name, week, ms)) return false;

        std::string text = line.substr(semi + 1);
        if (const auto star = text.find('*'); star != std::string::npos) text.resize(star); // 去掉 *CRC
        const std::vector<std::string> f = Str::split(text, ',', false);
        if (f.empty()) return false;

        std::uint16_t id = 0;
        std::vector<std::uint8_t> body;

        if (name == "RANGEA") {
            if (mRangeKind == RangeKind::Compressed) return false; // 有全量版时压缩版是重复历元
            buildRangeBody(f, body);
            id = LogID::RANGE;
        } else if (name == "RANGECMPA") {
            if (mRangeKind == RangeKind::Full) return false;
            if (!buildRangeCmpBody(f, body)) return false;
            id = LogID::RANGECMP;
        } else if (name == "GPSEPHEMA") {
            buildGpsEphemBody(f, body);
            id = LogID::GPSEPHEM;
        } else if (name == "BDSEPHEMERISA" || name == "BDSEPHEMA") {
            buildBdsEphemBody(f, body);
            id = LogID::BDSEPHEMERIS;
        } else if (name == "GALEPHEMERISA" || name == "GALEPHEMA") {
            buildGalEphemBody(f, body);
            id = LogID::GALEPHEM;
        } else if (name == "GALINAVEPHEMERISA" || name == "GALINAVEPHEMA") {
            buildGalInavEphemBody(f, body);
            id = LogID::GALEPHEM;
        } else if (name == "GALFNAVEPHEMERISA" || name == "GALFNAVEPHEMA") {
            buildGalFnavEphemBody(f, body);
            id = LogID::GALEPHEM;
        } else if (name == "GLOEPHEMERISA" || name == "GLOEPHEMA") {
            buildGloEphemBody(f, body);
            id = LogID::GLOEPHEM;
        } else if (name == "BESTPOSA") {
            buildBestPosBody(f, body);
            id = LogID::BESTPOS;
        } else if (name == "RAWIMUA" || name == "RAWIMUSA") {
            buildImuBody(f, body);
            id = name == "RAWIMUA" ? LogID::RAWIMU : LogID::RAWIMUS;
        } else if (name == "RAWIMUXA" || name == "RAWIMUSXA") {
            buildImuXBody(f, body);
            id = name == "RAWIMUXA" ? LogID::RAWIMUX : LogID::RAWIMUSX;
        } else {
            return false;
        }

        message = sealMessage(id, week, ms, body);
        return true;
    }
}
