#include "rinex/RINEXNavStore.h"
#include "StringUtils.h"
#include "TimeConvert.h"
#include "Exception.h"
#include "Log.h"

#include <vector>
#include <cctype>
#include <cmath>
namespace {

    // 解析导航首行：卫星号、历元、钟差 a0/a1/a2。
    // 首行在 RINEX2 与 RINEX3 的字段顺序、个数完全一致，差异仅「年」2 位 vs 4 位、
    // 以及 GPS 卫星号在 v2 无系统前缀（仅 2 位数字）。
    struct NavHeader {
        int prn = 0;
        CommonTime ct;
        double a0 = 0, a1 = 0, a2 = 0;
    };

    // 按「数值模式」逐个抽取浮点数，不依赖分隔符。
    // 首行的历元与钟差都是定宽字段（v3 历元 23 列、v2 22 列，其后 3×D19.12），
    // 某个值恰好填满 19 列时相邻字段之间没有空格——如
    // "...2.332665026188e-04-9.777068044059e-12..."，空白分词会把两个值粘成一个，
    // 导致字段数不足而整份星历读不出。故这里识别 [+-]?数字[.数字]([eE][+-]?数字) 的形态。
    std::vector<double> extractNumbers(const std::string &s) {
        std::vector<double> out;
        const std::size_t n = s.size();
        std::size_t i = 0;
        while (i < n) {
            if (s[i] == ' ' || s[i] == '\t' || s[i] == '\r') {
                ++i;
                continue;
            }
            const std::size_t b = i;
            if (s[i] == '+' || s[i] == '-') ++i;
            bool seenExp = false, hasDigit = false;
            while (i < n) {
                const char c = s[i];
                if (c >= '0' && c <= '9') {
                    hasDigit = true;
                    ++i;
                } else if (c == '.') {
                    ++i;
                } else if ((c == 'e' || c == 'E') && !seenExp) {
                    seenExp = true;
                    ++i;
                    if (i < n && (s[i] == '+' || s[i] == '-')) ++i; // 指数符号
                } else {
                    break;
                }
            }
            if (hasDigit) out.push_back(Str::toDouble(s.substr(b, i - b)));
            else if (i == b) ++i; // 落单的非数值字符，跳过避免死循环
        }
        return out;
    }

    NavHeader parseNavHeader(const std::string &line, double ver, const TimeSystem &ts) {
        NavHeader h;

        // RINEX 导航首行的钟差/轨道字段用 Fortran 风格 D 指数；MSVC 的 std::stod 不识 'D'，
        // 数据块各行已统一调 fortranExponent(D->e)，首行也要同样处理，否则 a0/a1/a2 会被截断成无指数值。
        std::string convLine = line;
        std::replace(convLine.begin(), convLine.end(), 'D', 'e');

        // 卫星号（首字段）：v3 形如 "G01"（系统前缀 + 2 位 PRN）；v2 GPS 仅 2 位数字无前缀，
        // GLONASS 在两种版本都带 "R" 前缀。
        const std::size_t p0 = convLine.find_first_not_of(" \t\r");
        if (p0 == std::string::npos)
            throw FFStreamError("导航首行为空");
        std::size_t p1 = convLine.find_first_of(" \t\r", p0);
        if (p1 == std::string::npos) p1 = convLine.size();
        const std::string sat = convLine.substr(p0, p1 - p0);
        if (sat.size() >= 3 && std::isalpha(static_cast<unsigned char>(sat[0])))
            h.prn = SatID(sat).id;
        else
            h.prn = Str::toInt(sat);

        // 卫星号之后的 9 个数：历元 6 字段（年月日时分秒）+ 钟差 3 字段（a0 a1 a2）
        const std::vector<double> v = extractNumbers(convLine.substr(p1));
        if (v.size() < 9)
            throw FFStreamError("导航首行字段不足 9 个: " + line);

        int yr = static_cast<int>(std::lround(v[0]));
        if (ver < 3.0) {                 // v2 年为 2 位，按 RINEX 世纪规则展开
            if (yr >= 0 && yr <= 79) yr += 2000;
            else if (yr >= 80 && yr <= 99) yr += 1900;
        }
        const int mo = static_cast<int>(std::lround(v[1]));
        const int day = static_cast<int>(std::lround(v[2]));
        const int hr = static_cast<int>(std::lround(v[3]));
        const int min = static_cast<int>(std::lround(v[4]));
        double sec = v[5];
        short ds = 0;
        if (sec >= 60.0) {
            ds = static_cast<short>(sec);
            sec = 0.0;
        }

        const CivilTime cvt(yr, mo, day, hr, min, sec, ts);
        h.ct = Time::CivilToCommon(cvt);
        if (ds != 0) h.ct += ds;

        h.a0 = v[6];
        h.a1 = v[7];
        h.a2 = v[8];
        return h;
    }
}

void RINEXNavStore::loadGPSEph(GPSEphem &eph, std::string &line) const {
    const NavHeader h = parseNavHeader(line, version, TimeSystem::GPS);
    eph.prn = h.prn;

    const WeekSecond ws = Time::CommonToWeekSecond(h.ct);
    eph.toc = ws.sow;
    eph.week = ws.week;

    eph.a0 = h.a0;
    eph.a1 = h.a1;
    eph.a2 = h.a2;


    double IODE, Crs, Delta_n, M0;
    double Cuc, ecc, Cus, sqrt_A;
    double Toe, Cic, OMEGA_0, Cis;
    double i0, Crc, omega, OMEGA_DOT;
    double IDOT;
    double URA, SV_health, TGD, IODC;
    double HOWtime;

    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        IODE = Str::toDouble(l.substr(4, 19));
        Crs = Str::toDouble(l.substr(23, 19));
        Delta_n = Str::toDouble(l.substr(42, 19));
        M0 = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        Cuc = Str::toDouble(l.substr(4, 19));
        ecc = Str::toDouble(l.substr(23, 19));
        Cus = Str::toDouble(l.substr(42, 19));
        sqrt_A = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        Toe = Str::toDouble(l.substr(4, 19));
        Cic = Str::toDouble(l.substr(23, 19));
        OMEGA_0 = Str::toDouble(l.substr(42, 19));
        Cis = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        i0 = Str::toDouble(l.substr(4, 19));
        Crc = Str::toDouble(l.substr(23, 19));
        omega = Str::toDouble(l.substr(42, 19));
        OMEGA_DOT = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        IDOT = Str::toDouble(l.substr(4, 19));



    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        URA = Str::toDouble(l.substr(4, 19));
        SV_health = Str::toDouble(l.substr(23, 19));
        TGD = Str::toDouble(l.substr(42, 19));
        IODC = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        HOWtime = Str::toDouble(l.substr(4, 19));

    }



    eph.toe = Toe;
    eph.m0 = M0;
    eph.e = ecc;
    eph.rootA = sqrt_A;
    eph.dn = Delta_n;
    eph.i0 = i0;
    eph.omega0 = OMEGA_0;
    eph.omega = omega;
    eph.omegaDot = OMEGA_DOT;
    eph.idot = IDOT;
    eph.cuc = Cuc;
    eph.cus = Cus;
    eph.crc = Crc;
    eph.crs = Crs;
    eph.cic = Cic;
    eph.cis = Cis;
    eph.health = static_cast<unsigned int>(SV_health);
    eph.ura = URA;
    eph.IODE = static_cast<unsigned int>(IODE);
    eph.IODC = static_cast<unsigned int>(IODC);
    eph.tgd = TGD;

    if (HOWtime >= 0.0 && HOWtime < FULL_WEEK) {
        if (HOWtime - eph.toe > HALF_WEEK && eph.week > 0) {
            eph.week--;
        } else if (eph.toe - HOWtime > HALF_WEEK) {
            eph.week++;
        }
    }
}

void RINEXNavStore::loadBDSEph(BDSEphem &eph, std::string &line) const {
    const NavHeader h = parseNavHeader(line, version, TimeSystem::BDT);
    eph.prn = h.prn;

    // 周数必须按**本系统**纪元折算：BDSEphem::timeSystem = BDT，getCommonTime 会用
    // BDT 纪元（MJD 53736）去还原，这里若按 GPS 纪元（44244）取周，会得到 GPS 周 2442，
    // 再由 BDT 纪元展开 → 时间整整多出 1356 周（9492 天），星历全程匹配不上。
    const WeekSecond ws = Time::CommonToWeekSecond(h.ct, TimeSystem::BDT);
    eph.toc = ws.sow;
    eph.week = ws.week;

    eph.a0 = h.a0;
    eph.a1 = h.a1;
    eph.a2 = h.a2;


    double IODE, Crs, Delta_n, M0;
    double Cuc, ecc, Cus, sqrt_A;
    double Toe, Cic, OMEGA_0, Cis;
    double i0, Crc, omega, OMEGA_DOT;

    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        IODE = Str::toDouble(l.substr(4, 19));
        Crs = Str::toDouble(l.substr(23, 19));
        Delta_n = Str::toDouble(l.substr(42, 19));
        M0 = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        Cuc = Str::toDouble(l.substr(4, 19));
        ecc = Str::toDouble(l.substr(23, 19));
        Cus = Str::toDouble(l.substr(42, 19));
        sqrt_A = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        Toe = Str::toDouble(l.substr(4, 19));
        Cic = Str::toDouble(l.substr(23, 19));
        OMEGA_0 = Str::toDouble(l.substr(42, 19));
        Cis = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        i0 = Str::toDouble(l.substr(4, 19));
        Crc = Str::toDouble(l.substr(23, 19));
        omega = Str::toDouble(l.substr(42, 19));
        OMEGA_DOT = Str::toDouble(l.substr(61, 19));
    }


    std::string l5;
    if (!nextLine(l5)) throw EndOfFile("nav data truncated");
    Str::stripTrailing(l5);
    fortranExponent(l5);
    double IDOT = Str::toDouble(l5.substr(4, 19));



    std::string l6;
    if (!nextLine(l6)) throw EndOfFile("nav data truncated");
    Str::stripTrailing(l6);
    fortranExponent(l6);
    double SatH1 = Str::toDouble(l6.substr(4, 19));
    double TGD1 = Str::toDouble(l6.substr(23, 19));
    double TGD2 = Str::toDouble(l6.substr(42, 19));
    double TGD3 = Str::toDouble(l6.substr(61, 19));
    bool shifted = std::fabs(TGD1) < 1e-12 && (std::fabs(TGD2) > 1e-12 || std::fabs(TGD3) > 1e-12);
    double tgdB1 = shifted ? TGD2 : TGD1;
    double tgdB2 = shifted ? TGD3 : TGD2;

    std::string _discard;
    nextLine(_discard);

    eph.toe = Toe;
    eph.m0 = M0;
    eph.e = ecc;
    eph.rootA = sqrt_A;
    eph.dn = Delta_n;
    eph.i0 = i0;
    eph.omega0 = OMEGA_0;
    eph.omega = omega;
    eph.omegaDot = OMEGA_DOT;
    eph.idot = IDOT;
    eph.cuc = Cuc;
    eph.cus = Cus;
    eph.crc = Crc;
    eph.crs = Crs;
    eph.cic = Cic;
    eph.cis = Cis;
    eph.health = static_cast<unsigned int>(SatH1);
    eph.ura = 2.0;
    eph.AODE = static_cast<unsigned int>(IODE);
    eph.AODC = static_cast<unsigned int>(IODE);
    eph.tgd1 = tgdB1;
    eph.tgd2 = tgdB2;
}

void RINEXNavStore::loadGalEph(GalileoEphem &eph, std::string &line) const {
    const NavHeader h = parseNavHeader(line, version, TimeSystem::GAL);
    eph.prn = h.prn;

    // 同 loadBDSEph：GalileoEphem::timeSystem = GAL，周数要用 GAL 纪元（MJD 51412）折算，
    // 用默认的 GPS 纪元会多出 1024 周（7168 天）。
    const WeekSecond ws = Time::CommonToWeekSecond(h.ct, TimeSystem::GAL);
    eph.toc = ws.sow;
    eph.week = ws.week;

    eph.a0 = h.a0;
    eph.a1 = h.a1;
    eph.a2 = h.a2;


    double IODE, Crs, Delta_n, M0;
    double Cuc, ecc, Cus, sqrt_A;
    double Toe, Cic, OMEGA_0, Cis;
    double i0, Crc, omega, OMEGA_DOT;
    double IDOT;
    double SISA, SV_health, BGD_E5a_E1;
    double HOWtime;

    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        IODE = Str::toDouble(l.substr(4, 19));
        Crs = Str::toDouble(l.substr(23, 19));
        Delta_n = Str::toDouble(l.substr(42, 19));
        M0 = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        Cuc = Str::toDouble(l.substr(4, 19));
        ecc = Str::toDouble(l.substr(23, 19));
        Cus = Str::toDouble(l.substr(42, 19));
        sqrt_A = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        Toe = Str::toDouble(l.substr(4, 19));
        Cic = Str::toDouble(l.substr(23, 19));
        OMEGA_0 = Str::toDouble(l.substr(42, 19));
        Cis = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        i0 = Str::toDouble(l.substr(4, 19));
        Crc = Str::toDouble(l.substr(23, 19));
        omega = Str::toDouble(l.substr(42, 19));
        OMEGA_DOT = Str::toDouble(l.substr(61, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        IDOT = Str::toDouble(l.substr(4, 19));
    }
    // 第 4 列 BGD(E5b,E1) 本项目用不到（只有 E1/E5a 单频才需要 BGD(E5a,E1)）
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        SISA = Str::toDouble(l.substr(4, 19));
        SV_health = Str::toDouble(l.substr(23, 19));
        BGD_E5a_E1 = Str::toDouble(l.substr(42, 19));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        HOWtime = Str::toDouble(l.substr(4, 19));
    }


    eph.toe = Toe;
    eph.m0 = M0;
    eph.e = ecc;
    eph.rootA = sqrt_A;
    eph.dn = Delta_n;
    eph.i0 = i0;
    eph.omega0 = OMEGA_0;
    eph.omega = omega;
    eph.omegaDot = OMEGA_DOT;
    eph.idot = IDOT;
    eph.cuc = Cuc;
    eph.cus = Cus;
    eph.crc = Crc;
    eph.crs = Crs;
    eph.cic = Cic;
    eph.cis = Cis;
    eph.health = static_cast<unsigned int>(SV_health);
    eph.ura = SISA;
    eph.tgd = BGD_E5a_E1;

    if (HOWtime >= 0.0 && HOWtime < FULL_WEEK) {
        if (HOWtime - eph.toe > HALF_WEEK && eph.week > 0) {
            eph.week--;
        } else if (eph.toe - HOWtime > HALF_WEEK) {
            eph.week++;
        }
    }
}

void RINEXNavStore::loadGloEph(GlonassEphem &eph, const std::string &line) const {
    const NavHeader h = parseNavHeader(line, version, TimeSystem::UTC);
    eph.prn = h.prn;

    eph.toeCommon = h.ct;

    eph.tauN = -h.a0;
    eph.gamma = h.a1; // RINEX 列 "SV relative frequency bias" = γ，与 NovAtel 的 H+116 同义
    eph.tk = h.a2;

    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        eph.X = Str::toDouble(l.substr(4, 19));
        eph.vX = Str::toDouble(l.substr(23, 19));
        eph.aX = Str::toDouble(l.substr(42, 19));
        eph.health = static_cast<unsigned int>(Str::toDouble(l.substr(61, 19)));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        eph.Y = Str::toDouble(l.substr(4, 19));
        eph.vY = Str::toDouble(l.substr(23, 19));
        eph.aY = Str::toDouble(l.substr(42, 19));
        eph.freqNum = static_cast<int>(Str::toDouble(l.substr(61, 19)));
    }
    {
        std::string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        Str::stripTrailing(l);
        fortranExponent(l);
        eph.Z = Str::toDouble(l.substr(4, 19));
        eph.vZ = Str::toDouble(l.substr(23, 19));
        eph.aZ = Str::toDouble(l.substr(42, 19));

    }
}

void RINEXNavStore::loadFile(const std::string &file, EphemerisTable &ephTable) {
    if (file.empty()) {
        throw FileMissingException("nav file path is empty");
    }

    try {
        openFile(file);
    } catch (const FFStreamError &) {
        throw FileMissingException("can't open nav file: " + file);
    }


    while (true) {
        std::string line;

        if (!nextLine(line)) break;
        Str::stripTrailing(line);

        if (line.empty()) continue;
        if (line.length() < 60) {
            throw FFStreamError("Invalid line length, may need dos2unix: " + line);
        }

        if (const std::string label = rinexLabel(line); label == "RINEX VERSION / TYPE") {
            const auto [tVer, tType, tSys] = parseRinexVersionType(line);
            version = tVer;
            fileType = tType;

            // RINEX2 与 RINEX3 的导航记录差异仅在首行（2 位 PRN + 2 位年 vs 3 字符卫星号 + 4 位年），
            // 数据块列位置一致；首行统一用空白分词解析，故 v2/v3 均可读。
            if (const char c = fileType.empty() ? ' ' : fileType[0];
                c != 'N' && c != 'n' && c != 'G' && c != 'g' &&
                c != 'E' && c != 'e' && c != 'M' && c != 'm') {
                throw FFStreamError("File type is not NAVIGATION: " + fileType);
            }
            fileSys = tSys;
            fileType = "NAVIGATION";
        } else if (label == "PGM / RUN BY / DATE") {
            fileProgram = Str::strip(line.substr(0, 20));
            fileAgency = Str::strip(line.substr(20, 20));
            date = Str::strip(line.substr(40, 20));
        } else if (label == "COMMENT") {
            commentList.push_back(Str::strip(line.substr(0, 60)));
        } else if (label == "IONOSPHERIC CORR") {
            std::string type = Str::strip(line.substr(0, 4));
            std::vector<double> coeff;
            coeff.reserve(4);
            for (int i = 0; i < 4; i++)
                coeff.push_back(Str::toDouble(line.substr(5 + 12 * i, 12)));
            ionoCorrData[type] = coeff;
        } else if (label == "TIME SYSTEM CORR") {
            TimeSysCorr tsc{};
            tsc.A0 = Str::toDouble(line.substr(5, 17));
            tsc.A1 = Str::toDouble(line.substr(22, 16));
            tsc.refSOW = Str::toInt(line.substr(38, 7));
            tsc.refWeek = Str::toInt(line.substr(45, 5));
            timeSysCorrData[Str::strip(line.substr(0, 4))] = tsc;
        } else if (label == "LEAP SECONDS") {
            leapSeconds = Str::toInt(line.substr(0, 6));
            leapDelta = Str::toInt(line.substr(6, 6));
            leapWeek = Str::toInt(line.substr(12, 6));
            leapDay = Str::toInt(line.substr(18, 6));
        } else if (label == "END OF HEADER") {
            break;
        }
    }


    std::string line;
    while (nextLine(line)) {
        fortranExponent(line);

        // 跳过空行或纯空白行；注意 RINEX2 GPS 记录首字符可能是空格（PRN 左补位），不能据此跳过。
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;

        // RINEX3 每条记录自带系统前缀（G/C/E/R）；RINEX2 单系统文件的 GPS 记录以数字
        // 开头，故遇到 v2 且首字符非字母时按 GPS 处理（GLONASS 仍带 R 前缀）。
        char sys = line[0];
        if (version < 3.0 && !std::isalpha(static_cast<unsigned char>(sys))) sys = 'G';

        if (sys == 'G') {
            auto eph = std::make_shared<GPSEphem>();
            loadGPSEph(*eph, line);
            ephTable.gps[eph->prn].push_back(eph);
        } else if (sys == 'C') {
            auto eph = std::make_shared<BDSEphem>();
            loadBDSEph(*eph, line);
            ephTable.bds[eph->prn].push_back(eph);
        } else if (sys == 'E') {
            auto eph = std::make_shared<GalileoEphem>();
            loadGalEph(*eph, line);
            ephTable.gal[eph->prn].push_back(eph);
        } else if (sys == 'R') {
            auto eph = std::make_shared<GlonassEphem>();
            loadGloEph(*eph, line);
            ephTable.glo[eph->prn].push_back(eph);
        }
    }
}
