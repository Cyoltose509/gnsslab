#include "RinexNavStore.h"
#include "StringUtils.h"
#include "TimeConvert.h"
#include "Exception.h"
#include "Log.h"

using namespace std;

namespace {
    /// 从 RINEX epoch 行解析 CommonTime
    CommonTime parseEpoch(const string &line, const TimeSystem &ts = TimeSystem::GPS) {
        const int yr = safeStoi(line.substr(4, 4));
        const int mo = safeStoi(line.substr(9, 2));
        const int day = safeStoi(line.substr(12, 2));
        const int hr = safeStoi(line.substr(15, 2));
        const int min = safeStoi(line.substr(18, 2));
        double sec = safeStod(line.substr(21, 2));
        short ds = 0;
        if (sec >= 60.0) {
            ds = static_cast<short>(sec);
            sec = 0.0;
        }

        const CivilTime cvt(yr, mo, day, hr, min, sec, ts);
        CommonTime ct = CivilTime2CommonTime(cvt);
        if (ds != 0) ct += ds;
        return ct;
    }
}

void RinexNavStore::loadGPSEph(GPSEphem &eph, string &line) const {
    SatID sat(line.substr(0, 3));
    eph.prn = sat.id;

    // Epoch line = time of clock (toc)
    CommonTime ctToc = parseEpoch(line, TimeSystem::GPS);
    WeekSecond ws;
    CommonTime2WeekSecond(ctToc, ws);
    eph.toc = ws.sow; // 时间钟参数 (TOC)
    eph.week = ws.week; // 从 TOC 时间推算

    eph.a0 = safeStod(line.substr(23, 19));
    eph.a1 = safeStod(line.substr(42, 19));
    eph.a2 = safeStod(line.substr(61, 19));

    // 读 7 行轨道参数（每行 4 个字段，各 19 列宽，起始列 4）
    double IODE, Crs, Delta_n, M0;
    double Cuc, ecc, Cus, sqrt_A;
    double Toe, Cic, OMEGA_0, Cis;
    double i0, Crc, omega, OMEGA_DOT;
    double IDOT;
    double URA, SV_health, TGD, IODC;
    double HOWtime;

    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        IODE = safeStod(l.substr(4, 19));
        Crs = safeStod(l.substr(23, 19));
        Delta_n = safeStod(l.substr(42, 19));
        M0 = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        Cuc = safeStod(l.substr(4, 19));
        ecc = safeStod(l.substr(23, 19));
        Cus = safeStod(l.substr(42, 19));
        sqrt_A = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        Toe = safeStod(l.substr(4, 19));
        Cic = safeStod(l.substr(23, 19));
        OMEGA_0 = safeStod(l.substr(42, 19));
        Cis = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        i0 = safeStod(l.substr(4, 19));
        Crc = safeStod(l.substr(23, 19));
        omega = safeStod(l.substr(42, 19));
        OMEGA_DOT = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        IDOT = safeStod(l.substr(4, 19));
        //L2Codes = safeStod(l.substr(23, 19));
        //GPSWeek = safeStod(l.substr(42, 19));
        //L2Pflag = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        URA = safeStod(l.substr(4, 19));
        SV_health = safeStod(l.substr(23, 19));
        TGD = safeStod(l.substr(42, 19));
        IODC = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        HOWtime = safeStod(l.substr(4, 19));
        //fitInterval = safeStod(l.substr(23, 19));
    }

    // ---- 填入 GPSEphem 字段 ----
    // eph.week / eph.toc already set from ws above
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

void RinexNavStore::loadBDSEph(BDSEphem &eph, string &line) const {
    SatID sat(line.substr(0, 3));
    eph.prn = sat.id;

    CommonTime ctToc = parseEpoch(line, TimeSystem::BDT);
    WeekSecond ws;
    CommonTime2WeekSecond(ctToc, ws);
    eph.toc = ws.sow;
    eph.week = ws.week;

    eph.a0 = safeStod(line.substr(23, 19));
    eph.a1 = safeStod(line.substr(42, 19));
    eph.a2 = safeStod(line.substr(61, 19));

    // 行 1-4：轨道参数（与 GPS 完全相同的字段布局）
    double IODE, Crs, Delta_n, M0;
    double Cuc, ecc, Cus, sqrt_A;
    double Toe, Cic, OMEGA_0, Cis;
    double i0, Crc, omega, OMEGA_DOT;

    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        IODE = safeStod(l.substr(4, 19));
        Crs = safeStod(l.substr(23, 19));
        Delta_n = safeStod(l.substr(42, 19));
        M0 = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        Cuc = safeStod(l.substr(4, 19));
        ecc = safeStod(l.substr(23, 19));
        Cus = safeStod(l.substr(42, 19));
        sqrt_A = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        Toe = safeStod(l.substr(4, 19));
        Cic = safeStod(l.substr(23, 19));
        OMEGA_0 = safeStod(l.substr(42, 19));
        Cis = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        i0 = safeStod(l.substr(4, 19));
        Crc = safeStod(l.substr(23, 19));
        omega = safeStod(l.substr(42, 19));
        OMEGA_DOT = safeStod(l.substr(61, 19));
    }

    // 行 5：IDOT, (blank), BDSWeek, (blank)
    string l5;
    if (!nextLine(l5)) throw EndOfFile("nav data truncated");
    stripTrailing(l5);
    fortranExponent(l5);
    double IDOT = safeStod(l5.substr(4, 19));
    //double BDSWeek = safeStod(l5.substr(42, 19));

    // 行 6：SatH1, TGD1, TGD2, TransTime (BDS 秒)
    string l6;
    if (!nextLine(l6)) throw EndOfFile("nav data truncated");
    stripTrailing(l6);
    fortranExponent(l6);
    double SatH1 = safeStod(l6.substr(4, 19));
    double TGD1 = safeStod(l6.substr(23, 19));
    double TGD2 = safeStod(l6.substr(42, 19));
    double TGD3 = safeStod(l6.substr(61, 19)); // 第 4 字段
    bool shifted = fabs(TGD1) < 1e-12 && (fabs(TGD2) > 1e-12 || fabs(TGD3) > 1e-12);
    double tgdB1 = shifted ? TGD2 : TGD1; // B1/B3 群时延
    double tgdB2 = shifted ? TGD3 : TGD2; // B2/B3 群时延

    string _discard;
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
    eph.AODC = static_cast<unsigned int>(IODE); // 近似等同 AODE
    eph.tgd1 = tgdB1;
    eph.tgd2 = tgdB2;
}

void RinexNavStore::loadGalEph(GalileoEphem &eph, string &line) const {
    SatID sat(line.substr(0, 3));
    eph.prn = sat.id;

    // Epoch line = time of clock (toc)
    CommonTime ctToc = parseEpoch(line, TimeSystem::GAL);
    WeekSecond ws;
    CommonTime2WeekSecond(ctToc, ws);
    eph.toc = ws.sow; // 时间钟参数 (TOC)
    eph.week = ws.week; // 从 TOC 时间推算

    eph.a0 = safeStod(line.substr(23, 19));
    eph.a1 = safeStod(line.substr(42, 19));
    eph.a2 = safeStod(line.substr(61, 19));

    // 读 7 行轨道参数（每行 4 个字段，各 19 列宽，起始列 4）
    double IODE, Crs, Delta_n, M0;
    double Cuc, ecc, Cus, sqrt_A;
    double Toe, Cic, OMEGA_0, Cis;
    double i0, Crc, omega, OMEGA_DOT;
    double IDOT;
    double URA, SV_health, TGD, IODC;
    double HOWtime;

    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        IODE = safeStod(l.substr(4, 19));
        Crs = safeStod(l.substr(23, 19));
        Delta_n = safeStod(l.substr(42, 19));
        M0 = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        Cuc = safeStod(l.substr(4, 19));
        ecc = safeStod(l.substr(23, 19));
        Cus = safeStod(l.substr(42, 19));
        sqrt_A = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        Toe = safeStod(l.substr(4, 19));
        Cic = safeStod(l.substr(23, 19));
        OMEGA_0 = safeStod(l.substr(42, 19));
        Cis = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        i0 = safeStod(l.substr(4, 19));
        Crc = safeStod(l.substr(23, 19));
        omega = safeStod(l.substr(42, 19));
        OMEGA_DOT = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        IDOT = safeStod(l.substr(4, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        URA = safeStod(l.substr(4, 19));
        SV_health = safeStod(l.substr(23, 19));
        TGD = safeStod(l.substr(42, 19));
        IODC = safeStod(l.substr(61, 19));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        HOWtime = safeStod(l.substr(4, 19));
    }

    // ---- 填入 GalEph 字段 ----
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
    eph.tgd = TGD;

    if (HOWtime >= 0.0 && HOWtime < FULL_WEEK) {
        if (HOWtime - eph.toe > HALF_WEEK && eph.week > 0) {
            eph.week--;
        } else if (eph.toe - HOWtime > HALF_WEEK) {
            eph.week++;
        }
    }
}

void RinexNavStore::loadGloEph(GlonassEphem &eph, const string &line) const {
    const SatID sat(line.substr(0, 3));
    eph.prn = sat.id;

    const CommonTime ctToc = parseEpoch(line, TimeSystem::UTC);
    eph.toeCommon = ctToc;

    eph.tauN = -safeStod(line.substr(23, 19));
    eph.tauD = safeStod(line.substr(42, 19));
    eph.tk = safeStod(line.substr(61, 19));

    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        eph.X = safeStod(l.substr(4, 19));
        eph.vX = safeStod(l.substr(23, 19));
        eph.aX = safeStod(l.substr(42, 19));
        eph.health = static_cast<unsigned int>(safeStod(l.substr(61, 19)));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        eph.Y = safeStod(l.substr(4, 19));
        eph.vY = safeStod(l.substr(23, 19));
        eph.aY = safeStod(l.substr(42, 19));
        eph.freqNum = static_cast<int>(safeStod(l.substr(61, 19)));
    }
    {
        string l;
        if (!nextLine(l)) throw EndOfFile("nav data truncated");
        stripTrailing(l);
        fortranExponent(l);
        eph.Z = safeStod(l.substr(4, 19));
        eph.vZ = safeStod(l.substr(23, 19));
        eph.aZ = safeStod(l.substr(42, 19));
        // 第 4 字段 = 龄期 / operator info，此处忽略
    }
}

void RinexNavStore::loadFile(const string &file, EphemerisTable &ephTable) {
    if (file.empty()) {
        throw FileMissingException("nav file path is empty");
    }

    try {
        openFile(file);
    } catch (const FFStreamError &) {
        throw FileMissingException("can't open nav file: " + file);
    }

    // ---- 读 header ----
    while (true) {
        string line;
        // 文件为空/截断(无 END OF HEADER 即遇 EOF)：安全退出，避免空行 continue 死循环。
        if (!nextLine(line)) break;
        stripTrailing(line);

        if (line.empty()) continue;
        if (line.length() < 60) {
            throw FFStreamError("Invalid line length, may need dos2unix: " + line);
        }

        if (const string label = getLabel(line); label == "RINEX VERSION / TYPE") {
            version = safeStod(line.substr(0, 20));
            fileType = strip(line.substr(20, 20));
            // 导航文件类型字符：N=GPS, G=GLONASS, E=Galileo, M=混合, C=钟差…
            if (const char c = fileType.empty() ? ' ' : fileType[0];
                c != 'N' && c != 'n' && c != 'G' && c != 'g' &&
                c != 'E' && c != 'e' && c != 'M' && c != 'm') {
                throw FFStreamError("File type is not NAVIGATION: " + fileType);
            }
            fileSys = strip(line.substr(40, 20));
            fileType = "NAVIGATION";
        } else if (label == "PGM / RUN BY / DATE") {
            fileProgram = strip(line.substr(0, 20));
            fileAgency = strip(line.substr(20, 20));
            date = strip(line.substr(40, 20));
        } else if (label == "COMMENT") {
            commentList.push_back(strip(line.substr(0, 60)));
        } else if (label == "IONOSPHERIC CORR") {
            string type = strip(line.substr(0, 4));
            vector<double> coeff;
            coeff.reserve(4);
            for (int i = 0; i < 4; i++)
                coeff.push_back(safeStod(line.substr(5 + 12 * i, 12)));
            ionoCorrData[type] = coeff;
        } else if (label == "TIME SYSTEM CORR") {
            TimeSysCorr tsc{};
            tsc.A0 = safeStod(line.substr(5, 17));
            tsc.A1 = safeStod(line.substr(22, 16));
            tsc.refSOW = safeStoi(line.substr(38, 7));
            tsc.refWeek = safeStoi(line.substr(45, 5));
            timeSysCorrData[strip(line.substr(0, 4))] = tsc;
        } else if (label == "LEAP SECONDS") {
            leapSeconds = safeStoi(line.substr(0, 6));
            leapDelta = safeStoi(line.substr(6, 6));
            leapWeek = safeStoi(line.substr(12, 6));
            leapDay = safeStoi(line.substr(18, 6));
        } else if (label == "END OF HEADER") {
            break;
        }
    }

    // ---- 读星历数据 ----
    string line;
    while (nextLine(line)) {
        fortranExponent(line);

        if (line.empty() || line[0] == ' ') continue; // 空白行跳过

        if (line[0] == 'G') {
            auto eph = std::make_shared<GPSEphem>();
            loadGPSEph(*eph, line);
            ephTable.gps[eph->prn].push_back(eph);
        } else if (line[0] == 'C') {
            auto eph = std::make_shared<BDSEphem>();
            loadBDSEph(*eph, line);
            ephTable.bds[eph->prn].push_back(eph);
        } else if (line[0] == 'E') {
            auto eph = std::make_shared<GalileoEphem>();
            loadGalEph(*eph, line);
            ephTable.gal[eph->prn].push_back(eph);
        } else if (line[0] == 'R') {
            auto eph = std::make_shared<GlonassEphem>();
            loadGloEph(*eph, line);
            ephTable.glo[eph->prn].push_back(eph);
        }
    }
}
