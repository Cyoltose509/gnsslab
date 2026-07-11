#include "RinexNavStore.h"
#include "StringUtils.h"
#include "TimeConvert.h"
#include "Exception.h"

using namespace std;

namespace {
    /// 读一行并替换科学计数法的 'D' → 'e'
    string nextLine(fstream &navFile) {
        string line;
        getline(navFile, line);
        if (!line.empty()) stripTrailing(line);
        replace(line.begin(), line.end(), 'D', 'e');
        return line;
    }

    /// 从 RINEX epoch 行解析 CommonTime
    CommonTime parseEpoch(const string &line) {
        const int yr  = safeStoi(line.substr(4, 4));
        const int mo  = safeStoi(line.substr(9, 2));
        const int day = safeStoi(line.substr(12, 2));
        const int hr  = safeStoi(line.substr(15, 2));
        const int min = safeStoi(line.substr(18, 2));
        double sec = safeStod(line.substr(21, 2));

        short ds = 0;
        if (sec >= 60.0) { ds = static_cast<short>(sec); sec = 0.0; }

        CivilTime cvt(yr, mo, day, hr, min, sec);
        CommonTime ct = CivilTime2CommonTime(cvt);
        if (ds != 0) ct += ds;
        return ct;
    }
}

// ============================================================================
// GPS 星历解析
// ============================================================================
void RinexNavStore::loadGPSEph(GPSEphem &eph, string &line, fstream &navFile) {
    SatID sat(line.substr(0, 3));
    eph.prn = sat.id;

    // Epoch line = time of clock (toc)
    CommonTime ctToc = parseEpoch(line);
    ctToc.setTimeSystem(TimeSystem::GPS);
    WeekSecond ws;
    CommonTime2WeekSecond(ctToc, ws);
    eph.toc = ws.sow;
    eph.week = ws.week;  // 从 TOC 时间推算，不依赖数据字段

    eph.a0 = safeStod(line.substr(23, 19));
    eph.a1 = safeStod(line.substr(42, 19));
    eph.a2 = safeStod(line.substr(61, 19));

    // 读 7 行轨道参数（每行 4 个字段，各 19 列宽，起始列 4）
    double IODE, Crs, Delta_n, M0;
    double Cuc, ecc, Cus, sqrt_A;
    double Toe, Cic, OMEGA_0, Cis;
    double i0,  Crc, omega, OMEGA_DOT;
    double IDOT, L2Codes, GPSWeek, L2Pflag;
    double URA, SV_health, TGD, IODC;
    double HOWtime, fitInterval;

    {
        string l = nextLine(navFile);
        IODE     = safeStod(l.substr( 4, 19));
        Crs      = safeStod(l.substr(23, 19));
        Delta_n  = safeStod(l.substr(42, 19));
        M0       = safeStod(l.substr(61, 19));
    }
    {
        string l = nextLine(navFile);
        Cuc      = safeStod(l.substr( 4, 19));
        ecc      = safeStod(l.substr(23, 19));
        Cus      = safeStod(l.substr(42, 19));
        sqrt_A   = safeStod(l.substr(61, 19));
    }
    {
        string l = nextLine(navFile);
        Toe      = safeStod(l.substr( 4, 19));
        Cic      = safeStod(l.substr(23, 19));
        OMEGA_0  = safeStod(l.substr(42, 19));
        Cis      = safeStod(l.substr(61, 19));
    }
    {
        string l = nextLine(navFile);
        i0       = safeStod(l.substr( 4, 19));
        Crc      = safeStod(l.substr(23, 19));
        omega    = safeStod(l.substr(42, 19));
        OMEGA_DOT= safeStod(l.substr(61, 19));
    }
    {
        string l = nextLine(navFile);
        IDOT     = safeStod(l.substr( 4, 19));
        L2Codes  = safeStod(l.substr(23, 19));
        GPSWeek  = safeStod(l.substr(42, 19));
        L2Pflag  = safeStod(l.substr(61, 19));
    }
    {
        string l = nextLine(navFile);
        URA      = safeStod(l.substr( 4, 19));
        SV_health= safeStod(l.substr(23, 19));
        TGD      = safeStod(l.substr(42, 19));
        IODC     = safeStod(l.substr(61, 19));
    }
    {
        string l = nextLine(navFile);
        HOWtime  = safeStod(l.substr( 4, 19));
        fitInterval = safeStod(l.substr(23, 19));
    }

    // ---- 填入 GPSEphem 字段 ----
    // eph.week already set from ws.week above (TOC time)
    eph.toe  = Toe;
    eph.m0   = M0;
    eph.e    = ecc;
    eph.rootA = sqrt_A;
    eph.dn   = Delta_n;
    eph.i0   = i0;
    eph.omega0   = OMEGA_0;
    eph.omega    = omega;
    eph.omegaDot = OMEGA_DOT;
    eph.idot = IDOT;
    eph.cuc  = Cuc;   eph.cus = Cus;
    eph.crc  = Crc;   eph.crs = Crs;
    eph.cic  = Cic;   eph.cis = Cis;
    eph.health = static_cast<unsigned int>(SV_health);
    eph.ura  = URA;
    eph.IODE = static_cast<unsigned int>(IODE);
    eph.IODC = static_cast<unsigned int>(IODC);
    eph.tgd  = TGD;

    // 修正 week：RINEX 中的 week 是 TOE 所在周
    while (HOWtime < 0) { HOWtime += static_cast<long>(FULL_WEEK); eph.week--; }
    if (HOWtime - eph.toe > HALF_WEEK)       eph.week--;
    else if (eph.toe - HOWtime > HALF_WEEK)  eph.week++;
}

// ============================================================================
// BDS 星历解析（RINEX 3.02，8 行格式）
// ============================================================================
void RinexNavStore::loadBDSEph(BDSEphem &eph, string &line, fstream &navFile) {
    SatID sat(line.substr(0, 3));
    eph.prn = sat.id;

    CommonTime ctToc = parseEpoch(line);
    ctToc.setTimeSystem(TimeSystem::BDT);
    WeekSecond ws;
    CommonTime2WeekSecond(ctToc, ws);
    eph.toc = ws.sow;

    eph.a0 = safeStod(line.substr(23, 19));
    eph.a1 = safeStod(line.substr(42, 19));
    eph.a2 = safeStod(line.substr(61, 19));

    // 行 1-4：轨道参数（与 GPS 完全相同的字段布局）
    double IODE, Crs, Delta_n, M0;
    double Cuc, ecc, Cus, sqrt_A;
    double Toe, Cic, OMEGA_0, Cis;
    double i0,  Crc, omega, OMEGA_DOT;

    {
        string l = nextLine(navFile);
        IODE     = safeStod(l.substr( 4, 19));
        Crs      = safeStod(l.substr(23, 19));
        Delta_n  = safeStod(l.substr(42, 19));
        M0       = safeStod(l.substr(61, 19));
    }
    {
        string l = nextLine(navFile);
        Cuc       = safeStod(l.substr( 4, 19));
        ecc       = safeStod(l.substr(23, 19));
        Cus       = safeStod(l.substr(42, 19));
        sqrt_A    = safeStod(l.substr(61, 19));
    }
    {
        string l = nextLine(navFile);
        Toe       = safeStod(l.substr( 4, 19));
        Cic       = safeStod(l.substr(23, 19));
        OMEGA_0   = safeStod(l.substr(42, 19));
        Cis       = safeStod(l.substr(61, 19));
    }
    {
        string l = nextLine(navFile);
        i0        = safeStod(l.substr( 4, 19));
        Crc       = safeStod(l.substr(23, 19));
        omega     = safeStod(l.substr(42, 19));
        OMEGA_DOT = safeStod(l.substr(61, 19));
    }

    // 行 5：IDOT, (blank), BDSWeek, (blank)
    string l5 = nextLine(navFile);
    double IDOT    = safeStod(l5.substr(4, 19));
    double BDSWeek = safeStod(l5.substr(42, 19));

    // 行 6：SatH1, TGD1, TGD2, TransTime (BDS 秒)
    string l6 = nextLine(navFile);
    double SatH1 = safeStod(l6.substr(4, 19));
    double TGD1  = safeStod(l6.substr(23, 19));
    double TGD2  = safeStod(l6.substr(42, 19));

    // 行 7：(blank) × 4 — 跳过
    nextLine(navFile);

    // ---- 填入 BDSEphem 字段 ----
    eph.week = static_cast<unsigned int>(BDSWeek);
    eph.toe  = Toe;
    eph.m0   = M0;
    eph.e    = ecc;
    eph.rootA = sqrt_A;
    eph.dn   = Delta_n;
    eph.i0   = i0;
    eph.omega0   = OMEGA_0;
    eph.omega    = omega;
    eph.omegaDot = OMEGA_DOT;
    eph.idot = IDOT;
    eph.cuc  = Cuc;   eph.cus = Cus;
    eph.crc  = Crc;   eph.crs = Crs;
    eph.cic  = Cic;   eph.cis = Cis;
    eph.health = static_cast<unsigned int>(SatH1);
    eph.ura  = 2.0;                         // RINEX BDS 无直接 URA 字段
    eph.AODE = static_cast<unsigned int>(IODE);
    eph.AODC = static_cast<unsigned int>(IODE);  // 近似等同 AODE
    eph.tgd1 = TGD1;
    eph.tgd2 = TGD2;

}

// ============================================================================
// loadFile — 主入口：读 header → 循环读星历数据行 → 写入 EphemerisTable
// ============================================================================
void RinexNavStore::loadFile(const string &file, EphemerisTable &ephTable) {
    if (file.empty()) {
        throw FileMissingException("nav file path is empty");
    }

    fstream navFileStream(file.c_str(), ios::in);
    if (!navFileStream) {
        throw FileMissingException("can't open nav file: " + file);
    }

    // ---- 读 header ----
    while (true) {
        string line;
        getline(navFileStream, line);
        stripTrailing(line);

        if (line.empty()) continue;
        if (line.length() < 60) {
            throw FFStreamError("Invalid line length, may need dos2unix: " + line);
        }

        const string label = strip(line.substr(60, 20));

        if (label == "RINEX VERSION / TYPE") {
            version = safeStod(line.substr(0, 20));
            fileType = strip(line.substr(20, 20));
            if (fileType[0] != 'N' && fileType[0] != 'n') {
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
            for (int i = 0; i < 4; i++)
                coeff.push_back(safeStod(line.substr(5 + 12 * i, 12)));
            ionoCorrData[type] = coeff;
        } else if (label == "TIME SYSTEM CORR") {
            TimeSysCorr tsc;
            tsc.A0 = safeStod(line.substr(5, 17));
            tsc.A1 = safeStod(line.substr(22, 16));
            tsc.refSOW = safeStoi(line.substr(38, 7));
            tsc.refWeek = safeStoi(line.substr(45, 5));
            timeSysCorrData[strip(line.substr(0, 4))] = tsc;
        } else if (label == "LEAP SECONDS") {
            leapSeconds = safeStoi(line.substr(0, 6));
            leapDelta   = safeStoi(line.substr(6, 6));
            leapWeek    = safeStoi(line.substr(12, 6));
            leapDay     = safeStoi(line.substr(18, 6));
        } else if (label == "END OF HEADER") {
            break;
        }
    }

    // ---- 读星历数据 ----
    while (navFileStream.peek() != EOF) {
        string line;
        getline(navFileStream, line);
        replace(line.begin(), line.end(), 'D', 'e');

        if (line.empty() || line[0] == ' ') continue;  // 空白行跳过

        if (line[0] == 'G') {
            auto eph = std::make_shared<GPSEphem>();
            loadGPSEph(*eph, line, navFileStream);
            ephTable.gps[eph->prn] = eph;
        } else if (line[0] == 'C') {
            auto eph = std::make_shared<BDSEphem>();
            loadBDSEph(*eph, line, navFileStream);
            ephTable.bds[eph->prn] = eph;
        }
        // R/GLONASS、E/Galileo —— 暂未实现
    }
}
