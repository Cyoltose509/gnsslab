#pragma once
#include "TimeConvert.h"
#include "CoordStruct.h"
#include "Const.h"
#include "FreqCombo.h"


/// ECEF position, velocity, clock bias and drift
class PVT {
public:
    /// Default constructor
    PVT() : p(0., 0., 0.), v(0., 0., 0.),
            clockBias(0.), clockDrift(0.) {
    }

    // member data

    Eigen::Vector3d p; ///< Sat position ECEF Cartesian (X,Y,Z) meters
    Eigen::Vector3d v; ///< satellite velocity in ECEF Cartesian, meters/second
    double clockBias; ///< Sat clock correction in seconds
    double clockDrift; ///< satellite clock drift in seconds/second
    double relativityCorrection{};

    PVT(const PVT &) = default;

    PVT &operator=(const PVT &) = default;

    PVT(PVT &&other) noexcept
        : p(std::move(other.p)),
          v(std::move(other.v)),
          clockBias(other.clockBias),
          clockDrift(other.clockDrift),
          relativityCorrection(other.relativityCorrection) {
    }

    PVT &operator=(PVT &&other) noexcept {
        if (this != &other) {
            p = std::move(other.p);
            v = std::move(other.v);
            clockBias = other.clockBias;
            clockDrift = other.clockDrift;
            relativityCorrection = other.relativityCorrection;
        }
        return *this;
    }
};

inline std::ostream &operator<<(std::ostream &os, PVT &pvt)
    noexcept {
    os << std::setprecision(10) << "p:" << pvt.p.transpose() << std::endl;
    os << "v:" << pvt.v.transpose() << std::endl;
    os << "clk bias:" << pvt.clockBias << std::endl;
    os << "clk drift:" << pvt.clockDrift << std::endl;
    os << "relcorr:" << pvt.relativityCorrection << std::endl;
    return os;
}

inline double satRelativityCorrection(const Eigen::Vector3d &p, const Eigen::Vector3d &v) {
    return -2.0 * p.dot(v) / (C_MPS * C_MPS);
}


struct Ephemeris {

    double toe{};
    double m0{};
    double e{}; // 偏心率
    double rootA{}; // 或者是 A (半长轴)，建议统一存 sqrt(A)，计算时更方便
    double dn{}; // 平均运动角速度的修正值
    double i0{}; // 轨道倾角
    double omega0{}; // 升交点赤经
    double omega{}; // 近地点角距
    double omegaDot{}; // 升交点赤经变化率
    double idot{}; // 轨道倾角变化率

    // 调和项摄动参数 (所有系统都有)
    double cuc{};
    double cus{};
    double crc{};
    double crs{};
    double cic{};
    double cis{};

    // --- 通用时钟参数 (所有系统都有) ---
    double toc{}; // 时钟参考时刻
    double a0{}; // 钟差
    double a1{}; // 钟漂
    double a2{}; // 钟漂率

    // --- 通用状态信息 ---
    int prn{}; // 卫星编号
    unsigned int health{}; // 健康状态
    double ura{}; // 用户测距精度

    unsigned int week{};
    char type{};


    const WeekSecond &getWeekSecond() const {
        ws.week = week;
        ws.sow = toe;
        ws.timeSystem = timeSystem;
        return ws;
    }

    virtual const CommonTime &getCommonTime() const {
        ct = Time::WeekSecondToCommon(getWeekSecond());
        return ct;
    }

    [[nodiscard]] std::string name() const {
        std::string s;
        s.reserve(4);
        s += type;
        s += std::to_string(prn);
        return s;
    }

    virtual PVT getPVT(CommonTime t, const FreqCombo *combo);

    FrameInfo refFrame{};
    TimeSystem timeSystem{};

    virtual ~Ephemeris() = default;

    virtual void fixTGD(PVT &pvt, const FreqCombo *combo) {
        (void) pvt;
        (void) combo;
    }

private:
    mutable WeekSecond ws;
    mutable CommonTime ct;
};

inline std::ostream &operator<<(std::ostream &os, const Ephemeris &eph) {
    os << "prn:" << eph.prn
            << " type:" << eph.type
            << " week:" << eph.week
            << " toe:" << eph.toe
            << " toc:" << eph.toc
            << " health:" << eph.health
            << " ura:" << eph.ura
            << " a0:" << eph.a0
            << " a1:" << eph.a1
            << " a2:" << eph.a2
            << " m0:" << eph.m0
            << " e:" << eph.e
            << " rootA:" << eph.rootA
            << " dn:" << eph.dn
            << " i0:" << eph.i0
            << " omega0:" << eph.omega0
            << " omega:" << eph.omega
            << " omegaDot:" << eph.omegaDot
            << " idot:" << eph.idot
            << " cuc:" << eph.cuc
            << " cus:" << eph.cus
            << " crc:" << eph.crc
            << " crs:" << eph.crs
            << " cic:" << eph.cic
            << " cis:" << eph.cis;
    return os;
}

struct GPSEphem : Ephemeris {
    GPSEphem() {
        type = 'G';
        timeSystem = TimeSystem::GPS;
        refFrame = Frame::GPS;
    }

    unsigned int IODE{};
    unsigned int IODC{};
    double tgd{}; // GPS 群时延
};

struct BDSEphem : Ephemeris {
    BDSEphem() {
        type = 'C';
        timeSystem = TimeSystem::BDT;
        refFrame = Frame::CGCS2000;
    }

    unsigned int AODE{};
    unsigned int AODC{};
    double tgd1{}; // B1I 相对 B3I 的群时延
    double tgd2{}; // B2I/B2b 相对 B3I 的群时延

    /// 北斗广播钟差以 B3I 单频为参考（ICD 5.2.4.12）
    void fixTGD(PVT &pvt, const FreqCombo *combo) override {
        const double f1 = combo && combo->f1 > 0.0 ? combo->f1 : L2_FREQ_BDS; // 默认 B1I
        const double f2 = combo && combo->f2 > 0.0 ? combo->f2 : L6_FREQ_BDS; // 默认 B3I
        const double s1 = f1 * f1, s2 = f2 * f2;
        if (s1 == s2) return;
        pvt.clockBias -= (s1 * tgdOf(f1) - s2 * tgdOf(f2)) / (s1 - s2);
    }

    /// 频点相对 B3I 的群时延。B1C/B2a 的独立群时延（TGD_B1Cp/TGD_B2ap）只由 B-CNAV
    /// 播发、只出现在 RINEX 4 的 CNV 记录里，D1 记录取不到，这里按邻近频点近似：
    /// B1C 用 TGD1（与真值差 ±4 ns 量级）；B2a 直接取 0（真值约 −10 ns，比错用 B1 的 TGD 好）；
    /// B2I/B2b 用 TGD2，但 BDS-3 的 TGD2 是 TGD1 的副本、不含 B2 信息，故 C19 起同样取 0。
    [[nodiscard]] double tgdOf(const double f) const {
        constexpr double kBand = 1.0e6;
        constexpr int kBds3FirstPrn = 19;
        if (std::abs(f - L6_FREQ_BDS) < kBand) return 0.0;
        if (std::abs(f - L2_FREQ_BDS) < kBand || std::abs(f - L1_FREQ_BDS) < kBand) return tgd1;
        if (std::abs(f - L5_FREQ_BDS) < kBand) return 0.0;
        if (std::abs(f - L7_FREQ_BDS) < kBand) return prn >= kBds3FirstPrn ? 0.0 : tgd2;
        return 0.0;
    }
};

struct GalileoEphem : Ephemeris {
    GalileoEphem() {
        type = 'E';
        timeSystem = TimeSystem::GAL;
        refFrame = Frame::GPS;
    }

    double tgd{};
};

struct GlonassEphem : Ephemeris {
    GlonassEphem() {
        type = 'R';
        timeSystem = TimeSystem::UTC;
        refFrame = Frame::PZ90;
    }

    double X{}, Y{}, Z{};
    double vX{}, vY{}, vZ{};
    double aX{}, aY{}, aZ{};
    double tauN{};
    double gamma{};
    double deltaTauN{};
    double tk{};
    int freqNum{};
    unsigned int health{};

    CommonTime toeCommon{};

    PVT getPVT(CommonTime t, const FreqCombo *combo) override;
    const CommonTime &getCommonTime() const override { return toeCommon; }
};
