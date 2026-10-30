#pragma once
#include "TimeConvert.h"
#include "CoordStruct.h"


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
    os << setprecision(10) << "p:" << pvt.p.transpose() << endl;
    os << "v:" << pvt.v.transpose() << endl;
    os << "clk bias:" << pvt.clockBias << endl;
    os << "clk drift:" << pvt.clockDrift << endl;
    os << "relcorr:" << pvt.relativityCorrection << endl;
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


    const WeekSecond &getWeekSecond() {
        ws.week = week;
        ws.sow = toe;
        ws.timeSystem = timeSystem;
        return ws;
    }

    virtual const CommonTime &getCommonTime() {
        WeekSecond2CommonTime(getWeekSecond(), ct);
        return ct;
    }

    [[nodiscard]] string name() const {
        string s;
        s.reserve(4);
        s += type;
        s += std::to_string(prn);
        return s;
    }

    virtual PVT getPVT(CommonTime t);

    FrameInfo refFrame{};
    TimeSystem timeSystem{};

    virtual ~Ephemeris() = default;

    virtual void fixTGD(PVT &pvt) {
    }

private:
    WeekSecond ws;
    CommonTime ct;
};

inline ostream &operator<<(ostream &os, const Ephemeris &eph) {
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
    double tgd1{}; // B1/B1C 群时延
    double tgd2{}; // B2/B2a 群时延
    void fixTGD(PVT &pvt) override {
        constexpr double f1 = getFreq('C', 2);
        constexpr double f2 = getFreq('C', 6);
        constexpr double alpha = f1 * f1 / (f1 * f1 - f2 * f2);
        pvt.clockBias -= alpha * tgd1;
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
    double aX{}, aY{}, aZ{};   // 日月引力加速度
    double tauN{};             // SV 钟差 τ_n (s)
    double tauD{};             // 相对频偏 τ_d (无量纲)
    double tk{};               // 信息帧时间 tk (UTC(SU) 周内秒，可选)
    int freqNum{};             // 频率通道号（FDMA 用）
    unsigned int health{};     // 健康/可用标志

    CommonTime toeCommon{};

    PVT getPVT(CommonTime t) override;
    const CommonTime &getCommonTime() override { return toeCommon; }
};
