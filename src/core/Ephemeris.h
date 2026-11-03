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

    virtual PVT getPVT(CommonTime t);

    FrameInfo refFrame{};
    TimeSystem timeSystem{};

    virtual ~Ephemeris() = default;

    // 群时延（TGD / BGD）修正：广播钟差 a0/a1/a2 参考的是「双频无电离层组合」，
    // 只有单频用户才需要扣掉 TGD 换成单频钟。本项目 SPP/PPP 的观测模型全部走 IF 组合
    // （SPP 由 FreqCombo::detect 强制 code1+code2 双频），此时广播钟差已含群时延，
    // 再扣一次等于重复改正。实测（HKWS 2026-183 全天 30 s，GREC）：
    //   扣 TGD → 均值偏差 1.43 m、3D 散布 6.24 m；不扣 → 1.14 m、5.19 m。
    // 故基类保持空实现；将来加单频通路时在对应派生类里覆盖。
    virtual void fixTGD(PVT &pvt) {
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
    double tgd{}; // GPS 群时延（仅单频使用；见基类 fixTGD 说明，IF 模型下不扣）
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
        // 早期 BDS 单频通路的遗留（把 IF 参考钟换成 B1I 单频钟）。按基类说明，IF 模型下同样
        // 不该扣；本轮实测中开/关这一项对 SPP 结果零差别，故暂按原样保留，待确认真实 BDS
        // 样本里 tgd1 非零时的表现再决定去留。
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

    double tgd{}; // Galileo BGD：RINEX 取 BGD(E5a,E1)；NovAtel 取报文自带的（I/NAV=E5b、F/NAV=E5a）；仅单频使用
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
    double tauN{};             // 卫星钟相对 GLONASS 时的改正 τ_n (s)
    /// 频率改正 γ (s/s)，即钟漂率：clockBias(t) = -τ_n + γ·(t-tb)。
    /// 别写成 τ_d ——δτ_n(L2−L1 时延) 是另一个量，见下。
    double gamma{};
    double deltaTauN{};        // L2 与 L1 信号时延差 Δτ_n (s)，仅单频 L2 改正用，当前不使用
    double tk{};               // 信息帧时间 tk (UTC(SU) 周内秒，可选)
    int freqNum{};             // 频率通道号（FDMA 用）
    unsigned int health{};     // 健康/可用标志

    CommonTime toeCommon{};

    PVT getPVT(CommonTime t) override;
    const CommonTime &getCommonTime() const override { return toeCommon; }
};
