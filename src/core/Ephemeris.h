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
}; // end class pvt

inline std::ostream &operator<<(std::ostream &os, PVT &pvt)
    noexcept {
    os << setprecision(10) << "p:" << pvt.p.transpose() << endl;
    os << "v:" << pvt.v.transpose() << endl;
    os << "clk bias:" << pvt.clockBias << endl;
    os << "clk drift:" << pvt.clockDrift << endl;
    os << "relcorr:" << pvt.relativityCorrection << endl;
    return os;
}

// 基类：包含所有卫星通用的轨道参数
struct Ephemeris {
    // --- 通用轨道参数 (所有系统都有) ---
    double toe{}; // 星历参考时刻
    double m0{}; // 平近点角
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

// GPS 星历：特有 IODE, TGD, AS 等
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

// Galileo 星历：RINEX 3.04 布局与 GPS 完全同构（同开普勒六根数 + 调和项），
// 直接复用基类 Ephemeris::getPVT()。仅时间系统(GAL)与参考框架(GTRF≈GPS)不同。
// IF 组合(L1+L5/E1+E5a)下 TGD 被吸收进接收机钟差，SPP 级可不单独修正。
struct GalileoEphem : Ephemeris {
    GalileoEphem() {
        type = 'E';
        timeSystem = TimeSystem::GAL;
        refFrame = Frame::GPS;
    }

    double tgd{}; // E5a 群时延 (TGDE5a, RINEX 第6行第3列)
};

// GLONASS 广播星历：RINEX 3.04 四行格式（epoch + 3 数据行），PZ-90 ECEF 框架。
// 与 GPS/GAL/BDS 的开普勒传播不同，GLONASS 用"位置/速度/加速度数值积分"
// 求得发射时刻卫星状态（RINEX 直接给出 tk 时刻的 X/Y/Z、Xdot/Ydot/Zdot、Xddot/Yddot/Zddot）。
// GLONASS 广播星历时间取 GPS 时间系统：RINEX 里的 GLONASS 历元按 UTC(SU) 写，但常用 BRDC
// 合并文件已把历元写成 UTC 墙钟（与 GPS 观测历元同一日历），故与 GPS/GAL/BDS 一样，把 toe 解析成
// GPS 标签的日历 SOD，getPVT 内用 convertTimeSystem(t, timeSystem) 对齐即可（GPS→GPS 为 no-op，
// dt 即为两日历之差，物理秒数正确，等价于 RTKLIB 的 utc2gpst 后做 timediff）。
// GLONASS 钟差模型 τ = -τ_n + τ_d·(t - t_b)，与 RTKLIB geph2pos 一致。
struct GlonassEphem : Ephemeris {
    GlonassEphem() {
        type = 'R';
        // GLONASS 广播历元是 GLONASS 时间 = UTC(SU)（不施加跳秒，与 UTC 同走时），
        // 与 GPS 相差 GPS-UTC 跳秒（2026 年为 18 s）。按原生时间系统解析，
        // getPVT 内 convertTimeSystem(t, timeSystem) 负责把 GPS 观测时刻对齐到 UTC。
        timeSystem = TimeSystem::UTC;
        refFrame = Frame::PZ90;
    }

    // tk 时刻的卫星状态（单位：km, km/s, km/s^2），直接来自 RINEX 三行数据
    double X{}, Y{}, Z{};
    double vX{}, vY{}, vZ{};
    double aX{}, aY{}, aZ{};   // 日月引力加速度（luni-solar perturbing acceleration）
    double tauN{};             // SV 钟差 τ_n (s)
    double tauD{};             // 相对频偏 τ_d (无量纲)
    double tk{};               // 信息帧时间 tk (UTC(SU) 周内秒，可选)
    int freqNum{};             // 频率通道号（FDMA 用）
    unsigned int health{};     // 健康/可用标志

    // 参考历元（绝对时刻，UTC 标记），供 EphemerisTable::find 计算星历年龄
    CommonTime toeCommon{};

    PVT getPVT(CommonTime t) override;
    const CommonTime &getCommonTime() override { return toeCommon; }
};
