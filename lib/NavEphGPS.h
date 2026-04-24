#pragma once

#include "GnssStruct.h"

class NavEphGPS {
public:
    /// Default constructor
    NavEphGPS()
        : Toc(0), af0(0), af1(0), af2(0), IODE(0), Crs(0), Delta_n(0), M0(0), Cuc(0), ecc(0), Cus(0), sqrt_A(0), Toe(0), Cic(0), OMEGA_0(0),
          Cis(0),
          i0(0),
          Crc(0),
          omega(0),
          OMEGA_DOT(0),
          IDOT(0),
          L2Codes(0),
          GPSWeek(0),
          L2Pflag(0),
          URA(0),
          SV_health(0),
          TGD(0), IODC(0),
          HOWtime(0),
          fitInterval(0),
          beginValid(END_OF_TIME),
          endValid(BEGINNING_OF_TIME) {
        beginValid.m_timeSystem = TimeSystem::GPS;
        endValid.m_timeSystem = TimeSystem::GPS;
    }

    /// Dump the overhead information to the given output stream.
    /// throw Invalid Request if the required data has not been stored.
    void printData() const;

    /// Compute the satellite clock bias (seconds) at the given time
    /// throw Invalid Request if the required data has not been stored.
    double svClockBias(const CommonTime &t) const;

    /// Compute the satellite clock drift (sec/sec) at the given time
    /// throw Invalid Request if the required data has not been stored.
    double svClockDrift(const CommonTime &t) const;

    /// Compute satellite relativity correction (sec) at the given time
    /// throw Invalid Request if the required data has not been stored.
    double svRelativity(const CommonTime &t) const;

    /// return URA of broadcast
    double svURA(const CommonTime &t) const;

    /// Compute satellite position at the given time.
    PVT svPVT(const CommonTime &t) const;

    bool isValid(const CommonTime &ct) const;

    ///Ephemeris data
    ///   SV/EPOCH/SV CLK
    CivilTime CivilToc;
    double Toc; ///< Time of clock (year/month/day/hour/min/sec GPS)
    double af0; ///< SV clock bias(seconds)
    double af1; ///< SV clock drift(sec/sec)
    double af2; ///< SV clock drift rate (sec/sec2)

    ///   BROADCAST ORBIT-1
    double IODE; ///< IODE Issue of Data, Ephemeris
    double Crs; ///< (meters)
    double Delta_n; ///< Mean Motion Difference From Computed Value(semicircles/sec)
    double M0; ///< Mean Anomaly at Reference Time(semicircles)

    ///   BROADCAST ORBIT-2
    double Cuc; ///< (radians)
    double ecc; ///< Eccentricity
    double Cus; ///< (radians)
    double sqrt_A; ///< Square Root of the Semi-Major Axis(sqrt(m))

    ///   BROADCAST ORBIT-3
    double Toe; ///< Time of Ephemeris(sec of GPS week)
    double Cic; ///< (radians)
    double OMEGA_0; ///< Longitude of Ascending Node of Orbit Plane(semicircles)
    double Cis; ///< (radians)

    ///   BROADCAST ORBIT-4
    double i0; ///< Inclination Angle at Reference Time(semi-circles)
    double Crc; ///< (meters)
    double omega; ///< Argument of Perigee(semi-circles)
    double OMEGA_DOT; ///< Rate of Right Ascension(semicircles/sec)

    ///   BROADCAST ORBIT-5
    double IDOT; ///< Rate of Inclination Angle(semicircles/sec)
    double L2Codes;
    double GPSWeek; ///< to go with TOE, Continuous number,
    ///not mod 1024
    double L2Pflag;

    ///   BROADCAST ORBIT-6
    double URA; ///< SV accuracy(meters) See GPS ICD
    double SV_health; ///< bits 17-22 w 3 sf 1
    double TGD; ///< (seconds)
    double IODC; ///< Issue of Data, Clock

    ///   BROADCAST ORBIT-7
    long HOWtime; ///< Transmission time of message， sec of GPSWeek
    double fitInterval; ///< Fit Interval in hours

    ///member data
    CommonTime ctToc; ///< Toc in CommonTime form
    CommonTime ctToe; ///< Toe in CommonTime form
    CommonTime transmitTime; ///< Transmission time in CommonTime form
    CommonTime beginValid; ///< Time at beginning of validity
    CommonTime endValid; ///< Time at end of fit validity
}; // end class NavEphGPS


// 基类：包含所有卫星通用的轨道参数
struct Ephemeris {
    // --- 通用轨道参数 (所有系统都有) ---
    double toe{}; // 星历参考时刻
    double M0{}; // 平近点角
    double e{}; // 偏心率
    double RootA{}; // 或者是 A (半长轴)，建议统一存 sqrt(A)，计算时更方便
    double dn{}; // 平均运动角速度的修正值
    double i0{}; // 轨道倾角
    double Omega0{}; // 升交点赤经
    double omega{}; // 近地点角距
    double Omegadot{}; // 升交点赤经变化率
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
    unsigned int PRN{}; // 卫星编号
    unsigned int health{}; // 健康状态
    double URA{}; // 用户测距精度

    unsigned int week{};
    char type{};
    TimeSystem timeSystem{};
    const WeekSecond &getWeekSecond() {
        ws.week = week;
        ws.sow = toe;
        ws.timeSystem = timeSystem;
        return ws;
    }
    const CommonTime &getCommonTime() {
        WeekSecond2CommonTime(getWeekSecond(), ct);
        return ct;
    }
    string name() const {
        string s;
        s.reserve(4);
        s += type;
        s += std::to_string(PRN);
        return s;
    }
    PVT svPVT(CommonTime t);
    const ReferenceFrame &refFrame;
    explicit Ephemeris(const ReferenceFrame &frame) : refFrame(frame) {
    }
    virtual ~Ephemeris() = default;

private:
    WeekSecond ws;
    CommonTime ct;
};

// GPS 星历：特有 IODE, TGD, AS 等
struct GPSEphem : Ephemeris {
    GPSEphem() : Ephemeris(Frame::GPS) {
        type = 'G';
        timeSystem = TimeSystem::GPS;
    }

    unsigned int IODE{};
    unsigned int IODC{};
    double tgd{}; // GPS 群时延
};

struct BDSEphem : Ephemeris {
    BDSEphem() : Ephemeris(Frame::WGS84) {
        type = 'C';
        timeSystem = TimeSystem::BDT;
    }

    unsigned int AODE{};
    unsigned int AODC{};
    double tgd1{}; // B1/B1C 群时延
    double tgd2{}; // B2/B2a 群时延
};
