#pragma once
#include "GnssStruct.h"

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
        s += std::to_string(prn);
        return s;
    }
    PVT svPVT(CommonTime t);
    const ReferenceFrame *refFrame;
    explicit Ephemeris(const ReferenceFrame &frame) : refFrame(&frame) {
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