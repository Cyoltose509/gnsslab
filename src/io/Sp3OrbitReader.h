#pragma once
#include "GnssStruct.h"
#include <vector>
#include <map>
#include <string>

/// 精密轨道 SP3 读取器（支持 SP3c 的 P/V 记录）。
class Sp3OrbitReader {
public:
    struct Record {
        CommonTime t;
        Eigen::Vector3d pos; // 单位 m
        Eigen::Vector3d vel; // 单位 m/s
        double clock = 0.0; // us
        bool hasVel = false;
    };

    /// 读取 SP3 文件。成功返回 true。
    bool read(const std::string &path);

    [[nodiscard]] bool empty() const { return data.empty(); }

    /// 卫星是否可用
    [[nodiscard]] bool contains(const SatID &sat) const {
        const auto it = data.find(sat);
        return it != data.end() && !it->second.empty();
    }

    /// 在某时刻 t 插值出卫星 PVT。
    [[nodiscard]] PVT getPVT(const SatID &sat, const CommonTime &t) const;

    /// 数据时间跨度（用于诊断）
    [[nodiscard]] std::pair<CommonTime, CommonTime> timeSpan(const SatID &sat) const;

    [[nodiscard]] int satCount() const { return static_cast<int>(data.size()); }

    /// 只读访问内部卫星→记录表（自动探测参考卫星等场景用）。
    [[nodiscard]] const std::map<SatID, std::vector<Record>> &getData() const { return data; }

private:
    std::map<SatID, std::vector<Record> > data;
    // 每卫星相对首历元的秒数向量(读入时一次性预计算)，避免 getPVT 每次调用都 O(n) 拷贝全部轨道历元时间。
    std::map<SatID, std::vector<double> > relTimes;
};
