#pragma once
/**
 * StatsUtils — 纯数学统计量工具（无 GNSS / GUI 依赖）
 * --------------------------------------------------------------------------
 * 仅放最通用的统计量：均值 / 均方根 / 中位数。GNSS 领域相关的计算
 * （如三次差噪声 tripleStd）仍留在各自模块，不在此处。
 */
#include <vector>
#include <algorithm>
#include <cmath>

namespace Stats {

    /// 算术均值；空向量返回 0
    inline double mean(const std::vector<double> &x) {
        if (x.empty()) return 0.0;
        double s = 0.0;
        for (double v : x) s += v;
        return s / static_cast<double>(x.size());
    }

    /// 均方根（相对于均值，N-1 无偏），样本数 < 2 时返回 0
    inline double rms(const std::vector<double> &x) {
        if (x.size() < 2) return 0.0;
        const double m = mean(x);
        double s = 0.0;
        for (double v : x) s += (v - m) * (v - m);
        return std::sqrt(s / static_cast<double>(x.size() - 1));
    }

    /// 中位数；空向量返回 0（传值，内部排序不影响调用方）
    inline double median(std::vector<double> v) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    }

} // namespace Stats
