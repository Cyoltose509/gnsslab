#pragma once

#include <cstring>
#include <optional>
#include <vector>
#include <algorithm>
#include "Exception.h"
#include "core/Const.h"
#include <Eigen/Eigen>

namespace Math {
    template<class XType, class YType>
    YType simpleLagrangeInterpolation(const std::vector<XType> &X, const std::vector<YType> &Y, const XType &x) {
        if (X.empty() || Y.size() < X.size()) {
            throw InvalidRequest("Input vectors must be non-empty and of same size");
        }
        YType Yx = Y[0] - Y[0];
        for (std::size_t i = 0; i < X.size(); i++) {
            if (x == X[i]) return Y[i];

            XType Li(1);
            for (std::size_t j = 0; j < X.size(); j++)
                if (i != j) Li *= (x - X[j]) / (X[i] - X[j]);

            Yx += Li * Y[i];
        }
        return Yx;
    }

    template<class T>
    T lerp(const T &a, const T &b, const T &t) {
        return a + (b - a) * t;
    }

    template<class T>
    int lowerBoundIndex(const std::vector<T> &x, const T &t) {
        if (x.empty()) return 0;
        auto it = std::lower_bound(x.begin(), x.end(), t); // 第一个 >= t 的迭代器
        const int i = static_cast<int>(it - x.begin());
        return std::max(0, i - 1); // 回退一位 = 最后一个 <= t（无重复键时即包围区间左端点）
    }

    struct Bracket {
        std::size_t lo = 0; // 左样本下标
        std::size_t hi = 0; // 右样本下标
        double f = 0.0; // 插值系数 ∈ [0,1]
    };

    /// 在按时间升序排列的采样序列里定位包住 t 的相邻两项（time 取记录里的时间字段）
    template<class Rec>
    std::optional<Bracket> bracketSample(const std::vector<Rec> &v, const double t, double Rec::*time) {
        if (v.empty()) return std::nullopt;
        const std::size_t last = v.size() - 1;
        if (t <= v.front().*time) return Bracket{0, 0, 0.0};
        if (t >= v.back().*time) return Bracket{last, last, 0.0};

        const auto it = std::lower_bound(v.begin(), v.end(), t,
                                         [time](const Rec &r, const double x) {
                                             return r.*time < x;
                                         });
        const auto hi = static_cast<std::size_t>(it - v.begin());
        const std::size_t lo = hi - 1;
        const double span = v[hi].*time - v[lo].*time;
        return Bracket{lo, hi, span > 0.0 ? (t - v[lo].*time) / span : 0.0};
    }
    Eigen::Matrix3d rotationX(double t);

    Eigen::Matrix3d rotationY(double t);

    Eigen::Matrix3d rotationZ(double t);


    double angleBetween(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

    inline std::uint16_t U2(const std::uint8_t *p) {
        std::uint16_t u;
        std::memcpy(&u, p, 2);
        return u;
    }

    inline std::uint32_t U4(const std::uint8_t *p) {
        std::uint32_t u;
        std::memcpy(&u, p, 4);
        return u;
    }

    inline std::int32_t I4(const std::uint8_t *p) {
        std::int32_t i;
        std::memcpy(&i, p, 4);
        return i;
    }

    inline float R4(const std::uint8_t *p) {
        float r;
        std::memcpy(&r, p, 4);
        return r;
    }

    inline double R8(const std::uint8_t *p) {
        double r;
        std::memcpy(&r, p, 8);
        return r;
    }

    /// 把数值按小端写回字节缓冲
    inline void putU1(std::vector<std::uint8_t> &b, const std::size_t off, const std::uint32_t v) {
        b[off] = static_cast<std::uint8_t>(v & 0xFF);
    }

    inline void putU2(std::vector<std::uint8_t> &b, const std::size_t off, const std::uint32_t v) {
        const auto x = static_cast<std::uint16_t>(v & 0xFFFF);
        std::memcpy(&b[off], &x, sizeof(x));
    }

    inline void putU4(std::vector<std::uint8_t> &b, const std::size_t off, const std::uint32_t v) {
        std::memcpy(&b[off], &v, sizeof(v));
    }

    inline void putI4(std::vector<std::uint8_t> &b, const std::size_t off, const std::int32_t v) {
        std::memcpy(&b[off], &v, sizeof(v));
    }

    inline void putR4(std::vector<std::uint8_t> &b, const std::size_t off, const double v) {
        const auto x = static_cast<float>(v);
        std::memcpy(&b[off], &x, sizeof(x));
    }

    inline void putR8(std::vector<std::uint8_t> &b, const std::size_t off, const double v) {
        std::memcpy(&b[off], &v, sizeof(v));
    }

    /// 符号：<=0 取 -1，否则 +1（0 归负）。
    inline double sign(const double x) { return x <= 0.0 ? -1.0 : 1.0; }
}

