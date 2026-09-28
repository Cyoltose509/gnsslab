#pragma once

#include <cmath>  // std::sin, std::max

// 高度角随机模型（观测方差形式）：σ²(elev) = a² + (b / sinE)²
// 高仰角方差趋于 a²，低仰角 sinE→0 时方差放大 ⇒ 权衰减，符合 GNSS 经验模型。
// 单一映射源：SPP/PPP 的观测定权与 RTK 的站间单差方差都从这里取，
// 不再允许 int↔string 双份编码或散落的裸公式。
class Weight {
public:
    double a = 0.004;          // 高仰角方差项（噪声底 / 多路径）
    double b = 0.003;          // 低仰角放大系数
    double minSinElev = 1e-3;  // sin(截止角)下限，避免低仰角除零 / 方差爆炸

    Weight() = default;
    Weight(double a_, double b_, double minSinElev_ = 1e-3)
        : a(a_), b(b_), minSinElev(minSinElev_) {}

    // 单颗星在给定高度角(rad)下的观测方差 σ²
    double variance(double elev) const {
        const double s = (std::max)(std::sin(elev), minSinElev);
        return a * a + b * b / (s * s);
    }

    // 对应的权 w = 1 / σ²
    double weight(double elev) const {
        return 1.0 / variance(elev);
    }
};
