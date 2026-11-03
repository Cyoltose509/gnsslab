#pragma once

#include <cmath>
#include <cstring>
#include <optional>
#include <vector>
#include <algorithm>
#include "Exception.h"
#include "Const.h"
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

    /// 在按时间升序排列的采样序列里定位包住 t 的相邻两项
    template<class Rec, class TimeOf>
    std::optional<Bracket> bracketSample(const std::vector<Rec> &v, const double t, TimeOf timeOf) {
        if (v.empty()) return std::nullopt;
        const std::size_t last = v.size() - 1;
        if (t <= timeOf(v.front())) return Bracket{0, 0, 0.0};
        if (t >= timeOf(v.back())) return Bracket{last, last, 0.0};

        const auto it = std::lower_bound(v.begin(), v.end(), t,
                                         [&timeOf](const Rec &r, const double x) {
                                             return timeOf(r) < x;
                                         });
        const auto hi = static_cast<std::size_t>(it - v.begin());
        const std::size_t lo = hi - 1;
        const double span = timeOf(v[hi]) - timeOf(v[lo]);
        return Bracket{lo, hi, span > 0.0 ? (t - timeOf(v[lo])) / span : 0.0};
    }

    template<class T>
    T lagrangeInterpolation(const std::vector<T> &X, const std::vector<T> &Y, const T &x, T &err) {
        if (Y.size() < X.size() || X.size() < 4) {
            throw InvalidRequest("Input vectors must be of same length, at least 4");
        }

        std::size_t i;
        T y, del;
        std::vector<T> D, Q;

        err = T(0);
        std::size_t k = X.size() / 2;
        if (x == X[k]) return Y[k];
        if (x == X[k - 1]) return Y[k - 1];
        if (std::abs(x - X[k - 1]) < std::abs(x - X[k])) k = k - 1;
        for (i = 0; i < X.size(); i++) {
            Q.push_back(Y[i]);
            D.push_back(Y[i]);
        }
        y = Y[k--];
        for (std::size_t j = 1; j < X.size(); j++) {
            for (i = 0; i < X.size() - j; i++) {
                del = (Q[i + 1] - D[i]) / (X[i] - X[i + j]);
                D[i] = (X[i + j] - x) * del;
                Q[i] = (X[i] - x) * del;
            }
            err = 2 * (k + 1) < X.size() - j ? Q[k + 1] : D[k--]; // NOT 2*k
            y += err;
        }
        return y;
    }

    template<class T>
    void lagrangeInterpolation(const std::vector<T> &X, const std::vector<T> &Y, const T &x, T &y, T &dydx) {
        if (Y.size() < X.size() || X.size() < 4) {
            throw InvalidRequest("Input vectors must be of same length, at least 4");
        }

        std::size_t i, k, N = X.size();
        std::size_t M = N * (N + 1) / 2;
        std::vector<T> P(N, T(1)), Q(M, T(1)), D(N, T(1));
        for (i = 0; i < N; i++) {
            for (std::size_t j = 0; j < N; j++) {
                if (i != j) {
                    P[i] *= x - X[j];
                    D[i] *= X[i] - X[j];
                    if (i < j) {
                        for (k = 0; k < N; k++) {
                            if (k == i || k == j) continue;
                            Q[i + j * (j + 1) / 2] *= x - X[k];
                        }
                    }
                }
            }
        }
        y = dydx = T(0);
        for (i = 0; i < N; i++) {
            y += Y[i] * (P[i] / D[i]);
            T S(0);
            for (k = 0; k < N; k++)
                if (i != k) {
                    if (k < i) S += Q[k + i * (i + 1) / 2] / D[i];
                    else S += Q[i + k * (k + 1) / 2] / D[i];
                }
            dydx += Y[i] * S;
        }
    }


    /// Returns the second derivative of Lagrange interpolation.
    template<class T>
    T lagrangeInterpolating2ndDerivative(const std::vector<T> &pos, const std::vector<T> &val, const T desiredPos) {
        int degree(pos.size());
        int i, j;

        // First, compute interpolation factors
        typedef std::vector<T> vectorType;
        std::vector<vectorType> delta(degree, vectorType(degree, 0.0));

        for (i = 0; i < degree; ++i) {
            for (j = 0; j < degree; ++j) {
                if (j != i) {
                    delta[i][j] = (desiredPos - pos[j]) / (pos[i] - pos[j]);
                }
            }
        }

        double retVal(0.0);
        for (i = 0; i < degree; ++i) {
            double sum(0.0);

            for (int m = 0; m < degree; ++m) {
                if (m != i) {
                    const double weight1(1.0 / (pos[i] - pos[m]));
                    double sum2(0.0);

                    for (j = 0; j < degree; ++j) {
                        if (j != i && j != m) {
                            double weight2(1.0 / (pos[i] - pos[j]));
                            for (int n = 0; n < degree; ++n) {
                                if (n != j && n != m && n != i) {
                                    weight2 *= delta[i][n];
                                }
                            }
                            sum2 += weight2;
                        }
                    }
                    sum += sum2 * weight1;
                }
            }
            retVal += val[i] * sum;
        }

        return retVal;
    }

    /// Perform the root sum square of aa, bb and cc
    template<class T>
    T RSS(T aa, T bb, T cc) {
        T a(std::abs(aa)), b(std::abs(bb)), c(std::abs(cc));
        if (a < b) std::swap(a, b);
        if (a < c) std::swap(a, c);
        if (a == T(0)) return T(0);
        return a * std::sqrt(1 + b / a * (b / a) + c / a * (c / a));
    }

    /// Perform the root sum square of aa, bb
    template<class T>
    T RSS(T aa, T bb) {
        return RSS(aa, bb, T(0));
    }

    /// Perform the root sum square of aa, bb, cc and dd
    template<class T>
    T RSS(T aa, T bb, T cc, T dd) {
        T a(std::abs(aa)), b(std::abs(bb)), c(std::abs(cc)), d(std::abs(dd));
        // For numerical reason, let's just put the biggest in "a" (we are not sorting)
        if (a < b) std::swap(a, b);
        if (a < c) std::swap(a, c);
        if (a < d) std::swap(a, d);
        if (a == T(0)) return T(0);
        return a * std::sqrt(1 + b / a * (b / a) + c / a * (c / a) + d / a * (d / a));
    }

    ///多项式最小二乘拟合
    /// @param x 自变量
    /// @param y 因变量
    /// @param deg 多项式最高次幂
    std::vector<double> polyFit(const std::vector<double> &x, const std::vector<double> &y, int deg);

    ///多项式求值
    /// @param c 多项式系数
    /// @param x 自变量
    /// @return 多项式值
    double polyVal(const std::vector<double> &c, double x);

    Eigen::MatrixXd rotationMatrix(double angle, int axis);

    Eigen::Matrix3d rotationX(double t);

    Eigen::Matrix3d rotationY(double t);

    Eigen::Matrix3d rotationZ(double t);

    double frobenius(const Eigen::MatrixXd &m);

    /// returns the magnitude of the vector
    double magnitude(const Eigen::VectorXd &v);

    /// 算术均值
    double mean(const std::vector<double> &x);

    /// 样本标准差（贝塞尔校正，除以 n-1）。
    double stdDev(const std::vector<double> &x);

    /// 中位数（偶数个取中间两个的均值）
    double median(std::vector<double> v);

    /// 反对称（叉乘）矩阵：skewSymmetric(v)·w == v×w
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v);

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

    inline double wrapTwoPi(const double a) {
        double x = std::fmod(a, 2.0 * PI);
        if (x < 0.0) x += 2.0 * PI;
        return x;
    }

    inline double wrapPi(const double a) {
        return wrapTwoPi(a + PI) - PI;
    }
}

inline double sign(const double x) { return x <= 0.0 ? -1.0 : 1.0; }
