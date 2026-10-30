#pragma once

#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include "Exception.h"
#include "Const.h"   // PI（wrapPi/wrapTwoPi 用）
#include <Eigen/Eigen>
using namespace std;

namespace Math {
    template<class XType, class YType>
    YType simpleLagrangeInterpolation(const vector<XType> &X, const vector<YType> &Y, const XType &x) {
        if (X.empty() || Y.size() < X.size()) {
            throw InvalidRequest("Input vectors must be non-empty and of same size");
        }
        YType Yx = Y[0] - Y[0];
        for (size_t i = 0; i < X.size(); i++) {
            if (x == X[i]) return Y[i];

            XType Li(1);
            for (size_t j = 0; j < X.size(); j++)
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

    template<class T>
    T lagrangeInterpolation(const vector<T> &X, const vector<T> &Y, const T &x, T &err) {
        if (Y.size() < X.size() || X.size() < 4) {
            throw InvalidRequest("Input vectors must be of same length, at least 4");
        }

        size_t i;
        T y, del;
        vector<T> D, Q;

        err = T(0);
        size_t k = X.size() / 2;
        if (x == X[k]) return Y[k];
        if (x == X[k - 1]) return Y[k - 1];
        if (abs(x - X[k - 1]) < abs(x - X[k])) k = k - 1;
        for (i = 0; i < X.size(); i++) {
            Q.push_back(Y[i]);
            D.push_back(Y[i]);
        }
        y = Y[k--];
        for (size_t j = 1; j < X.size(); j++) {
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
    void lagrangeInterpolation(const vector<T> &X, const vector<T> &Y, const T &x, T &y, T &dydx) {
        if (Y.size() < X.size() || X.size() < 4) {
            throw InvalidRequest("Input vectors must be of same length, at least 4");
        }

        size_t i, k, N = X.size();
        size_t M = N * (N + 1) / 2;
        vector<T> P(N, T(1)), Q(M, T(1)), D(N, T(1));
        for (i = 0; i < N; i++) {
            for (size_t j = 0; j < N; j++) {
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
    } // end void LagrangeInterpolation(vector, vector, const T, T&, T&)


    /// Returns the second derivative of Lagrange interpolation.
    template<class T>
    T lagrangeInterpolating2ndDerivative(const vector<T> &pos, const vector<T> &val, const T desiredPos) {
        int degree(pos.size());
        int i, j;

        // First, compute interpolation factors
        typedef vector<T> vectorType;
        vector<vectorType> delta(degree, vectorType(degree, 0.0));

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
        T a(abs(aa)), b(abs(bb)), c(abs(cc));
        if (a < b) swap(a, b);
        if (a < c) swap(a, c);
        if (a == T(0)) return T(0);
        return a * sqrt(1 + b / a * (b / a) + c / a * (c / a));
    }

    /// Perform the root sum square of aa, bb
    template<class T>
    T RSS(T aa, T bb) {
        return RSS(aa, bb, T(0));
    }

    /// Perform the root sum square of aa, bb, cc and dd
    template<class T>
    T RSS(T aa, T bb, T cc, T dd) {
        T a(abs(aa)), b(abs(bb)), c(abs(cc)), d(abs(dd));
        // For numerical reason, let's just put the biggest in "a" (we are not sorting)
        if (a < b) swap(a, b);
        if (a < c) swap(a, c);
        if (a < d) swap(a, d);
        if (a == T(0)) return T(0);
        return a * sqrt(1 + b / a * (b / a) + c / a * (c / a) + d / a * (d / a));
    }

    ///多项式最小二乘拟合
    /// @param x 自变量
    /// @param y 因变量
    /// @param deg 多项式最高次幂
    vector<double> polyFit(const vector<double> &x, const vector<double> &y, int deg);

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

    /// 均方根
    double rms(const std::vector<double> &x);

    /// 中位数
    double median(std::vector<double> v);

    /// 反对称（叉乘）矩阵：skewSymmetric(v)·w == v×w
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v);

    /// 两矢量夹角 (rad)。用 atan2(|a×b|, a·b) 而非 acos(a·b/(|a||b|))：
    /// acos 在 0 附近导数无穷（dot 差 1e-16 就放大成 ~1e-8 rad 的角度误差），
    /// atan2 全区间稳健。对准残差这类"接近 0 的角度"必须用这个。
    double angleBetween(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

    inline uint16_t U2(const uint8_t *p) {
        uint16_t u;
        memcpy(&u, p, 2);
        return u;
    }

    inline uint32_t U4(const uint8_t *p) {
        uint32_t u;
        memcpy(&u, p, 4);
        return u;
    }

    inline int32_t I4(const uint8_t *p) {
        int32_t i;
        memcpy(&i, p, 4);
        return i;
    }

    inline float R4(const uint8_t *p) {
        float r;
        memcpy(&r, p, 4);
        return r;
    }

    inline double R8(const uint8_t *p) {
        double r;
        memcpy(&r, p, 8);
        return r;
    }

    /// 归化到 (-pi, pi]
    inline double wrapPi(const double a) {
        double x = std::fmod(a + PI, 2.0 * PI);
        if (x < 0.0) x += 2.0 * PI;
        return x - PI;
    }

    /// 归化到 [0, 2pi)
    inline double wrapTwoPi(const double a) {
        double x = std::fmod(a, 2.0 * PI);
        if (x < 0.0) x += 2.0 * PI;
        return x;
    }
}

inline double sign(const double x) { return x <= 0.0 ? -1.0 : 1.0; }
