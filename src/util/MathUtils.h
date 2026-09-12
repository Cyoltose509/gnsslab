#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include "Exception.h"
#include <Eigen/Eigen>
using namespace std;

namespace Math {
    /// This is a straightforward version of Lagrange Interpolation.
    /// Y must have size at least as large as X, and X.size() must be >= 2;
    /// x should lie within the range of X.
    /// 节点横坐标 X(如时间) 与纵坐标 Y(如矢量) 类型可不同：YType 需支持 YType(0) 与 (double * YType) 的累加。
    /// 例：X=vector<double>(时间), Y=vector<Eigen::Vector3d>(位置) → 直接插值矢量。
    template<class XType, class YType>
    YType simpleLagrangeInterpolation(const vector<XType> &X, const vector<YType> &Y, const XType &x) {
        if (X.empty() || Y.size() < X.size()) {
            throw InvalidRequest("Input vectors must be non-empty and of same size");
        }
        // 对 Eigen 固定大小向量，YType(0) / Ytype(0.0) 都会匹配到 Matrix(Index rows) 构造函数，
        // 触发 resize(0) 导致断言崩溃；用 Y[0]-Y[0] 得到类型正确的零，对 double/Eigen 均安全。
        YType Yx = Y[0] - Y[0];
        for (size_t i = 0; i < X.size(); i++) {
            if (x == X[i]) return Y[i];

            XType Li(1);
            for (size_t j = 0; j < X.size(); j++)
                if (i != j) Li *= (x - X[j]) / (X[i] - X[j]);

            Yx += Li * Y[i];
        }
        return Yx;
    } // end YType SimpleLagrangeInterpolation(const vector<XType>, const vector<YType>, const XType)

    template<class T>
    T lerp(const T &a, const T &b, const T &t) {
        return a + (b - a) * t;
    }

    /// 升序序列二分查找：返回最大的 i 使 x[i] <= t（lower-bound 索引），越界 clamp 到 [0, n-1]。
    /// 底层委托 C++ 标准库 std::lower_bound（要求 x 升序）；本函数仅做「取 int 索引 + 边界兜底」的薄封装，
    /// 调用方据结果 i 与 i+1 取「包围区间」或「最近点」（如 SP3/CLK 历元插值）。
    template<class T>
    int lowerBoundIndex(const std::vector<T> &x, const T &t) {
        if (x.empty()) return 0;
        auto it = std::lower_bound(x.begin(), x.end(), t); // 第一个 >= t 的迭代器
        const int i = static_cast<int>(it - x.begin());
        return std::max(0, i - 1); // 回退一位 = 最后一个 <= t（无重复键时即包围区间左端点）
    }

    /// Lagrange interpolation on data (X[i],Y[i]), i=0,N-1 to compute Y(x).
    /// Also return an estimate of the estimation error in 'err'.
    /// This routine assumes that N=X.size() is even and that x is centered on the
    /// interval, that is X[N/2-1] <= x <= X[N/2].
    /// NB This routine will work for N as small as 4, however tests with satellite
    /// ephemerides have shown that N=4 yields m-level errors, N=6 cm-level,
    /// N=8 ~0.1mm level and N=10 ~numerical noise errors; best to use N>=8.
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
    } // end T LagrangeInterpolation(vector, vector, const T, T&)


    /// Perform Lagrange interpolation on the data (X[i],Y[i]), i=1,N (N=X.size()),
    /// returning the value of Y(x) and dY(x)/dX.
    /// Assumes that x is between X[k-1] and X[k], where k=N/2 and N > 2;
    /// Warning: for use with the precise (SP3) ephemeris only when velocity is not
    /// available; estimates of velocity, and especially clock drift, not as accurate.
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

    ///多项式最小二乘拟合
    /// @param x 自变量
    /// @param y 因变量
    /// @param deg 多项式最高次幂
    inline vector<double> polyFit(const vector<double> &x, const vector<double> &y, const int deg) {
        const int n = static_cast<int>(x.size());
        Eigen::MatrixXd A(n, deg + 1);
        Eigen::VectorXd Y(n);
        for (int i = 0; i < n; i++) {
            double p = 1;
            for (int j = 0; j <= deg; j++) {
                A(i, j) = p;
                p *= x[i];
            }
            Y(i) = y[i];
        }
        Eigen::VectorXd c = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
        std::vector<double> r(deg + 1);
        for (int j = 0; j <= deg; j++) r[j] = c(j);
        return r;
    }

    ///多项式求值
    /// @param c 多项式系数
    /// @param x 自变量
    /// @return 多项式值
    inline double polyVal(const std::vector<double> &c, const double x) {
        double r = 0, p = 1;
        for (const double cc: c) {
            r += cc * p;
            p *= x;
        }
        return r;
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


    inline Eigen::MatrixXd rotationMatrix(const double angle, const int axis) {
        if (axis < 1 || axis > 3) {
            throw InvalidRequest("Invalid axis (must be 1,2, or 3)");
        }

        Eigen::MatrixXd toReturn(3, 3);
        toReturn.setZero();

        const int i1 = axis - 1;
        const int i2 = (i1 + 1) % 3;
        const int i3 = (i2 + 1) % 3;

        toReturn(i1, i1) = 1.0;
        toReturn(i2, i2) = toReturn(i3, i3) = ::cos(angle);
        toReturn(i3, i2) = -(toReturn(i2, i3) = ::sin(angle));
        return toReturn;
    }

    // 绕 X/Y/Z 轴的 3x3 旋转矩阵（固定大小，避免上面的动态 MatrixXd）。
    // 符号约定与 RTKLIB rtkcmn.c 的 Rx/Ry/Rz 宏一致：RTKLIB 用列主序数组，
    // 其 Rx/Ry/Rz(t) 等效于标准旋转 Rx(-t)/Ry(-t)/Rz(-t)（即三个轴均为负向旋转）。
    // 此处用 AngleAxisd 显式写出该等效旋转，便于直接阅读而不必再想列主序。
    inline Eigen::Matrix3d rotationX(double t) {
        return Eigen::AngleAxisd(-t, Eigen::Vector3d::UnitX()).toRotationMatrix();
    }
    inline Eigen::Matrix3d rotationY(double t) {
        return Eigen::AngleAxisd(-t, Eigen::Vector3d::UnitY()).toRotationMatrix();
    }
    inline Eigen::Matrix3d rotationZ(double t) {
        return Eigen::AngleAxisd(-t, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }

    // 地球自转(Sagnac)改正统一复用 src/core/CoordStruct.h 的 applyEarthRotation(...)，PPP/LEO/SPP 三端共用。

    inline double frobenius(const Eigen::MatrixXd &m) {
        double sum(0);
        for (auto i = 0; i < m.rows(); i++)
            for (auto j = 0; j < m.cols(); j++)
                sum += m(i, j) * m(i, j);
        return sqrt(sum);
    }

    /** returns the magnitude of the vector */
    inline double magnitude(const Eigen::VectorXd &v) {
        double mag(0);
        if (v.size() == 0) return mag;
        mag = abs(v(0));

        for (auto i = 1; i < v.size(); i++) {
            if (mag > abs(v(i)))
                mag *= sqrt(1.0 + v(i) / mag * (v(i) / mag));
            else if (abs(v(i)) > mag)
                mag = abs(v(i)) * sqrt(1.0 + mag / v(i) * (mag / v(i)));
            else
                mag *= sqrt(2.0);
        }
        return mag;
    }

    /// 算术均值
    inline double mean(const std::vector<double> &x) {
        if (x.empty()) return 0.0;
        double s = 0.0;
        for (const double v : x) s += v;
        return s / static_cast<double>(x.size());
    }

    /// 均方根
    inline double rms(const std::vector<double> &x) {
        if (x.size() < 2) return 0.0;
        const double m = mean(x);
        double s = 0.0;
        for (const double v : x) s += (v - m) * (v - m);
        return std::sqrt(s / static_cast<double>(x.size() - 1));
    }

    /// 中位数
    inline double median(std::vector<double> v) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    }
}

inline double sign(const double x) { return x <= 0.0 ? -1.0 : 1.0; }
