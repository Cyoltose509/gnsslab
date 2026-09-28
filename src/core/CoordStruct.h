#pragma once

#include <Eigen/Eigen>
#include "Const.h"   // OMEGA_EARTH, C_MPS（applyEarthRotation 需要）

using namespace std;

struct FrameInfo {
    double a; // 长半轴
    double f; // 扁率
    double gm; // 引力常数
    double omega; // 地球自转角速度
    double e2;
    double j2; // 可选
};

namespace Frame {
    constexpr FrameInfo WGS84{
        6378137.0,
        1.0 / 298.257223563,
        3.986004418e14,
        7.292115e-5,
        2.0 * (1.0 / 298.257223563) - 1.0 / 298.257223563 * (1.0 / 298.257223563),
        0.0,
    };
    constexpr FrameInfo GPS{
        6378137.0,
        1.0 / 298.257223563,
        3.986005e14,
        7.2921151467e-5,
        2.0 * (1.0 / 298.257223563) - 1.0 / 298.257223563 * (1.0 / 298.257223563),
        0.0
    };

    constexpr FrameInfo PZ90{
        6378136.0,
        1.0 / 298.257839,
        3.9860044e14,
        7.292115e-5,
        2.0 * (1.0 / 298.257839) - 1.0 / 298.257839 * (1.0 / 298.257839),
        1.08262575e-3
    };

    constexpr FrameInfo CGCS2000{
        6378137.0,
        1.0 / 298.257222101,
        3.986004418e14,
        7.292115e-5,
        2.0 * (1.0 / 298.257222101) - 1.0 / 298.257222101 * (1.0 / 298.257222101),
        0.0,
    };
}


class XYZ : public Eigen::Vector3d {
public:
    // 默认构造函数
    XYZ() : Eigen::Vector3d(0.0, 0.0, 0.0) {
    }

    XYZ(const Eigen::Vector3d &vec) : Eigen::Vector3d(vec) //NOLINT
    {
    }

    // 带参数的构造函数
    XYZ(const double x_, const double y_, const double z_) : Eigen::Vector3d(x_, y_, z_) {
    }

    [[nodiscard]] double X() const { return this->x(); }
    [[nodiscard]] double Y() const { return this->y(); }
    [[nodiscard]] double Z() const { return this->z(); }

    // 成员函数形式重载减法运算符
    Eigen::Vector3d operator-(const XYZ &other) const {
        Eigen::Vector3d result;
        result[0] = this->x() - other.x();
        result[1] = this->y() - other.y();
        result[2] = this->z() - other.z();
        return result;
    }
};


class BLH : public Eigen::Vector3d {
public:
    BLH() : Eigen::Vector3d(0.0, 0.0, 0.0) {
    }

    // 默认构造函数
    BLH(const Eigen::Vector3d &vec) : Eigen::Vector3d(vec) { //NOLINT
    }

    // 带参数的构造函数
    BLH(const double B_, const double L_, const double H_) : Eigen::Vector3d(B_, L_, H_) {
    }

    [[nodiscard]] double B() const { return this->x(); }
    [[nodiscard]] double L() const { return this->y(); }
    [[nodiscard]] double H() const { return this->z(); }
};

class ENU : public Eigen::Vector3d {
public:
    ENU() : Eigen::Vector3d(0.0, 0.0, 0.0) {
    }

    // 默认构造函数
    ENU(const Eigen::Vector3d &vec) : Eigen::Vector3d(vec) { //NOLINT
    }

    // 带参数的构造函数
    ENU(const double E_, const double N_, const double U_) : Eigen::Vector3d(E_, N_, U_) {
    }

    [[nodiscard]] double E() const { return this->x(); }
    [[nodiscard]] double N() const { return this->y(); }
    [[nodiscard]] double U() const { return this->z(); }
};

// 地球自转(Sagnac)改正：将卫星在发射时刻 tTx 的 ECEF 位置/速度旋转到接收时刻 tRx 的 ECEF 框架。
inline void applyEarthRotation(Eigen::Vector3d &pTx, Eigen::Vector3d &vTx,
                              const Eigen::Vector3d &recv) {
    const double tau = (pTx - recv).norm() / C_MPS;   // 信号传播时间
    const double ang = OMEGA_EARTH * tau;             // +ωτ
    const double c = std::cos(ang), s = std::sin(ang);
    Eigen::Matrix3d rot;                              // R(−ωτ)
    rot <<  c, s, 0,
           -s, c, 0,
            0, 0, 1;
    pTx = rot * pTx;
    vTx = rot * vTx;
}

// ECEF 位置合理性判据：有限、且处在地球附近（模长 1e5~1e8.5 m 量级）。
// 运动学/LEO 的冷启动种子与发散兜底共用（原先在 PPPKinematic.cpp 与 LEO.cpp 各写一份）。
inline bool isSaneEcef(const Eigen::Vector3d &v) {
    return v.allFinite() && v.squaredNorm() > 1.0e10 && v.squaredNorm() < 1.0e17;
}
