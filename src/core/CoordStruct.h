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
// 导航系局部坐标（NED：x=北 y=东 z=地）。与 ENU 是同一局部水平面的两种轴向命名，
// 差一个置换 + 天向符号，互转见 CoordConvert.h 的 ENUToNED / NEDToENU。
class NED : public Eigen::Vector3d {
public:
    NED() : Eigen::Vector3d(0.0, 0.0, 0.0) {
    }

    NED(const Eigen::Vector3d &vec) : Eigen::Vector3d(vec) { //NOLINT
    }

    NED(const double N_, const double E_, const double D_) : Eigen::Vector3d(N_, E_, D_) {
    }

    [[nodiscard]] double N() const { return this->x(); }
    [[nodiscard]] double E() const { return this->y(); }
    [[nodiscard]] double D() const { return this->z(); }
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


inline bool isSaneECEF(const Eigen::Vector3d &v) {
    return v.allFinite() && v.squaredNorm() > 1.0e10 && v.squaredNorm() < 1.0e17;
}


// e → ENU 旋转矩阵 C_e^enu（行 = 当地 东/北/天 在 ECEF 中的方向余弦）。
// 命名与归属同 CoordConvert.h 命名律的矩阵族（ECEFToNEDDCM 的孪生：差一个置换+天向符号）。
// ⚠️ 为何留在本文件而非 CoordConvert.h：GnssStruct.h（computePositionDops）依赖它，而
// GnssStruct.h 不能 include CoordConvert.h——后者经 MathUtils.h 把全局 using namespace std
// 扩散进全部 solver TU，正是 GuiHelpers _Fn 事故的根因顺序。故此处是受包含关系约束的例外。
inline Eigen::Matrix3d ECEFToENUDCM(const double B, const double L) {
    const double sinB = std::sin(B);
    const double cosB = std::cos(B);
    const double sinL = std::sin(L);
    const double cosL = std::cos(L);

    Eigen::Matrix3d R;
    R << -sinL, cosL, 0.0,
            -sinB * cosL, -sinB * sinL, cosB,
            cosB * cosL, cosB * sinL, sinB;

    return R;
}
