#pragma once

#include <Eigen/Eigen>

using namespace std;

struct ReferenceFrame {
    double a; // 长半轴
    double f; // 扁率
    double gm; // 引力常数
    double omega; // 地球自转角速度
    double e2;
    double j2; // 可选
};

namespace Frame {
    constexpr ReferenceFrame WGS84{
        6378137.0,
        3.3528106647474e-3,
        3.986004418e14,
        7.292115e-5,
        6.6694379990e-3,
        0.0,
    };
    constexpr ReferenceFrame GPS{
        6378137.0,
        3.3528106647474e-3,
        3.986005e14,
        7.2921151467e-5,
        6.6694379990e-3,
        0.0
    };

    constexpr ReferenceFrame PZ90{
        6378136.0,
        3.352803743019e-3,
        3.9860044e14,
        7.2921150e-5,
        6.6943661930987e-3,
        1.08262575e-3
    };
}


// class CGCS2000, 请增加类，以管理bds的椭球

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

    double E() const { return this->x(); }
    double N() const { return this->y(); }
    double U() const { return this->z(); }
};
