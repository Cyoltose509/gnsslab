#pragma once

#include <cctype>
#include <string_view>
#include <Eigen/Eigen>
#include "core/Const.h"

/// 椭球几何：XYZ↔BLH、ENU/NED、仰角方位角、子午/卯酉圈半径只认 a/f/e² 这三个数。
/// 引力常数与自转角速度不在这里——那组常数按星座取，见 Ephemeris.h 的 EphConst。
struct GeodeticFrame {
    const char *name;  // 仅用于界面与日志显示
    double a;          // 长半轴 (m)
    double f;          // 扁率
    double e2;         // 第一偏心率平方

    /// 吃规格书原值：长半轴与扁率倒数 1/f，f 与 e² = f(2−f) 由构造算出来。
    constexpr GeodeticFrame(const char *n, const double a_, const double invF)
        : name(n), a(a_), f(1.0 / invF), e2(f * (2.0 - f)) {
    }
};

namespace Frame {
    enum class Ref { IGS14, IGS20, ITRF2014, ITRF2020, WGS84, CGCS2000, PZ90 };

    /// "标签 → 名字"的唯一真源。顺序无关，加一个框架就是加一个枚举项 + 一行。
    struct RefName {
        Ref ref;
        const char *name;
    };

    constexpr RefName kRefNames[] = {
        {Ref::IGS14, "IGS14"}, {Ref::IGS20, "IGS20"},
        {Ref::ITRF2014, "ITRF2014"}, {Ref::ITRF2020, "ITRF2020"},
        {Ref::WGS84, "WGS84"}, {Ref::CGCS2000, "CGCS2000"}, {Ref::PZ90, "PZ90"},
    };

    [[nodiscard]] constexpr const char *refName(const Ref r) {
        for (const RefName &e: kRefNames)
            if (e.ref == r) return e.name;
        return "UNKNOWN";
    }

    /// 名字 → 标签，大小写不敏感（igs14 / IGS14 都认）。
    inline bool refByName(const std::string_view s, Ref &out) {
        for (const auto &[ref, name]: kRefNames) {
            const std::string_view n = name;
            if (n.size() != s.size()) continue;
            bool same = true;
            for (std::size_t k = 0; k < n.size() && same; ++k)
                same = std::tolower(static_cast<unsigned char>(s[k])) ==
                       std::tolower(static_cast<unsigned char>(n[k]));
            if (!same) continue;
            out = ref;
            return true;
        }
        return false;
    }

    /// IGS/ITRF 产品与 CGCS2000 都用 GRS80 椭球（a 与 1/f 相同），WGS84 差在扁率末位，PZ90 是 GLONASS 那套。
    inline constexpr GeodeticFrame WGS84{"WGS84", 6378137.0, 298.257223563};
    inline constexpr GeodeticFrame CGCS2000{"CGCS2000", 6378137.0, 298.257222101};
    inline constexpr GeodeticFrame PZ90{"PZ90", 6378136.0, 298.257839};
}

/// 正常重力场的定义常数，口径取自指导书 1.5（1）的 GRS80 公式 (34)(35)：
///   (34) γ0 = γe (1 + k2 sin²φ + k4 sin⁴φ)
///   (35) γ(φ,h) = γ0 − (h0 − hLat sin²φ)·h + h2·h²，h 按米
/// 注意 (34) 只到 sin⁴φ；再往上加 sin⁶/sin⁸ 会让 γ0 高出 4.5e-9 m/s²，
/// 与示例数据参考轨迹差出一个常值加速度（1 小时高程差 0.19 m）。
struct NormalGravityModel {
    double gammaE;            // 赤道正常重力 γe (m/s²)
    double k2, k4;            // γ0 的 sin²φ / sin⁴φ 系数
    double h0, hLat, h2;      // 高度改正系数，按米
};

namespace NormalGravity {
    constexpr NormalGravityModel GRS80{
        9.7803267715,
        0.0052790414, 0.0000232718,
        3.087691089e-6, 4.397731e-9, 0.721e-12
    };

    /// 正常重力 γ(B,h)：B 为大地纬度 (rad)，h 为椭球高 (m)。
    inline double at(const double latRad, const double hMeters) {
        const double s2 = std::sin(latRad) * std::sin(latRad);
        const double gamma = GRS80.gammaE * (1.0 + GRS80.k2 * s2 + GRS80.k4 * s2 * s2);
        return gamma - (GRS80.h0 - GRS80.hLat * s2) * hMeters + GRS80.h2 * hMeters * hMeters;
    }
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


inline bool isSaneECEF(const Eigen::Vector3d &v) {
    return v.allFinite() && v.squaredNorm() > 1.0e10 && v.squaredNorm() < 1.0e17;
}
