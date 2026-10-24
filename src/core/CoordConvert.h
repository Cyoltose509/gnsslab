#pragma once

#include "CoordStruct.h"  // ECEFToENUDCM 等（已含 Eigen/cmath），无循环依赖
#include <Eigen/Eigen>
#include <cmath>
#include "MathUtils.h"   // Math::rotationZ（ECI↔ECEF 用）
#include <Eigen/Geometry>  // Quaterniond / AngleAxisd（AttQuat 类）

// 机体→导航 方向余弦矩阵 C_b^n 与欧拉角的互转（姿态/坐标转换）。
// 导航系取 NED（x=北,y=东,z=地），机体取前-右-下；旋转顺序 yaw(ψ,z)-pitch(θ,y)-roll(φ,x)。
inline Eigen::Matrix3d EulerToDCM(const double roll, const double pitch, const double yaw) {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    Eigen::Matrix3d C;
    C << cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy,
            cp * sy, cr * cy + sr * sp * sy, -sr * cy + cr * sp * sy,
            -sp, sr * cp, cr * cp;
    return C;
}

inline void DCMToEuler(const Eigen::Matrix3d &C, double &roll, double &pitch, double &yaw) {
    pitch = std::atan2(-C(2, 0), std::sqrt(C(2, 1) * C(2, 1) + C(2, 2) * C(2, 2)));
    roll = std::atan2(C(2, 1), C(2, 2));
    yaw = std::atan2(C(1, 0), C(0, 0));
}


// ===================== 惯性导航坐标系族（i / e / n / b）转换轮子 =====================
// 坐标系约定（与课件/指导书一致）：
//   i 系（ECI，地心惯性系）：卫星星历解算框架；忽略岁差/章动时与 e 系只差一个格林尼治真恒星时旋转
//        （完整岁差+章动+极移模型见 Geodesy::eciToECEF）。
//   e 系（ECEF，地心地固系 WGS84）：XYZ 直角坐标；大地坐标 BLH 见 XYZToBLH / BLHToXYZ。
//   n 系（导航系 NED）：x=北 y=东 z=地；INS 姿态/速度/位置积分框架。
//   b 系（载体/机体系）：前-右-下；IMU 原始输出即在 b 系。
//   ENU（东-北-天）：GNSS 侧惯用的局部框架，与 n 系差置换+天向符号（ENUToNED / NEDToENU）。
// 方向余弦矩阵记法 C_a^b：v^b = C_a^b · v^a（列向量为 a 系各轴在 b 系的方向余弦）。
// ⚠️ Math::rotationX/Y/Z 是负角约定（内部 AngleAxisd(-t)）；本文件新增 DCM 均已按该约定
//    实现或显式验证，调用方不要再取负号。
//
// 命名律（新增转换函数必须遵守，勿再造 From 式 / 第三种连接符写法）：
//   统一形状 <源>To<目标>[DCM]，连接符一律大写 To；缩写 token 一律全大写
//   （ECEF/ECI/DCM；Euler 是人名不是缩写，保持首字母大写）：
//   1) 系间旋转矩阵（C_a^b，返回 Matrix3d）：<源>To<目标>DCM，例：ECEFToNEDDCM(B,L) /
//      ECEFToENUDCM(B,L) / ECIToECEFDCM(θ) / ECEFToECIDCM(θ) / EulerToDCM(r,p,y) /
//      DCMToEuler(C,&r,&p,&y)。
//   2) 坐标/向量在系间转换（CoordStruct.h 的结构体类型之间）：<源类型>To<目标类型>，
//      token 用类型名全大写缩写（XYZ/BLH/ENU/NED），不带后缀。例：XYZToBLH / XYZToENU /
//      XYZToNED / ENUToNED。
//   两族一眼区分：有 DCM 后缀的是旋转矩阵，没有的是坐标/向量。
//   Geodesy::eciToECEF 是完整岁差/章动/极移模型（类静态方法，小写开头遵循 Geodesy.h
//   自身风格），与仅差 GMST 旋转的 ECIToECEFDCM 是两个函数，按需选用。
//   注：局部变量仍按 camel 小写（recvEcef/rsunEcef/posEcef），缩写全大写只约束
//   函数名——公共 API 面；rsunECEF 这类写法会驼峰断裂，可读性更差。
// ============================================================================================
// 坐标转换函数（computePositionDops 见 GnssStruct.h，依赖其中 Result 定义）
inline BLH XYZToBLH(const XYZ &xyz, const FrameInfo &frame) {
    // 获取椭球参数
    const double a = frame.a;
    const double e2 = frame.e2;
    // 计算水平距离 rho（即 sqrt(x^2 + y^2)）
    const double rho = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);

    // 定义阈值，用于判断是否在极点
    constexpr double eps = 1.0e-13;

    // 判断是否在极点
    if (rho < eps) {
        // 在极点，根据 z 的符号判断是南极还是北极
        double B = xyz.Z() > 0 ? PI / 2 : -PI / 2; // 北极为 +90°，南极为 -90°
        double L = 0.0; // 经度在极点无定义，通常设为 0
        double H = fabs(xyz.Z()) - a * sqrt(1 - e2); // 高度计算

        return {B, L, H};
    }

    // 不在极点，正常计算
    double B0 = atan2(xyz.Z(), rho);

    // 迭代计算大地纬度 B
    int iterationCount = 0;
    double B1, N;
    do {
        N = a / sqrt(1 - e2 * sin(B0) * sin(B0));
        B1 = atan2(xyz.Z() + e2 * N * sin(B0), rho);

        if (fabs(B1 - B0) < eps) break;

        B0 = B1;
        iterationCount++;

        if (constexpr int maxIterations = 100; iterationCount > maxIterations) {
            throw std::runtime_error("Iteration did not converge.");
        }
    } while (true);

    // 计算大地经度 L
    double L = atan2(xyz[1], xyz[0]);

    // 计算高度 H
    double H = rho / cos(B1) - N;

    // 返回大地坐标
    return {B1, L, H};
}

inline XYZ BLHToXYZ(const BLH &blh, const FrameInfo &frame) {
    const double B = blh[0]; // 纬度 (rad)
    const double L = blh[1]; // 经度 (rad)
    const double H = blh[2]; // 高程 (m)

    const double a = frame.a;
    const double e2 = frame.e2;

    const double sinB = sin(B);
    const double cosB = cos(B);
    const double sinL = sin(L);
    const double cosL = cos(L);

    const double N = a / sqrt(1 - e2 * sinB * sinB);

    double x = (N + H) * cosB * cosL;
    double y = (N + H) * cosB * sinL;
    double z = (N * (1 - e2) + H) * sinB;

    return {x, y, z};
}

inline ENU XYZToENU(const XYZ &xyz, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84) {
    const auto diffXYZ = xyz - refXYZ;
    const auto refBLH = XYZToBLH(refXYZ, frame);
    const Eigen::Matrix3d R = ECEFToENUDCM(refBLH.B(), refBLH.L());
    return {R * diffXYZ};
}

inline XYZ ENUToXYZ(const ENU &enu, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84) {
    const auto refBLH = XYZToBLH(refXYZ, frame);
    const Eigen::Matrix3d R = ECEFToENUDCM(refBLH.B(), refBLH.L());
    const auto diffXYZ = R.transpose() * enu;
    XYZ resXYZ;
    resXYZ << refXYZ[0] + diffXYZ[0], refXYZ[1] + diffXYZ[1], refXYZ[2] + diffXYZ[2];
    return resXYZ;
}

inline double elevation(const XYZ &refXYZ, const XYZ &targetXYZ, const FrameInfo &frame = Frame::WGS84) {
    // 先转到ENU
    const ENU enu = XYZToENU(targetXYZ, refXYZ, frame);
    const double elev = atan2(enu(2), sqrt(enu(0) * enu(0) + enu(1) * enu(1)));
    return elev;
}

inline double azimuth(const XYZ &refXYZ, const XYZ &targetXYZ, const FrameInfo &frame = Frame::WGS84) {
    // 转到ENU
    const ENU enu = XYZToENU(targetXYZ, refXYZ, frame);
    double az = atan2(enu(0), enu(1));
    if (az < 0.0) az += PI * 2.0;
    return az;
}

// M6: SPP/RTK 共用——由接收机与卫星位置一次性算仰角+方位，消除两边重复的 elevation/azimuth 调用样板
inline void satElevAzim(const XYZ &refXYZ, const XYZ &satPos, double &elev, double &azim,
                        const FrameInfo &frame = Frame::WGS84) {
    elev = elevation(refXYZ, satPos, frame);
    azim = azimuth(refXYZ, satPos, frame);
}

inline void geometricRangeAndLos(const Eigen::Vector3d &satPos, const Eigen::Vector3d &recvPos,
                                 double &rho, Eigen::Vector3d &losUnit) {
    const Eigen::Vector3d d = recvPos - satPos;
    rho = d.norm();
    losUnit = d.normalized();
}


struct RtnComponents {
    double R = 0.0; // 径向误差(m)
    double T = 0.0; // 沿迹误差(m)
    double N = 0.0; // 法向误差(m)
    double d3 = 0.0; // 三维位置误差模长(m)
    bool hasFull = false;
};

/// posEst: 解算位置 ECEF(m)；posRef: 参考位置 ECEF(m)；velRef: 参考速度 ECEF(m/s)
inline RtnComponents computeRTN(const Eigen::Vector3d &posEst, const Eigen::Vector3d &posRef,
                                const Eigen::Vector3d &velRef) {
    RtnComponents out;
    const Eigen::Vector3d err = posEst - posRef;
    out.d3 = err.norm();
    const Eigen::Vector3d ur = posRef.normalized();
    out.R = err.dot(ur);
    if (velRef.squaredNorm() > 1.0) {
        const Eigen::Vector3d ut = velRef.normalized();
        const Eigen::Vector3d uc = ur.cross(ut).normalized();
        out.T = err.dot(ut);
        out.N = err.dot(uc);
        out.hasFull = true;
    }
    return out;
}

inline NED ENUToNED(const ENU &enu) { return NED(enu.N(), enu.E(), -enu.U()); }
inline ENU NEDToENU(const NED &ned) { return ENU(ned.E(), ned.N(), -ned.D()); }

// e → n（NED）------------------------------------------------------------------
// C_e^n：行 = 当地 北/东/地 在 ECEF 中的方向余弦。与 ECEFToENUDCM（C_e^enu）差一个置换。
inline Eigen::Matrix3d ECEFToNEDDCM(const double B, const double L) {
    const double sinB = std::sin(B), cosB = std::cos(B);
    const double sinL = std::sin(L), cosL = std::cos(L);
    Eigen::Matrix3d C;
    C << -sinB * cosL, -sinB * sinL, cosB,
            -sinL, cosL, 0.0,
            -cosB * cosL, -cosB * sinL, -sinB;
    return C;
}

inline Eigen::Matrix3d NEDToECEFDCM(const double B, const double L) {
    return ECEFToNEDDCM(B, L).transpose();
}

// ECEF ↔ NED（带参考点；与 XYZToENU/ENUToXYZ 同构，只是导航系用 NED）
inline NED XYZToNED(const XYZ &xyz, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84) {
    const Eigen::Vector3d diff = xyz - refXYZ;
    const auto refBLH = XYZToBLH(refXYZ, frame);
    return {ECEFToNEDDCM(refBLH.B(), refBLH.L()) * diff};
}

inline XYZ NEDToXYZ(const NED &ned, const XYZ &refXYZ, const FrameInfo &frame = Frame::WGS84) {
    const auto refBLH = XYZToBLH(refXYZ, frame);
    const Eigen::Vector3d d = ECEFToNEDDCM(refBLH.B(), refBLH.L()).transpose() * Eigen::Vector3d(ned);
    return XYZ(refXYZ.X() + d.x(), refXYZ.Y() + d.y(), refXYZ.Z() + d.z());
}

// i → e ------------------------------------------------------------------------
// C_e^i：v^e = C_e^i·v^i，theta = 格林娼治真恒星时 (rad)。
// Math::rotationZ(theta) 的负角约定恰好等于 R3(theta)，与 Geodesy::eciToECEF 的
// W*R3 分量同构；岁差/章动/极移请用 Geodesy::eciToECEF。
inline Eigen::Matrix3d ECIToECEFDCM(const double theta) { return Math::rotationZ(theta); }
inline Eigen::Matrix3d ECEFToECIDCM(const double theta) { return Math::rotationZ(theta).transpose(); }

// 姿态四元数 -------------------------------------------------------------------
// 统一约定：q 表示 C_b^n（v^n = q⊗v^b⊗q*）。Eigen::Quaterniond 系数顺序为
// (x, y, z, w)（coeffs()），构造函数按 (w, x, y, z) 传参；一律经本类封装，
// 调用方不直接碰系数（raw() 仅作与 Eigen 互操作的逃逸口）。
// 命名法：一元运算用实例方法（toDCM/toEuler/rotate/update…），构造用静态工厂
// （fromDCM/fromEuler/fromRotationVector），二元乘法走 operator*。
class AttQuat {
public:
    AttQuat() = default;                             // 单位四元数
    explicit AttQuat(const Eigen::Quaterniond &q);   // 显式：禁止隐式转换进本类

    // ---- 静态工厂 ----
    static AttQuat identity() { return AttQuat(Eigen::Quaterniond::Identity()); }

    static AttQuat fromDCM(const Eigen::Matrix3d &C) { return AttQuat(Eigen::Quaterniond(C)); }

    static AttQuat fromEuler(const double roll, const double pitch, const double yaw) {
        return AttQuat(Eigen::Quaterniond(EulerToDCM(roll, pitch, yaw)));
    }

    // 旋转矢量 → 四元数（绕 rv 方向精确旋转 |rv|，指数映射）
    static AttQuat fromRotationVector(const Eigen::Vector3d &rv) {
        const double ang = rv.norm();
        if (ang < 1e-12) return identity();
        return AttQuat(Eigen::Quaterniond(Eigen::AngleAxisd(ang, rv / ang)));
    }

    // ---- 导出 ----
    [[nodiscard]] Eigen::Matrix3d toDCM() const { return mQ.toRotationMatrix(); }

    void toEuler(double &roll, double &pitch, double &yaw) const { DCMToEuler(toDCM(), roll, pitch, yaw); }

    [[nodiscard]] Eigen::Vector3d toRotationVector() const {
        const Eigen::AngleAxisd aa(normalized().toDCM());  // 先归一化再取轴角（Eigen 假定单位四元数）
        return aa.axis() * aa.angle();
    }

    [[nodiscard]] double norm() const { return mQ.coeffs().norm(); }

    // ---- 运算 ----
    [[nodiscard]] Eigen::Vector3d rotate(const Eigen::Vector3d &v) const { return mQ * v; }

    [[nodiscard]] AttQuat operator*(const AttQuat &rhs) const { return AttQuat(mQ * rhs.mQ); } // Hamilton 积

    [[nodiscard]] AttQuat conjugated() const { return AttQuat(mQ.conjugate()); }

    [[nodiscard]] AttQuat normalized() const {
        const double n = mQ.coeffs().norm();
        return n > 1e-12 ? AttQuat(Eigen::Quaterniond(mQ.coeffs() / n)) : identity(); // 防除零兜底
    }

    // 姿态递推（b 系角速率 ω_ib^b 右乘更新）：q_{k+1} = q_k ⊗ exp(ω_ib^b·dt/2)。
    // 对逐段常值 ω 精确（旋转矢量指数映射）；dt 内 ω 变化的整流误差（coning）
    // 由导航机械编排层处理，本函数保持单步精确。
    [[nodiscard]] AttQuat update(const Eigen::Vector3d &omegaBody, const double dt) const {
        return (normalized() * fromRotationVector(omegaBody * dt)).normalized();
    }

    [[nodiscard]] const Eigen::Quaterniond &raw() const { return mQ; }

private:
    Eigen::Quaterniond mQ{1.0, 0.0, 0.0, 0.0};  // (w, x, y, z)，默认单位
};

inline AttQuat::AttQuat(const Eigen::Quaterniond &q) : mQ(q) {
}
