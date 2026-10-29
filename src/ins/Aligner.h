#pragma once
#include <vector>
#include "core/CoordConvert.h"
#include "core/InsMath.h"
#include "core/InsStruct.h"

// 对准质量：按指导书实验二能真检验的量组织（不是"看起来像质量"的数字）
//   1) 姿态矩阵是否单位正交 —— 指导书式(30)(31)(32) 先把参与解算的矢量正交化、归一化，
//      其目的就是保证解出的 C 是单位正交阵，所以 det(C)=+1、CᵀC=I 是可直接验收的硬指标；
//   2) 重力模长 / 自转模长 —— 指导书实验目的(3)「了解影响对准精度的误差因素」：
//      |f| 应等于当地正常重力、|ω| 应等于 Ω，偏差即当量/比例因子/安装误差的直接体现；
//   3) 反解纬度 φ̂ —— 只用三轴计数的**比值**（当量在比值里约掉），与输入 φ 比较即可判断
//      陀螺有没有资格做解析粗对准（指导书脚注：陀螺全温零偏 ≤ 1°/h 才有此能力）。
struct AlignQuality {
    Eigen::Vector3d axisX = Eigen::Vector3d::UnitX(); // 机体 X 轴在当地 NED 的单位方向
    Eigen::Vector3d axisY = Eigen::Vector3d::UnitY(); // 机体 Y 轴在当地 NED 的单位方向
    Eigen::Vector3d axisZ = Eigen::Vector3d::UnitZ(); // 机体 Z 轴在当地 NED 的单位方向

    // (1) 单位正交性（式 32 的直接验收）
    double dcmDet = 1.0;      // det(C)，应为 +1
    double dcmOrthoErr = 0.0; // max|CᵀC − I|，应为 0

    // (2) 实测 ω 与 ω_ie^n 的夹角（°）。注意 TRIAD 保证重力方向严格对齐，故重力方向残差恒为 0，
    //     这里只保留真正非构造性的角量：沿重力方向的分量不受双矢量定姿约束。
    double earthRateErrDeg = 0.0;

    // (3) 模长校验：需要型号当量（无型号时 hasScale=false，界面必须标「需型号」，不得填 0 冒充实测）
    bool hasScale = false;
    double accelNorm = 0.0;  // |f| 实测 (m/s²)
    double gravityRef = 0.0; // 正常重力 g(φ) (m/s²)
    double gyroNorm = 0.0;   // |ω| 实测 (rad/s)
    double gyroRef = 0.0;    // Ω = OMEGA_EARTH (rad/s)

    // (4) 反解纬度（只用计数比值，与当量无关）
    bool latDerivedValid = false;
    double latDerivedDeg = 0.0;
};

// 型号自描述带来的换算能力：来自记录里的 IMU Type → 内置型号表（InsStruct.h::kImuModels）。
// 当量结构就是 InsStruct.h::ImuScale（标定与对准共用一份），known()==false 时模长类指标不可用。

// 静态解析粗对准
class Aligner {
public:
    // 对整段（或指定区间）求均值做一次静态对准
    static Attitude solve(const ImuLog &log,
                          size_t begin, size_t end,
                          const SiteParams &sp);

    static AttQuat solveQuat(const ImuLog &log,
                             size_t begin, size_t end,
                             const SiteParams &sp);

    // 对准质量诊断
    static AlignQuality diagnose(const ImuLog &log,
                                 size_t begin, size_t end,
                                 const SiteParams &sp,
                                 const ImuScale &sen = {});

    static AlignQuality diagnoseMeans(const Eigen::Vector3d &accMean, const Eigen::Vector3d &gyrMean,
                                      const SiteParams &sp,
                                      const ImuScale &sen = {});

    // 滑动窗口序列
    static std::vector<AttitudeSample> alignSeries(const ImuLog &log, int win,
                                                   const SiteParams &sp);

    // 流式滑窗粗对准
    struct AlignStream {
        int win = 200;
        SiteParams sp{};
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
        size_t count = 0;

        void init(int window, const SiteParams &site);

        void step(const ImuSample &s, AttitudeSample &out);
    };
};
