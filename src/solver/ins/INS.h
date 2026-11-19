#pragma once

#include <deque>

#include "core/ins/InsStruct.h"


// 纯惯导捷联机械编排；零速修正（ZUPT）作为开关内建：zuptEnabled=false 即自由解。
// 当量由调用方填好再进来；初始化按 粗对准(align) → 初始化(init) → 静置段标定(calibrate)
// → 逐历元推进(update) 的次序推进，前几步都直接修改自身成员。
class INS {
public:
    // 零速修正内参
    static constexpr double kZuptAttGain = 0.0;            // 静止姿态反馈增益；车停在斜坡上时向 -g 拉会引入横滚误差，故关闭
    static constexpr double kZuptAttRateMax = 0.1 * DEG_TO_RAD; // 姿态修正限速 (rad/s)：真实需求只有零偏量级
    static constexpr double kZuptGyroEstTau = 10.0;         // 陀螺零偏一阶滤波时间常数 (s)
    static constexpr double kZuptEstWarmSec = 10.0;         // 零偏估计的静止保持时长：误判静一般 <3 s，真停 >50 s
    static constexpr double kZuptGyroBiasCap = 1e-4;        // 陀螺零偏估计钳位 (rad/s)
    // 静置段标定内参
    static constexpr double kAccBiasApplyTol = 0.05;        // 加计零偏可信阈值 (m/s^2)，超过则不扣
    static constexpr double kZuptAccStdTol = 0.02;          // 静止判据：比力窗口标准差上限 (m/s^2)
    static constexpr double kZuptGyroStdTol = 0.002;        // 静止判据：角速度窗口标准差上限 (rad/s)

    NavState s;

    // ---- 配置（直填） ----
    bool zuptEnabled = false;
    double zuptAccelTol = 0.3;   // 比力窗口均值偏离静置基准容差 (m/s^2)
    double zuptGyroTol = 0.02;   // 角速度窗口均值偏离静置基准容差 (rad/s)
    double zuptHoldSec = 0.5;    // 连续判静时长 (s)，init 时按采样率换算成样本数
    double gravity = 0.0;        // 初始位置正常重力 (m/s^2)；align 前可直填，init 按初始位置重算

    // ---- 当量（两个当量 + 采样率都由调用方一次给全，src 不查型号也不看时间戳） ----
    ImuScale scale;

    // ---- 静态粗对准（align 成功后有效） ----
    Euler alignAtt;                              // 对准姿态 (rad)
    std::size_t alignBegin = 0;                  // 静止段起样本下标
    std::size_t alignCount = 0;                  // 静止段样本数（也是静置标定段）
    double alignEarthRateErrDeg = 0.0;
    double alignYawSpreadDeg = 0.0;
    double alignGyroSigmaRad = 0.0;
    bool alignHeadingReliable = false;

    // ---- 静置段标定（calibrate 后有效） ----
    Eigen::Vector3d accelBias = Eigen::Vector3d::Zero();    // 静置段标出的加计零偏 (m/s^2)
    bool biasApplied = false;                    // 零偏可信，toEpoch 已在比力增量中扣除
    Eigen::Vector3d gyroBiasDiag = Eigen::Vector3d::Zero(); // 静置陀螺偏离地球自转（含航向误差，只诊断）
    Eigen::Vector3d zuptFRef = Eigen::Vector3d::Zero();     // 静置段比力均值基准（未扣零偏）
    Eigen::Vector3d zuptWRef = Eigen::Vector3d::Zero();     // 静置段角速度均值基准
    bool zuptHasRef = false;                     // 有静置段基准；没有则退化为 |f|≈g 幅值判据

    // ---- 逐历元输出（update 后读） ----
    bool zuptApplied = false;    // 本历元零速修正是否生效
    struct Diag {
        double accDevMax = 0.0;  // 比力窗口均值偏离基准最大值 (m/s^2)
        double gyrDevMax = 0.0;  // 角速度窗口均值偏离基准最大值 (rad/s)
        std::size_t accFail = 0; // 比力偏差超限历元数
        std::size_t gyrFail = 0; // 角速度偏差超限历元数
    };
    Diag zuptDiag;

    // toEpoch 的转换接续点：首个被转换样本的前一个样本（调用方按流接续赋值）。
    ImuSample prevRaw{};
    bool hasPrevRaw = false;

    void align(const ImuLog &head, double latRad);  // scale 必须已知；静置段不足或解算失败抛异常
    void init(const NavState &x0);                  // x0 是初始时刻的完整状态
    void calibrate(const ImuLog &head);  // init 之后；静置段取 alignBegin/alignCount
    InsEpoch toEpoch(const ImuSample &sm);  // 当量/名义间隔/加计零偏/上一样本全取自成员
    void update(const InsEpoch &e);

private:
    void mechanize(const InsEpoch &e); // 式 6.203~6.237 机械编排主体
    InsEpoch makeEpoch(const ImuSample &sm, const ImuSample *prev) const; // 原始样本→增量，唯一转换点
    bool detectStatic(const InsEpoch &e, Eigen::Vector3d &fMean); // update 第一步：判静窗口累计与判定
    void holdStill(const InsEpoch &e, const Eigen::Vector3d &fMean,
                   const Eigen::Vector3d &prevLlh);               // update 第三步：静止修正
    Eigen::Vector3d prevDtheta = Eigen::Vector3d::Zero();  // Δθ_{k-1} (角增量)
    Eigen::Vector3d prevDvel = Eigen::Vector3d::Zero();    // Δv_{k-1}  (速度增量)
    Eigen::Vector3d prevPrevPos = Eigen::Vector3d::Zero();  // p_{k-2}, 用于中点外推 (式 6.206)
    Eigen::Vector3d prevPrevVel = Eigen::Vector3d::Zero();  // v_{k-2}
    bool hasPrev = false;
    Eigen::Vector3d gyroEst = Eigen::Vector3d::Zero(); // 静止段折算的机体系陀螺零偏
    std::deque<InsEpoch> winHist;                      // 判静滑动窗（约 1 s 的增量）
    Eigen::Vector3d fSum = Eigen::Vector3d::Zero();
    Eigen::Vector3d wSum = Eigen::Vector3d::Zero();
    double hSum = 0.0;
    int quiet = 0;
    std::size_t zuptHoldN = 1;   // 连续判静样本数（zuptHoldSec × rateHz，init 时换算）
    std::size_t zuptWinN = 1;    // 判静滑动窗样本数（rateHz，init 时换算）
    std::size_t zuptEstWarmN = 1; // 零偏估计的静止保持样本数（init 时换算）
};
