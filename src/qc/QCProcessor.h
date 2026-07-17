#pragma once
/**
 * QCProcessor — 观测数据质量分析（纯计算，无 GUI 依赖）
 * --------------------------------------------------------------------------
 * 此模块从 lib 层提供 GNSS 观测质量评估的全部算法，UI 层 (main/widgets) 只负责
 * 把解算结果转成 QCObsEpoch 并调用 QC::compute()，再渲染返回的 QualityReport。
 *
 * 参考（严格依据）：
 *   BD 420022—2019《北斗/GNSS 测量型接收机观测数据质量评估方法》
 *   Estey & Meertens (1999) 多路径组合
 *
 * 实现指标（全部按 2019 标准条款）：
 *   6.1 观测数据完整率     —— 频点级 DI_f 与系统级 DIs
 *   6.2 周跳比
 *        6.2.2 粗差探测     —— MW 组合递推均值/方差 + 4σ + 三历元判据区分粗差/周跳
 *        6.2.3 周跳探测     —— MW(LMW) 与 GF(LGF) 联合探测
 *        6.2.4 接收机钟跳探测 —— ΔL 探测 + 多星一致性确认
 *   6.3 多路径误差         —— Estey-Meertens 双频组合 + 滑动窗口(默认 Nsw=50) RMS
 *   6.4 电离层延迟变化率 IOD —— GF 组合 + 0.07 m/s 阈值
 *   6.5 伪距噪声           —— 分弧段三次差，σ = √(Σ(ΔΔΔρ)² / (8(Nρ-1)))
 *   6.6 载波相位噪声       —— 分弧段三次差(周)，同 6.5 分母
 *   6.7 载噪比             —— 逐星均值再对星求平均
 *   6.8 数据质量综合评估   —— 同趋势化 + 无量纲化 + 熵权 + TOPSIS(E 越小越好)
 */
#include <vector>
#include <map>
#include <string>
#include "GnssStruct.h"   // SatID, TypeValueMap

namespace QC {

/// 单历元输入：UI 把 GuiFileProcessor::SppEpochData 转换成此轻量结构后传入，
/// 这样 lib 层完全不依赖任何 GUI / main 代码。
struct QCObsEpoch {
    double sow = 0;                          // 周内秒
    std::vector<SatID> satIds;               // 该历元可见卫星
    std::vector<TypeValueMap> allObs;        // 与 satIds 一一对应的观测类型-数值表
    // 以下由解算器提供（用于与 RTKLIB 对齐的 SNR/EL/DOP/Nsat/Skyplot 视图）
    std::vector<double> elevations;          // deg, 与 satIds 同序
    std::vector<double> azimuths;            // deg, 与 satIds 同序
    double pdop = 0, hdop = 0, vdop = 0, gdop = 0;
    int nsat = 0;
};

struct SatQC {
    SatID sat;
    char band1 = 0;          // 高频 (f1)
    char band2 = 0;          // 低频 (f2)
    int totalEpochs = 0;     // 文件总历元数 (理论历元)
    int validDual = 0;       // 双频均有效的历元数 (用于 DIs)

    double completeness = 0; // 系统级完整率 DIs % = validDual / totalEpochs * 100
    double di_f1 = 0;        // 频点级完整率 DI_f % (band1 有效历元 / 总历元)
    double di_f2 = 0;        // 频点级完整率 DI_f % (band2 有效历元 / 总历元)

    int slips = 0;           // 探测到的周跳次数 (MW+GF，已剔除钟跳误报)
    int outliers = 0;        // 探测到的粗差次数
    int clockJumps = 0;      // 该星被判定为接收机钟跳的历元数
    double slipRatio = 0;    // 周跳比 = totalEpochs / slips (历元/周跳)，无周跳时用 totalEpochs 表示

    double mp1Mean = 0, mp1Rms = 0;
    double mp2Mean = 0, mp2Rms = 0;

    double sigRho1 = 0;      // 伪距噪声 (m)  @f1
    double sigRho2 = 0;      // 伪距噪声 (m)  @f2
    double sigPh1 = 0;       // 载波相位噪声 (周) @f1
    double sigPh2 = 0;       // 载波相位噪声 (周) @f2

    int ionoJumps = 0;       // 电离层残差变化率 IOD 越界 (>0.07 m/s) 次数
    double ionoRateStd = 0;  // IOD std (m/s)

    // 绘图序列 (按时间排序的逐历元记录)
    std::vector<int> epIdx;
    std::vector<double> t;        // sow
    std::vector<double> mp1, mp2; // m  (按弧段去均值，便于观察)
    std::vector<double> ionoRate; // m/s 电离层残差变化率 IOD (逐历元差分)
    std::vector<char> slipFlag;   // 该历元是否发生周跳 (1=是)
    std::vector<char> outlierFlag;// 该历元是否为粗差 (1=是)
    std::vector<char> clockJumpFlag; // 该历元是否为接收机钟跳 (1=是)

    // 与 RTKLIB 对齐：SNR/EL/AZ 序列及统计
    std::vector<double> snr;      // dB-Hz, 与 t 对齐 (选中频点的 SNR)
    double snrMean = 0, snrMin = 0, snrMax = 0;
};

/// 6.8 数据质量综合评估结果
struct ComprehensiveEval {
    std::vector<std::string> indicators;            // 7 项指标名称
    std::vector<double> weights;                    // 熵权 w_l (size = 7)
    std::map<SatID, std::vector<double>> z;          // 各卫星无量纲化值 z_lm (size 7)
    std::map<SatID, double> E;                       // 各卫星综合评估值 (越小越好)
    std::map<char, double> sysE;                    // 各系统综合评估值（系统内卫星 E 的均值）
    double overallE = 0;                            // 全部卫星 E 的均值
};

struct QualityReport {
    int totalEpochs = 0;       // 已解算（用于统计）的历元数
    int totalInputEpochs = 0;  // 文件读入的全部历元数（含未解算），供概览对照
    double interval = 0;       // 估计的历元间隔 (s)
    std::vector<SatID> satOrder;
    std::map<SatID, SatQC> sats;

    std::map<char, double> sysCompleteness; // 系统级完整率 DIs 均值
    std::map<char, double> sysDI_f1, sysDI_f2; // 频点级完整率 DI_f 均值
    std::map<char, double> sysSlipRatio;
    std::map<char, double> sysMp1, sysMp2;
    std::map<char, double> sysSigRho, sysSigPhase;
    std::map<char, int> sysSlips, sysIonoJumps, sysOutliers, sysClockJumps;
    std::map<char, double> sysIonoRate;     // 各系统电离层变化率 std 均值

    std::map<char, double> sysSnr;
    double overallSnr = 0;

    double overallCompleteness = 0;

    int clockJumpEpochs = 0;       // 全局确认发生接收机钟跳的历元数
    ComprehensiveEval comprehensive;
};

/// 从逐历元观测数据计算质量分析报告。
/// 纯计算、无 GUI / 线程 / 进度回调依赖——进度展示完全由调用方（UI 层）负责。
QualityReport compute(const std::vector<QCObsEpoch> &epochs);

} // namespace QC
