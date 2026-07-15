#pragma once
/**
 * QCProcessor — 观测数据质量分析（纯计算，无 GUI 依赖）
 * --------------------------------------------------------------------------
 * 此模块从 lib 层提供 GNSS 观测质量评估的全部算法，UI 层 (main/widgets) 只负责
 * 把解算结果转成 QCObsEpoch 并调用 QC::compute()，再渲染返回的 QualityReport。
 *
 * 参考：
 *   BD 420022—2019《北斗/GNSS 测量型接收机观测数据质量评估方法》
 *   Estey & Meertens (1999) 多路径组合
 *
 * 实现指标：
 *   A. 观测数据完整率   (单系统/单星，按频点有效历元数 / 理论历元数)
 *   B. 周跳比统计       (MW 宽巷 + 几何无关 L4 联合探测；周跳比 = 历元数 / 周跳数)
 *   C. 多路径误差       (MP1 / MP2，Estey & Meertens 双频组合)
 *   D. 观测值噪声       (伪距/载波相位三次差标准差)
 *   E. 电离层残差变化率 IOD (m/s, BD 420022—2019 §6.2.3)
 *
 * 说明（用户 2026-07-14 要求）：
 *   - 几何无关组合 L4 不再参与任何展示/统计（仅作为周跳探测的内部量保留，不对外暴露）。
 *   - MP 仅做「全局去均值」(减去整段均值)，不再做逐弧段中心化/多项式去趋势。
 *   - 无几何信息的卫星(全部历元 elevation=0，通常因无星历/无有效码)不计入 QC 统计。
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
    int validDual = 0;       // 双频均有效的历元数
    double completeness = 0; // 完整率 % = validDual / totalEpochs * 100
    int slips = 0;           // 探测到的周跳次数
    double slipRatio = 0;    // 周跳比 = totalEpochs / slips (历元/周跳)，无周跳时为 INF→用 totalEpochs 表示

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
    std::vector<double> mp1, mp2; // m  (全局去均值版)
    std::vector<double> ionoRate; // m/s 电离层残差变化率 IOD (逐历元差分)
    std::vector<char> slipFlag;   // 该历元是否发生周跳 (1=是)

    // 与 RTKLIB 对齐：SNR/EL/AZ 序列及统计
    std::vector<double> snr;      // dB-Hz, 与 t 对齐 (选中频点的 SNR)
    double snrMean = 0, snrMin = 0, snrMax = 0;
};

struct QualityReport {
    int totalEpochs = 0;       // 已解算（用于统计）的历元数
    int totalInputEpochs = 0;  // 文件读入的全部历元数（含未解算），供概览对照
    double interval = 0;       // 估计的历元间隔 (s)
    std::vector<SatID> satOrder;
    std::map<SatID, SatQC> sats;

    std::map<char, double> sysCompleteness; // 按系统均值
    std::map<char, double> sysSlipRatio;
    std::map<char, double> sysMp1, sysMp2;
    std::map<char, double> sysSigRho, sysSigPhase;
    std::map<char, int> sysSlips, sysIonoJumps;

    std::map<char, double> sysSnr;
    double overallSnr = 0;

    double overallCompleteness = 0;
};

/// 从逐历元观测数据计算质量分析报告。
/// 纯计算、无 GUI / 线程 / 进度回调依赖——进度展示完全由调用方（UI 层）负责。
QualityReport compute(const std::vector<QCObsEpoch> &epochs);

} // namespace QC
