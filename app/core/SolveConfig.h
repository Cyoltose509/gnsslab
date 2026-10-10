#pragma once
// 解算任务配置结构：从 ini 的 [input]/[solver]/[output] 三段读入。
// 不同 solver 只填充各自关心的键（见各 ini 模板），未用的键留空/默认即可。
// 配置项与 GUI 的"处理配置"面板一一对应（命名尽量贴近）。
//
// 放在 core 而非 cli/：它是"解算任务的配置模型"这一领域概念，与 ini 文本格式无关；
// GUI 侧日后若也要读同一份 ini，走的是同一条 IniFile → SolveConfig 流水线。
#include <set>
#include <string>

struct IniFile;

struct SolveConfig {
    // ---- [input] ----
    std::string source = "file";     // SPP 专用：file | realtime
    std::string obsPath;             // SPP / PPP / LEO 的 RINEX 观测
    std::string navPath;             // 广播星历（SPP/PPP/RTK 共用 brdc）
    std::string basePath, roverPath; // RTK 基准站 / 流动站
    std::string ip;                  // SPP realtime 接收机 IP
    int         port = 0;            // SPP realtime 端口

    // PPP 精密产品
    std::string sp3, clk, atx, osb, erp, tro, ionex;
    std::string gpt2;                // GPT2 全球格网：TRO 缺站时给先验 ZWD
    // LEO 精密产品（与 PPP 共用 atx/osb，另有自己的 gnss* 与参考轨道）
    std::string gnssSp3, gnssClk, gnssBrdc, refSp3, aux;
    int         refPrn = 49;        // LEO 参考轨道卫星 PRN

    // SPP realtime 停止条件：realtimeSec>0 计时停止；maxEpochs>0 到数停止；两者皆 0 时按回车停止
    int realtimeSec = 0;
    int maxEpochs = 0;

    // ---- [solver] ----
    double cutoffDeg = 10.0;
    std::set<char> systems{'G', 'C'};
    // 默认值对齐实测最优（test/hkws_best.ini / .workbuddy/memory/2026-10-09.md）：
    // 0.01273 = RTKLIB 真实相位 sigma 12.73mm，HKWS 末段 2.06cm（0.003→36.4cm、0.01→3.25cm）。
    double sigIFCode = 0.3, sigPhase = 0.01273;
    int useAR = 1, useCycleSlip = 1, useGfCycleSlip = 1, fullCovariance = 1;
    int satPCO = 1, satPCV = 1, solidTide = 1;
    int rcvPCO = 1;              // 接收机天线 PCO（rtkpost 默认不施）
    int kinematic = 0;               // PPP / LEO 运动学
    double posNoise = 100.0;
    // 静态位置过程噪声(m^2/epoch)。RTKLIB 静态 PPP：P += SQR(prn[5])*dt（prn[5]=1e-3，
    // 30s历元 ⇒ ≈3e-5/历元），即位置并非硬冻结、协方差每历元微增，这是其快速收敛的关键。
    // 我们静态历史硬冻结(posNoise=0)⇒ 收敛慢。0=冻结(历史)，非0=对齐 RTKLIB。
    double staticPosNoise = 0.0;
    int applyWindup = 1;             // 相位缠绕总开关(默认开)。实际是否生效由 PPPStatic 自适应决定：
                                  // 模糊度锁定才启用，未锁定自动停用（见 PPPStatic::updateWindupAdaptivity）
    double ztdQ = 1.0E-8;        // ZTD 过程噪声(m^2/epoch)：TRO 钉死 ZTD 时此值无效；无 TRO 时控制随机游走
    double ambQ = 1.0E-8;        // 模糊度过程噪声(m^2/epoch)：默认近似冻结
    double troZtdSigma = 0.05;   // TRO 产品 ZTD 约束 sigma(m)
    int useTroGrad = 1;           // TRO 水平梯度固定改正开关(默认1；=0 仅 ZTD，便于 A/B)
    std::string tropoMap = "gmf"; // 对流层映射函数: gmf(默认,对接VMF3) / nmf(Niell)
    double ztdPrior = 0.30;      // ZTD 先验 sigma(m)：TRO 缺站时若钉死 0 会把真实日变湿延迟漏进 U；设为极大值(如1e9)即关闭先验，改由观测随机游走估计
    // ZTD 初值(m)与初方差(m^2)。PPPStatic 默认初值 0：湿延迟未建模，首历元会把~0.25m 的
    // 对流层缺额推给位置，且 ZTD 需数小时才爬上来 ⇒ 收敛慢。给合理初值可显著加快收敛。
    double ztdInitValue = 0.0, ztdInitVar = 0.04;
    // 模糊度初方差(m^2)。RTKLIB VAR_BIAS=SQR(60)=3600，我们默认 100（紧 36倍）。
    // 过紧 ⇒ EKF 过度信任不准的"码减相位"初值，位置被牵连、收敛慢。
    double ambInitVar = 100.0;
    double motionThresh = 0.0, sppVsPredThresh = 300.0;
    // 首历元位置先验方差(m^2)。SPP 锚定初值常带 m 级偏差，先验过紧会把 EKF 拴在初值附近，
    // 表现为 20h+ 的单调慢收敛；调大 = 更信任观测、更快脱离初值吸引域。
    double firstEpochPosVar = 1.0;
    // 首历元是否走 SPP 引导（给钟差/位置起点）。PPPStatic 默认 0；harness 曾置 1。
    int firstEpochSPP = 0;

    // ---- [output] ----
    std::string csvPath;             // 输出 CSV（为空则按 exe 名取默认）
    double refX = 0, refY = 0, refZ = 0; // 可选 ENU 参考点(ECEF)；全 0 表示用首历元解作参考
    bool hasRef = false;
};

SolveConfig loadConfig(const IniFile &ini);
