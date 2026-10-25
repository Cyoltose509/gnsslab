#pragma once
#include <set>
#include <string>

namespace AppConfig { struct IniFile; }

struct SolveConfig {
    // ---- [input] ----
    std::string source = "file"; //file | realtime
    std::string obsPath; // SPP / PPP / LEO 的 RINEX 观测
    std::string navPath; // 广播星历（SPP/PPP/RTK 共用 brdc）
    std::string basePath, roverPath; // RTK 基准站 / 流动站
    std::string ip; // SPP realtime 接收机 IP
    int port = 0; // SPP realtime 端口
    std::string baseIp, roverIp;
    int basePort = 0, roverPort = 0;

    // PPP 精密产品
    std::string sp3, clk, atx, osb, erp, tro, ionex;
    std::string gpt2; // GPT2 全球格网：TRO 缺站时给先验 ZWD
    // LEO 精密产品（与 PPP 共用 atx/osb，另有自己的 gnss* 与参考轨道）
    std::string gnssSp3, gnssClk, gnssBrdc, refSp3, aux;
    int refPrn = 49; // LEO 参考轨道卫星 PRN

    // SPP realtime 停止条件：realtimeSec>0 计时停止；maxEpochs>0 到数停止；两者皆 0 时按回车停止
    int realtimeSec = 0;
    int maxEpochs = 0;

    // ---- [solver] ----

    double cutoffDeg = 15.0;// 截止高度角(deg)
    std::set<char> systems{'G', 'C'};
    double sigIFCode = 1.273, sigPhase = 0.01273;
    int useAR = 1, useCycleSlip = 1, useGfCycleSlip = 1, fullCovariance = 1;
    int solidTide = 1; // 固体潮（PCO/PCV 由 ATX 文件是否存在自动决定，不再设开关）
    int kinematic = 0; // PPP / LEO 运动学
    double posNoise = 100.0;
    double staticPosNoise = 0.0;
    int applyWindup = 1; // 相位缠绕总开关
    double ztdQ = 1.0E-8; // ZTD 过程噪声(m^2/epoch)
    double ambQ = 1.0E-8; // 模糊度过程噪声(m^2/epoch)
    double troZtdSigma = 0.05; // TRO 产品 ZTD 约束 sigma(m)
    int useTroGrad = 1; // TRO 水平梯度固定改正开关
    std::string tropoMap = "gmf"; // 对流层映射函数: gmf(默认,对接VMF3) / nmf(Niell)
    double ztdPrior = 0.30; // ZTD 先验 sigma(m)
    double ztdInitValue = 0.0, ztdInitVar = 0.36;

    double ambInitVar = 3600.0;    // 模糊度初方差(m^2)
    double motionThresh = 0.0, sppVsPredThresh = 300.0;

    double firstEpochPosVar = 3600.0; // 首历元位置先验方差(m^2)

    double keepAmbMaxGap = 0.0;// 间隙后保留模糊度的最大间隙(s)

    double codeBiasProcNoise = 1.0E-8; // 码偏置随机常数过程噪声(m^2/epoch)
    double codeBiasInitVar = 900.0;// 码偏置初方差(m^2)

    double gloIfcbProcNoise = 1.0E-8; // 逐星 IFCB 随机常数过程噪声(m^2/epoch)
    double gloIfcbInitVar = 900.0; // 逐星 IFCB 初方差(m^2)

    // ISB（系统间偏差）过程噪声/初方差：非参考系统接收机钟差(cdt2~4)不再当白噪声，
    // 改以"常数(isb_proc_noise=0)"或"随机游走(isb_proc_noise>0)"估计（对齐老师"ISB 当常数/随机游走估"）。
    // 参考系统(GPS)钟差仍走 clock* 默认白噪声。GPS-only 不受任何影响。
    // 默认 RW-ISB：isb_proc_noise=1.0e-3、isb_init_var=1.0（GLONASS 同样走 RW-ISB）。
    // ini 用标量 isb_proc_noise / isb_init_var 下发；isb_init_var<=0 ⇒ 未启用(白噪声旧口径)。
    double isbProcNoise = 1.0e-3; // ISB 随机游走 Q(m^2/epoch)；0=常数 ISB
    double isbInitVar = 1.0;      // ISB 初方差(m^2)；<=0 表示未启用，cdt2~4 走 clock* 白噪声
    double posVarFloor = 1.0E-6;

    // ---- [output] ----
    std::string csvPath; // 输出 CSV（为空则按 exe 名取默认）
    double refX = 0, refY = 0, refZ = 0; // 可选 ENU 参考点(ECEF)；全 0 表示用首历元解作参考
    bool hasRef = false;
};

SolveConfig loadConfig(const AppConfig::IniFile &ini);
