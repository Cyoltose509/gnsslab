#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "Ephemeris.h"
#include "EphemerisTable.h"
#include "RinexNavStore.h"   // loadBrdc：广播星历加载（与 PPP 一致，类内加载）
#include "FreqCombo.h"
#include "Weight.h"
#include "SolverOptions.h"   // 统一参数对象：applyOptions 注入
#include <Eigen/Eigen>
#include <set>

// SPP 的 IF 组合类型
using IFCodeTypes = std::map<char, FreqCombo>;

class SPP {
protected:
    struct Config {
        // —— 基础段：SPP 与 PPP 都读 ——
        double cutoffElevRad = 15.0 * DEG_TO_RAD;
        std::set<char> enabledSystems{};
        double sigIFCode = 1.273;
        double convEps = 1e-4; // 收敛阈值(m)
        int maxIter = 10; // 最大迭代
        int minCheckIter = 2; // 起始收敛检查的迭代
        // —— PPP / LEO 专有段：SPP 不读 ——
        double sigmaPhase = 0.01273; // IF 相位量测噪声(PPP 默认)
        double troZtdSigma = 0.05; // TRO ZTD 先验 sigma
        double arRatioThreshold = 3.0; // LAMBDA 成功率阈值
        double arSigGate = 5.0; // 进入 LAMBDA 的浮点模糊度 std 上限(m)
        double posNoise = 0.0; // 位置过程噪声(静态0)

        double posVarFloor = 1.0E-6;// 位置后验方差下限(m^2)
        double ambNoise = 1.0E-8; // 模糊度过程噪声

        double ambInitVar = 3600.0;// 模糊度初方差(m^2)
        double clockNoise = 0.0; // 钟差过程噪声
        double ztdInitValue = 0.0; // ZTD 初值
        double ztdInitVar = 0.36;// ZTD 初方差(m^2)
        double ztdProcNoise = 1.0E-8; // ZTD 随机游走噪声
        double ztdPriorSigma = 0.30; // ZWD 先验 sigma(m)

        double firstEpochPosVar = 3600.0;// 首历元位置先验方差(m^2)
        int minSatsForUpdate = 0; // 滤波更新最少卫星(PPP 默认0=不限)

        // 内部码偏/IFCB 估计：取消开关，改为「自动规则」——外部 OSB 未覆盖的星才估内部码偏/IFCB，
        // OSB 已覆盖则跳过（避免与 OSB 共线）。过程噪声/初方差参数保留供调节。
        double gloIfcbProcNoise = 1.0E-8;   // 逐星 IFCB 随机常数过程噪声(m^2/epoch)
        double gloIfcbInitVar = 900.0;      // 逐星 IFCB 初方差(m^2)

        double keepAmbMaxGap = 0.0;// 观测中断(间隙)恢复时的模糊度策略阈值(s)。
        double codeBiasProcNoise = 1.0E-8;   // 码偏置随机常数过程噪声(m^2/epoch)
        double codeBiasInitVar = 900.0;// 码偏置初方差(m^2)
        int maxReanchorIter = 8; // LEO 重锚定最大迭代
        double sppResidualThr = 10.0; // SPP 后验残差 RMS 阈值(m)
        // ISB（系统间偏差）过程噪声/初方差：非参考系统接收机钟差(cdt2~4)不再当白噪声，
        // 改以"常数(isbProcNoise=0)"或"随机游走(isbProcNoise>0)"估计（对齐老师"ISB 当常数/随机游走估"）。
        // SPP 是逐历元 LSQ，无跨历元状态；此处用于给每历元非参考系统钟差加"上一历元值"伪观测先验
        // （等价于 EKF 里的常数 ISB）。参考系统(GPS)不施加——SPP 每历元重解主钟。GPS-only 不受影响。
        // 默认 RW-ISB：isbProcNoise=1.0e-3、isbInitVar=1.0。由 SPP 解算入口（spp_cli / ppp_cli / leo_cli / GUI 处理器）经 setIsbProcNoise 注入。
        double isbProcNoise = 1.0e-3; // 0=常数 ISB；>0=随机游走(m^2/epoch)，默认 RW-ISB
        double isbInitVar = 1.0;   // ISB 初方差(m^2)，>0 启用；SPP 侧作伪观测先验方差
    };

    Config mConfig;
    Weight mWeight;
    bool mWeightInit = false; // mWeight 首轮解算惰性初始化一次，迭代内不再重建

public:
    bool loadBrdc(const std::string &path);

    void detectIFCombinations(const ObsData &obsData, const IFCodeTypes &defaultTypes = {});

    void setCutoffElevDeg(const double deg) { mConfig.cutoffElevRad = deg * DEG_TO_RAD; }

    /// ZTD 先验 sigma 旋钮（诊断用：调到很大即关闭 ZTD 状态约束，仅留卫星观测估计）
    void setZtdPriorSigma(const double s) { mConfig.ztdPriorSigma = s; }

    /// TRO 产品 ZTD 约束 sigma(m)：>0 且 TRO 含该站时生效并覆盖 ztdPriorSigma；调大=让 ZTD 自由估计。
    void setTroZtdSigma(const double s) { mConfig.troZtdSigma = s; }

    /// ISB 估计模式（对齐老师"ISB 当常数/随机游走估"）：proc==0 为"常数 ISB"，
    /// proc>0 为"随机游走 ISB"(m^2/epoch)；init 为初方差(m^2)。参考系统(GPS)钟差不受影响。
    void setIsbProcNoise(const double proc, const double init) {
        mConfig.isbProcNoise = proc;
        mConfig.isbInitVar = init;
    }

    void setEnabledSystems(const std::set<char> &s) { mConfig.enabledSystems = s; }

    /// 统一参数注入入口：把 SolverOptions 灌入本解算器。
    /// SPP 级只设截止角/系统/ISB；派生类（PPP/LEO）覆盖以追加专有参数。
    virtual void applyOptions(const SolverOptions &o);

    virtual bool processEpoch(ObsData &obsData);

    virtual void preprocess(ObsData &obsData);

protected:
    virtual void readback(const ObsData &obsData);

    virtual void solve(ObsData &obsData);

public:
    virtual void computeSatPos(ObsData &obsData);

    void computeElevAzim();

    virtual void earthRotation();

    /// IF-code 解算的结果位置和钟差，供 UC 模式热启动
    double getClockBias(const char sys) const {
        const auto it = rClockBias.find(sys);
        return it != rClockBias.end() ? it->second : 0.0;
    }

    virtual ~SPP() = default;

    Eigen::Vector3d xyz{0, 0, 0};
    Eigen::Vector3d vel{0, 0, 0};
    FrameInfo frame = Frame::WGS84;
    Result result{};
    SatValueMap satElevData{};
    SatValueMap satAzimData{};
    std::set<SatID> satRejected{};
    std::map<SatID, PVT> satPVTTransTime{};
    std::map<SatID, PVT> satPVTRecTime{};


    EphemerisTable ephTable; // 星历表（值存储，类内拥有）；文件模式由 loadBrdc 填充，流式模式每历元覆盖

protected:
    bool isRover = true;
    bool mRequirePhaseForIF = false;

    /// GPS 与 BDS 各自钟差 [m]
    std::map<char, double> rClockBias{};

    /// 上一历元各系统终值钟差 [m]：ISB 常数/随机游走估计的跨历元载体（SPP 是逐历元 LSQ，
    /// 无跨历元状态，故用此成员把上一历元解出的钟差当作本历元先验起点）。
    /// isbInitVar<=0 时不使用（保持 rClockBias 每历元清空的历史行为）。
    std::map<char, double> mPrevClockBias{};

    /// 系统→钟差参数映射；多系统各自独立钟差=吸收 ISB。LEO 经继承链访问，故置于 protected
    static inline std::map<char, Parameter> sysCdtParam = {
        {'G', Parameter::cdt},
        {'C', Parameter::cdt2},
        {'R', Parameter::cdt3},
        {'E', Parameter::cdt4},
    };

    IFCodeTypes ifCodeTypes{}; // 系统 → FreqCombo（码名 + 频率/系数/波长，及 MW/GF 组合）

    /// GLONASS 每星 IF 组合（FDMA 通道号决定频率）
    std::map<SatID, FreqCombo> mGloCombo;

    /// 取卫星的 IF 频率组合：G/C/E 用系统级，R 用逐星（FDMA）。无可用组合返回 nullptr。
    [[nodiscard]] const FreqCombo *freqComboFor(const SatID &sat) const {
        if (sat.system == 'R') {
            const auto it = mGloCombo.find(sat);
            return it == mGloCombo.end() ? nullptr : &it->second;
        }
        const auto it = ifCodeTypes.find(sat.system);
        return it == ifCodeTypes.end() ? nullptr : &it->second;
    }

    /// 上一次 IF 码类型检测的"各系统可用码类型"签名；不变则跳过重扫
    std::map<char, std::string> ifCodeSig;

    /// 当前历元有卫星的系统集合
    std::set<char> activeSystems;

    /// 上一轮解算的标准化残差和原始残差，用于粗差探测
    std::map<SatID, double> lastWStats;
    std::map<SatID, double> lastResidMag;

    /// 解算完成后计算每颗卫星的 Baarda w 统计量
    virtual void buildResidualMap();

    void buildVelEquation(const SatID &sat, const TypeValueMap &codeList,
                          const PVT &pvt, const Eigen::Vector3d &los,
                          double posWeight,
                          const Variable &dvx, const Variable &dvy,
                          const Variable &dvz, const Variable &dcdt);

private:
    // —— SPP 私有 LSQ 实现细节（H3：PPP 不依赖，下沉为 private） ——
    double rClockDrift{};

    EquSys posEquations{};
    EquSys velEquations{};
    SolverLSQ posSolver{};
    SolverLSQ velSolver{};

    void linearize(ObsData &obsData, int iter);

    /// 构建单颗卫星的位置观测方程（IF 组合）
    void buildPosEquation(const SatID &sat, const PVT &pvt, const Eigen::Vector3d &los,
                          double rho, double trop,
                          const string &obsType, double obsVal, double weight,
                          const Variable &dx, const Variable &dy,
                          const Variable &dz, const Variable &cdtVar);
};
