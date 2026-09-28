#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "Ephemeris.h"
#include "EphemerisTable.h"
#include "RinexNavStore.h"   // loadBrdc：广播星历加载（与 PPP 一致，类内加载）
#include "FreqCombo.h"
#include "Weight.h"
#include <Eigen/Eigen>
#include <set>

// SPP 的 IF 组合类型
using IFCodeTypes = std::map<char, FreqCombo>;

class SPP {
protected:
    struct Config {
        // —— 基础段：SPP 与 PPP 都读 ——
        double cutoffElevRad = 10.0 * PI / 180.0; // 截止高度角(rad)
        std::set<char> enabledSystems{}; // 参与解算的星座集合（空=不筛选，全部参与）
        double sigIFCode = 0.3; // IF 码观测噪声基准
        double convEps = 1e-4; // 收敛阈值(m)
        int maxIter = 10; // 最大迭代
        int minCheckIter = 2; // 起始收敛检查的迭代
        // —— PPP / LEO 专有段：SPP 不读 ——
        double sigmaPhase = 0.005; // IF 相位量测噪声(PPP 默认)
        double troZtdSigma = 0.05; // TRO ZTD 先验 sigma
        double arRatioThreshold = 3.0; // LAMBDA 成功率阈值
        double arSigGate = 0.5; // 固定模糊度 sigma 上限(m)
        double posNoise = 0.0; // 位置过程噪声(静态0)
        double ambNoise = 1.0E-8; // 模糊度过程噪声
        double clockNoise = 0.0; // 钟差过程噪声
        double ztdInitValue = 0.0; // ZTD 初值
        double ztdInitVar = 0.0144; // ZTD 初方差
        double ztdProcNoise = 1.0E-8; // ZTD 随机游走噪声
        double ztdPriorSigma = 0.30; // ZWD 先验 sigma(m)
        double firstEpochPosVar = 0.0; // 首历元位置先验方差
        int minSatsForUpdate = 0; // 滤波更新最少卫星(PPP 默认0=不限)
        int maxReanchorIter = 8; // LEO 重锚定最大迭代
        double sppResidualThr = 10.0; // SPP 后验残差 RMS 阈值(m)
    };

    Config mConfig;

    // 高度角随机模型（σ²=a²+(b/sinE)²），与 RTK/PPP 共用 Weight 类。
    // 由 mConfig.sigIFCode / cutoffElevRad 在解算开始时同步，避免散落裸公式。
    Weight mWeight;
    bool mWeightInit = false; // mWeight 首轮解算惰性初始化一次，迭代内不再重建

public:
    void setEphemeris(const SatEphemerisMap &ephs) {
        ephMap = ephs;
    }

    bool loadBrdc(const std::string &path);

    void detectIFCombinations(const ObsData &obsData, const IFCodeTypes &defaultTypes = {});

    void setCutoffElevDeg(const double deg) { mConfig.cutoffElevRad = deg * PI / 180.0; }

    /// ZTD 先验 sigma 旋钮（诊断用：调到很大即关闭 ZTD 状态约束，仅留卫星观测估计）
    void setZtdPriorSigma(const double s) { mConfig.ztdPriorSigma = s; }

    void setEnabledSystems(const std::set<char> &s) { mConfig.enabledSystems = s; }

    /// 从一批历元中用 SPP 估计一个近似坐标（冷启动：线性化初值取地心）。
    /// 逐历元尝试，首个解出 |xyz| > 1e6 m 的即采纳；全部失败返回 false。
    /// 供 RTK 基准站/流动站求线性化初值使用（原本由 GUI 内联实现）。
    static bool estimateApproxPosition(const std::vector<ObsData> &epochs,
                                       const std::vector<EphemerisTable> &ephs,
                                       const std::set<char> &systems,
                                       double cutoffElevDeg, Vector3d &out);

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

    Vector3d xyz{0, 0, 0};
    Vector3d vel{0, 0, 0};
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

    /// 系统→钟差参数映射（仅 GPS + BDS）；LEO 经继承链访问，故置于 protected
    static inline std::map<char, Parameter> sysCdtParam = {
        {'G', Parameter::cdt},
        {'C', Parameter::cdt2},
    };

    SatEphemerisMap ephMap{};

    IFCodeTypes ifCodeTypes{}; // 系统 → FreqCombo（码名 + 频率/系数/波长，及 MW/GF 组合）

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
                          const PVT &pvt, const Vector3d &los,
                          double elev, double posWeight,
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
    void buildPosEquation(const SatID &sat, const PVT &pvt, const Vector3d &los,
                          double rho, double trop,
                          const string &obsType, double obsVal, double weight,
                          const Variable &dx, const Variable &dy,
                          const Variable &dz, const Variable &cdtVar);
};
