#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "Ephemeris.h"
#include "EphemerisTable.h"
#include "RinexNavStore.h"
#include "FreqCombo.h"
#include "Weight.h"
#include "SppConfig.h"
#include "SolverOptions.h"
#include <Eigen/Eigen>
#include <set>


using IFCodeTypes = std::map<char, FreqCombo>;

class SPP {
protected:
    // 配置结构体定义在 `SppConfig.h`（纯数据、含默认值）：`SolverOptions` 直接持有它，
    // 所以此处用的就是同一个类型名，不另起别名。
    SppConfig mConfig;
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

    /// 每历元一次的测站对流层几何：NMF/GMF 映射需要纬度/经度/高程与时间，逐星重算没有必要
    struct TropoSite {
        bool valid = false;
        double latRad = 0.0, lonRad = 0.0, hgt = 0.0;
        double doy = 0.0, mjd = 0.0;
    };
    TropoSite mTropoSite;

    /// 由当前线性化点与历元刷新 mTropoSite（位置无效则 valid=false → 该历元不改正对流层）
    void updateTropoSite(const Eigen::Vector3d &xyzEcef, const CommonTime &epoch);

    /// 对流层斜路径延迟(m)：天顶延迟用 Hopfield 干/湿模型，映射函数由 mConfig.tropoMap 选
    [[nodiscard]] double tropoDelay(double elevRad) const;

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
