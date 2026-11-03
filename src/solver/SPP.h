#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "Ephemeris.h"
#include "EphemerisTable.h"
#include "rinex/RINEXNavStore.h"
#include "FreqCombo.h"
#include "Weight.h"
#include "SPPConfig.h"
#include "SolverOptions.h"
#include "TropoGrid.h"
#include "TropoSite.h"
#include <Eigen/Eigen>
#include <set>
#include <vector>


using IFCodeTypes = std::map<char, FreqCombo>;

class SPP {
protected:
    SPPConfig mConfig;
    Weight mWeight;
    bool mWeightInit = false;

public:
    bool loadBRDC(const std::string &path);

    /// 外部对流层格网
    bool loadVMF3(const std::vector<std::string> &paths) { return mTropoGrid.loadVMF3(paths); }
    bool loadGPT3(const std::string &path) { return mTropoGrid.loadGPT3(path); }

    /// 所选对流层模型的格网是否已就位
    [[nodiscard]] bool tropoGridReady() const { return mTropoGrid.readyFor(mConfig.tropoMap); }

    void detectIFCombinations(const ObsData &obsData, const IFCodeTypes &defaultTypes = {});

    void setCutoffElevDeg(const double deg) { mConfig.cutoffElevRad = deg * DEG_TO_RAD; }

    /// ISB 估计模式
    void setISBProcNoise(const double proc, const double init) {
        mConfig.isbProcNoise = proc;
        mConfig.isbInitVar = init;
    }

    void setEnabledSystems(const std::set<char> &s) { mConfig.enabledSystems = s; }

    /// 统一参数注入入口
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
    [[nodiscard]] double getClockBias(const char sys) const {
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


    EphemerisTable ephTable;
    /// 广播电文健康标志
    bool ephUnhealthy(const SatID &sat, const Ephemeris *eph);

protected:
    std::set<SatID> mHealthWarned; // 健康剔除日志去重
    bool isRover = true;
    bool mRequirePhaseForIF = false;

    std::map<char, double> rClockBias{};

    /// 上一历元各系统终值钟差 [m]
    std::map<char, double> mPrevClockBias{};

    /// 系统→钟差参数映射；多系统各自独立钟差=吸收 ISB。
    static inline std::map<char, Parameter> sysCdtParam = {
        {'G', Parameter::cdt},
        {'C', Parameter::cdt2},
        {'R', Parameter::cdt3},
        {'E', Parameter::cdt4},
    };

    IFCodeTypes ifCodeTypes{};

    std::map<SatID, FreqCombo> mGloCombo;

    /// 取卫星的 IF 频率组合
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
    TropoGrid mTropoGrid;

private:
    double rClockDrift{};
    TropoSite mTropoSite;

    /// 上一次成功解算的位置 [m]
    Eigen::Vector3d mLastXyz{0, 0, 0};


    [[nodiscard]] Eigen::Vector3d roughInitialXyz(const ObsData &obsData) const;

    void updateTropoSite(const Eigen::Vector3d &xyzEcef, const CommonTime &epoch);

    [[nodiscard]] double tropoDelay(double elevRad) const;

    EquSys posEquations{};
    EquSys velEquations{};
    SolverLSQ posSolver{};
    SolverLSQ velSolver{};

    void linearize(ObsData &obsData, int iter);

    /// 构建单颗卫星的位置观测方程（IF 组合）
    void buildPosEquation(const SatID &sat, const PVT &pvt, const Eigen::Vector3d &los,
                          double rho, double trop,
                          const std::string &obsType, double obsVal, double weight,
                          const Variable &dx, const Variable &dy,
                          const Variable &dz, const Variable &cdtVar);
};
