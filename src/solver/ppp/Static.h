#pragma once

#include "SPP.h"
#include "GnssEKF.h"
#include "core/obs/CycleSlip.h"
#include <Eigen/Eigen>
#include <map>
#include <set>

class PPPStatic : public SPP {
public:
    PPPStatic() {
        mRequirePhaseForIF = true;
    }

    ~PPPStatic() override;

    /// 精密产品与辅助数据：SP3 / CLK / ATX / OSB / ERP / TRO / ATT / IONEX
    bool processEpoch(ObsData &obs) override;

    /// 对流层 ZTD 估计开关（σ 口径：initSigma/ztdProcSigma 均为 σ）
    void setTropo(const bool tropo, const double initVal = 0.0, const double initSigma = 0.2,
                  const double ztdProcSigma = 1.0E-3) {
        mHasTropo = tropo;
        mConfig.ztdInitValue = initVal;
        mConfig.ztdInitSigma = initSigma;
        mConfig.ztdProcSigma = ztdProcSigma;
    }

    void applyOptions(const SolverOptions &o) override;

    /// 码偏差是否覆盖了这颗星/这个系统：GLONASS 的标定是逐测站的，站名对不上就交给 IFCB 状态。
    [[nodiscard]] bool osbCovers(const SatID &sat) const { return prod->osb.covers(sat, osbStation()); }
    [[nodiscard]] bool osbCoversSystem(const char sys) const { return prod->osb.coversSystem(sys, osbStation()); }
    [[nodiscard]] std::string osbStation() const { return mCurObs ? mCurObs->station : std::string(); }

    [[nodiscard]] virtual Eigen::Vector3d predictPosition(double dt, const CommonTime &epoch) const;

    [[nodiscard]] virtual Eigen::Vector3d recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst,
                                                                const CommonTime &epoch,
                                                                const SatID &sat) const;

    void buildEquSys() override;

    void addSatEquation(const SatID &sat, const TypeValueMap &tv, bool probe) override;

    /// 首历元锚定参考位置：把观测文件头/接收机自报的近似坐标收进 mRefPos 当作起算点。
    virtual void bindRefPosition();

    void initFilter() override;

    void updateFilter() override;

    void computeSatPos() override;

    void solve() override;

    void publishResult() override;

    /// 从 EKF 读回位置、钟、码偏、模糊度与码/相位残差（`SPP::readback` 的超集）。
    void readback() override;

    void resetFilterState() override;

    /// 固体潮 + 极潮位移：重锚定那段独立做 SPP 时也要扣
    [[nodiscard]] Eigen::Vector3d computeTideDisplacement(const Eigen::Vector3d &xyzEst, const CommonTime &epoch) const;

    bool mReanchorEveryEpoch = false; // 每历元重锚定
    double mMotionThresh = 0.0;
    double mSppVsPredThresh = 300.0;
    bool mApplyWindup = true; // 相位缠绕总开关

    bool mApplySolidTide = true; // 固体潮

    bool mUseCycleSlip = true;
    double mGfSlipMult = 3.0; // GF 周跳阈值倍数（×单周跳阶跃）

    bool mHasTropo = true; // 估计 ZTD
    bool mUseTroGrad = true; // TRO 水平梯度固定改正开关（默认开；关=退化为仅 ZTD，便于 A/B）
    double mZtdEstimate = 0.0; // 当前 ZTD 状态估计(m)

    bool mPinInitHeight = true;

    bool mUseAR = true; // AR 开关
    double mArRatio = 0.0; // 最近一次 LAMBDA 成功率
    int mArNumFixed = 0; // 最近一次固定卫星数
    long mArAttempts = 0, mArSuccess = 0; // AR 累计计数
    long mDivergeRecovers = 0; // 运动学发散兜底触发次数（诊断）

    // ---- 运动状态与跨类共享的解算上下文 ----
    Eigen::Vector3d mRecvVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d mRefPos = Eigen::Vector3d::Zero();
    bool mFirstEpoch = true;
    // 锚点不可信时（按截止角筛剩下的星不足 4 颗）本历元不筛星。
    bool mSkipElevCutoff = false;

    // ---- 产品与改正量 ----
    std::string mRcvAntenna;
    std::map<SatID, double> mWindupPrev;
    std::map<SatID, double> mWindupAcc;
    std::set<SatID> mFallbackWarned; // 回退广播星历的日志去重

    std::map<SatID, double> mNwSmooth; // MW 宽巷平滑值(周)
    std::map<SatID, double> mNwVar; // MW 宽巷平滑方差(周²)

    [[nodiscard]] Eigen::Vector3d recvEffFor(const SatID &sat) const;

    // 相位缠绕基准
    void recvWindupBasis(const Eigen::Vector3d &recv, Eigen::Vector3d &exr, Eigen::Vector3d &eyr) const;

    /// 天线相位中心变化改正(m)
    [[nodiscard]] double pcvCorrection(const SatID &sat) const;

    bool tryFixAmbiguities();

    /// 首历元精确初位置：只用码方程迭代求解，钉高与码偏按偏好配置。
    [[nodiscard]] std::pair<Eigen::Vector3d, bool> estimateInitialPosition();

    /// 用本历元码/相位预fit 判粗差，先扣同系统中位数。
    void decidePrefitGate();

    /// 预fit 试算结果，供 decidePrefitGate() 决定哪几颗星要剔。
    struct PrefitRec {
        double code = 0.0;
        double phase = 0.0;
        bool freshAmb = false; // 本历元新弧段：相位预fit 已把钟差反算进模糊度
    };

    void resetSlipDetectors();

    /// 对流层 ZTD 状态 + 先验/外部产品约束方程
    void addZTDConstraint();

    [[nodiscard]] double computeIfAmbiguity(const SatID &sat, const TypeValueMap &tv,
                                            double map, double tropoFixed) const;

    void resetAmbiguity(const SatID &sat, const TypeValueMap &tv,
                        double map, double tropoFixed, bool isSlipReset);

    /// 单星前置：筛星、算映射函数与对流层固定项、判周跳、必要时重开模糊度。方程由 addSatEquation 填。
    void prepareSatellite(const SatID &sat, const TypeValueMap &tv);

    void detectCycleSlip(const SatID &sat, const TypeValueMap &tv,
                         bool &slipReset, bool &resetAmb);

    void addObservationEquation(const EquID &eid, double prefit, const Eigen::Vector3d &los,
                                const Weight &w, bool hasAmb, double elev);

    // ---- 滤波器与权重 ----
    VariableDataMap mEpochStates; // 本历元新增状态量，随 initFilter/updateFilter 走

    Weight mWeightCode;
    Weight mWeightPhase;
    int mProcEpoch = 0; // 已处理历元计数

    // ---- 状态回读 ----
    std::map<char, double> mClkBase;
    std::map<char, double> mClkEstimate;
    double mZtdBase = 0.0;
    bool mZtdBaseValid = false;
    std::map<char, double> mCbEstimate; // 每系统 IF 码偏状态估计(m)
    std::map<SatID, double> mIfcbEstimate; // 逐 GLONASS/BDS 星 IFCB 状态估计(m)
    std::map<SatID, double> mAmbEstimate;

    // ---- 本历元派生量 ----
    Eigen::Vector3d mEpochXyz = Eigen::Vector3d::Zero(); // 本历元线性化点
    Eigen::Vector3d mEpochTide = Eigen::Vector3d::Zero(); // 固体潮+极潮位移(ECEF)
    TropoSite mEpochTropo; // 测站对流层快照（每历元一次，所有星共享）

    std::map<SatID, double> mSatMap; // 每星湿映射函数
    std::map<SatID, double> mSatTropoFixed; // 每星对流层固定项(m)
    std::map<SatID, PrefitRec> mPrefitProbe; // 本历元预fit 试算结果
    std::set<SatID> mPrefitBad; // 本历元判为粗差、要剔除的星
    std::set<SatID> mPrefitPhaseBad; // 只剔除相位、码仍可用的星（整周跳变）
    std::set<char> mSysSeen; // 本历元出现过的系统
    std::set<SatID> mCurrentSatSet; // 本历元参与解算的星

    VariableDataMap *mCsData = nullptr; // 本历元待写入的状态量（buildEquSys 的调用方设好）
    CycleSlipEpoch mGfEpoch; // GF 周跳共模裁决簿记

    std::map<SatID, CycleSlip> mSlipDetectors;
    std::set<SatID> mPrevSatSet;
    int mGfCommonMode = 2; // GF 共模抑制阈值
    long slipVerdicts = 0; // 本运行累计 Slip 判定数
    long ambResets = 0; // 本运行累计模糊度重置(newArc/Slip/Gap)数

private:
    /// 钟差先验均值：EKF 已初始化取估计值，否则取 SPP 引导基值。
    [[nodiscard]] double clockPrior(const char sys) const {
        return mFilterInit
                   ? (mClkEstimate.count(sys) ? mClkEstimate.at(sys) : 0.0)
                   : mClkBase.count(sys)
                         ? mClkBase.at(sys)
                         : 0.0;
    }

    /// ZTD 先验均值：EKF 已初始化取估计值，否则取 SPP 引导基值/配置初值。
    [[nodiscard]] double ztdPrior() const {
        return mFilterInit ? mZtdEstimate : mZtdBaseValid ? mZtdBase : mConfig.ztdInitValue;
    }
};
