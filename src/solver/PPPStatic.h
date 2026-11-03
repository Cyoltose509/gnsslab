#pragma once

#include "product/SP3OrbitReader.h"
#include "product/CLKReader.h"
#include "product/ATXReader.h"
#include "product/OSBBias.h"
#include "product/ERPReader.h"
#include "product/TROReader.h"
#include "product/METReader.h"
#include "product/ATTReader.h"
#include "product/IONEXReader.h"
#include "SPP.h"
#include "GnssEKF.h"
#include "CycleSlip.h"
#include <vector>
#include <Eigen/Eigen>
#include <map>
#include <set>

class PPPStatic : public SPP {
public:
    PPPStatic() {
        mRequirePhaseForIF = true;
    }

    ~PPPStatic() override;

    bool loadSP3(const std::string &path);

    bool loadCLK(const std::string &path, const std::set<char> &allowedSys = {});

    bool loadATX(const std::string &path);

    bool loadOSB(const std::string &path);

    bool loadERP(const std::string &path);

    bool loadTRO(const std::string &path);

    /// 气象文件（RINEX3 *_MM.rnx）
    bool loadMET(const std::string &path);

    /// ORBEX 姿态四元数（*.OBX）
    bool loadATT(const std::string &path);

    bool loadIONEX(const std::string &path);

    bool processEpoch(ObsData &obs) override;

    /// 对流层 ZTD 估计开关
    void setTropo(const bool tropo, const double initVal = 0.0, const double initVar = 0.04, const double ztdQ = 1.0E-6) {
        mHasTropo = tropo;
        mConfig.ztdInitValue = initVal;
        mConfig.ztdInitVar = initVar;
        mConfig.ztdProcNoise = ztdQ;
    }

    /// 观测噪声配置
    void setMeasNoise(const double sigCode, const double sigPhase) {
        mConfig.sigIFCode = sigCode;
        mConfig.sigmaPhase = sigPhase;
    }

    void setFirstEpochPosVar(const double v) { mConfig.firstEpochPosVar = v; }

    /// 运动学噪声/先验旋钮
    void setKinematicNoise(const double clockInitVar = 1.0E8,
                           const double firstEpochPosVar = 1.0,
                           const double ambNoise = 1.0E-7,
                           const double sigmaPhase = 0.003,
                           const int minSatsForUpdate = 4) {
        mConfig.firstEpochPosVar = firstEpochPosVar;
        mConfig.ambNoise = ambNoise;
        mConfig.sigmaPhase = sigmaPhase;
        mConfig.minSatsForUpdate = minSatsForUpdate;
        ekf.clockInitVar = clockInitVar;
    }

    void setPosProcNoise(const double v) { mConfig.posNoise = v; }

    void setKeepAmbMaxGap(const double sec) { mConfig.keepAmbMaxGap = sec; }

    void setPosVarFloor(const double v) { mConfig.posVarFloor = v; }

    void applyOptions(const SolverOptions &o) override;

    void setZTDProcNoise(const double q) { mConfig.ztdProcNoise = q; }

    /// ZTD 先验 sigma 旋钮
    void setZTDPriorSigma(const double s) { mConfig.ztdPriorSigma = s; }

    /// TRO 产品 ZTD 约束 sigma(m)
    void setTROZTDSigma(const double s) { mConfig.troZtdSigma = s; }

    void setAmbInitVar(const double v) { mConfig.ambInitVar = v; }

    void setAmbProcNoise(const double q) { mConfig.ambNoise = q; }

    void setCodeBiasProcNoise(const double q, const double initVar) {
        mConfig.codeBiasProcNoise = q;
        mConfig.codeBiasInitVar = initVar;
    }

    void setGloIfcbProcNoise(const double q, const double initVar) {
        mConfig.gloIfcbProcNoise = q;
        mConfig.gloIfcbInitVar = initVar;
    }

    [[nodiscard]] bool osbCovers(const SatID &sat) const { return mOsbSatSet.count(sat) > 0; }
    [[nodiscard]] bool osbCoversSystem(const char sys) const {
        return std::any_of(mOsbSatSet.begin(), mOsbSatSet.end(), [sys](const SatID &s) { return s.system == sys; });
    }


    bool mKinematic = false;
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
protected:
    // 接收机速度(逐历元有限差分)，驱动运动学风缠绕基
    Eigen::Vector3d mRecvVel = Eigen::Vector3d::Zero();

    [[nodiscard]] virtual Eigen::Vector3d recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst, const CommonTime &epoch) const;

    // 相位缠绕基准
    void recvWindupBasis(const Eigen::Vector3d &recv, Eigen::Vector3d &exr, Eigen::Vector3d &eyr) const;


    [[nodiscard]] virtual Eigen::Vector3d predictPosition(double dt, const CommonTime &epoch) const;

    bool tryFixAmbiguities(ObsData &obsData);

    void computeSatPos(ObsData &obsData) override;

    void solve(ObsData &obsData) override;

    [[nodiscard]] std::pair<Eigen::Vector3d, bool> solveInitialPosition(ObsData &obsData);

    virtual void buildEquSys(ObsData &obsData, const Eigen::Vector3d &xyzEst, VariableDataMap &csData, bool firstIter);

    /// 首历元引导
    void initFilter(ObsData &obsData, VariableDataMap &csData);

    /// 常规历元
    void updateFilter(ObsData &obsData, VariableDataMap &csData);

    virtual void bootstrapFirstEpoch(ObsData &obsData);

    void applyPriorClocks();

    [[nodiscard]] Eigen::Vector3d currentEstimatedPosition() const;

    void readback(const ObsData &obsData) override;

    /// 天线相位中心变化改正(m)
    [[nodiscard]] double pcvCorrection(const SatID &sat) const;

    void addSatelliteEquations(const SatID &sat, const TypeValueMap &tv,
                              double mapW, double tropoFixed);

    void resetSlipDetectors(const ObsData &obsData);

    /// 对流层 ZTD 状态 + 先验/外部产品约束方程
    void addZTDConstraint();

    [[nodiscard]] Eigen::Vector3d computeTideDisplacement(const Eigen::Vector3d &xyzEst, const CommonTime &epoch) const; // 固体潮 + 极潮

    [[nodiscard]] double computeIfAmbiguity(const SatID &sat, const TypeValueMap &tv,
                              double map, double tropoFixed) const;

    void resetAmbiguity(const SatID &sat, const TypeValueMap &tv,
                        double map, double tropoFixed, bool isSlipReset);

    void processSatellite(const SatID &sat, const TypeValueMap &tv);

    void detectCycleSlip(const SatID &sat, const TypeValueMap &tv,
                         bool &slipReset, bool &resetAmb);

    void addObservationEquation(const EquID &eid, double prefit, const Eigen::Vector3d &los,
                                const Weight &w, bool hasAmb, double elev);

    void computeDops();

    std::string mRcvAntenna;
    std::map<SatID, double> mWindupPrev;
    std::map<SatID, double> mWindupAcc;
    std::set<SatID> mFallbackWarned; // 回退广播星历的日志去重

    std::map<SatID, double> mNwSmooth;  // MW 宽巷平滑值(周)
    std::map<SatID, double> mNwVar;     // MW 宽巷平滑方差(周²)


    SP3OrbitReader mSp3;
    CLKReader mClk;
    ATXReader mAntx;
    OSBBias mOsb;
    std::set<SatID> mOsbSatSet; // 外部 OSB 覆盖的卫星集合
    ERPReader mErp;
    TROReader mTro;
    METReader mMet;
    ATTReader mAtt;
    IONEXReader mIonex;

    GnssEKF ekf;
    EquSys mEq;
    VariableSet mVarSet; // EKF 状态参数集

    Weight mWeightCode;
    Weight mWeightPhase;
    bool mWeightInit = false;
    bool mEkfInit = false; // 首历元引导完成
    bool mFirstEpoch = true;
    Eigen::Vector3d mRefPos = Eigen::Vector3d::Zero();
    int mProcEpoch = 0; // 已处理历元计数
    static constexpr int GATE_WARMUP = 30; // 前 N 历元钟差/模糊度未收敛，禁用粗差门限
    bool mFilterFrozen = false;

    std::map<char, double> mClkBase;
    std::map<char, double> mClkEstimate;
    double mZtdBase = 0.0;
    bool mZtdBaseValid = false;
    std::map<char, double> mCbEstimate;   // 每系统 IF 码偏状态估计(m)
    std::map<SatID, double> mIfcbEstimate; // 逐 GLONASS 星 IFCB 状态估计(m)
    std::map<SatID, double> mAmbEstimate;

    const ObsData *mCurObs = nullptr; // 当前历元观测
    Eigen::Vector3d mEpochXyz = Eigen::Vector3d::Zero(); // 本历元线性化点
    Eigen::Vector3d mEpochTide = Eigen::Vector3d::Zero(); // 固体潮+极潮位移(ECEF)
    Eigen::Vector3d mEpochPcoE = Eigen::Vector3d::Zero(); // 接收机 PCO（ECEF）
    TropoSite mEpochTropo; // 测站对流层快照（每历元一次，所有星共享）
    bool mEpochFirstIter = true; // 是否首次迭代

    std::map<SatID, double> mSatMap; // 每星湿映射函数
    std::map<SatID, double> mSatTropoFixed; // 每星对流层固定项(m)
    std::set<char> mSysSeen; // 本历元出现过的系统
    std::set<SatID> mCurrentSatSet; // 本历元参与解算的星

    // 本历元是否启用"公共接收机钟(cdt)+每系统 ISB(cdt2~4)"状态模型
    bool mCommonClockActive = false;

    [[nodiscard]] bool wantCommonClock(const ObsData &obs) const;
    VariableDataMap *mCsData = nullptr; // 本历元待写入的状态量（由 solve 传入）
    CycleSlipEpoch mGfEpoch; // GF 周跳共模裁决簿记
    std::set<SatID> mGfSlipSats; // 本历元 GF 判跳的星

    std::map<SatID, CycleSlip> mSlipDetectors;
    std::set<SatID> mPrevSatSet;
    int mGfCommonMode = 2; // GF 共模抑制阈值
    bool mForceAmbReset = false;
    CommonTime mLastEpochTime; // 上一已处理历元时刻
    double mNominalInterval = -1.0; // 标称历元间隔(自适应学习，秒)
    bool mHasLastEpoch = false;
    bool mGapThisEpoch = false; // 观测中断恢复首历元标志
    long slipVerdicts = 0; // 本运行累计 Slip 判定数
    long ambResets = 0; // 本运行累计模糊度重置(newArc/Slip/Gap)数

private:
    /// 钟差先验均值：EKF 已初始化取估计值，否则取 SPP 引导基值。
    [[nodiscard]] double clockPrior(const char sys) const {
        return mEkfInit
            ? (mClkEstimate.count(sys) ? mClkEstimate.at(sys) : 0.0)
            : mClkBase.count(sys) ? mClkBase.at(sys) : 0.0;
    }
    /// ZTD 先验均值：EKF 已初始化取估计值，否则取 SPP 引导基值/配置初值。
    [[nodiscard]] double ztdPrior() const {
        return mEkfInit ? mZtdEstimate : mZtdBaseValid ? mZtdBase : mConfig.ztdInitValue;
    }
};
