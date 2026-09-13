#pragma once

#include "SPP.h"
#include "Sp3OrbitReader.h"
#include "ClkReader.h"
#include "GnssEKF.h"
#include "FreqCombo.h"
#include "AntxReader.h"
#include "OsbBias.h"
#include "CycleSlip.h"
#include <Eigen/Eigen>

class PPP : public SPP {
public:
    PPP() { mRequirePhaseForIF = true; }

    ~PPP() override = default;

    bool loadSp3(const std::string &path);

    bool loadClk(const std::string &path);

    bool loadAtx(const std::string &path);

    bool loadOsb(const std::string &path);

    virtual bool process(ObsData &obs);


    void setTropo(const bool tropo, const double initVal = 0.0, const double initVar = 0.04, const double ztdQ = 1.0E-6) {
        mHasTropo = tropo;
        mZtdInitValue = initVal;
        mZtdInitVar = initVar;
        mZtdProcNoise = ztdQ;
    }

    void setKinematic(const bool on) {
        mKinematic = on;
        mPosNoise = on ? 100.0 : 0.0;
    }

    bool mKinematic = false;
    bool mApplySatPCO = true; // 卫星天线 PCO
    bool mApplyRcvPCO = true; // 接收机天线 PCO
    bool mApplyPcv = true;    // 相位中心变化 PCV（接收机 + 卫星）
    bool mApplyWindup = true; // 相位缠绕
    bool mApplySolidTide = true; // 固体潮

    double mArRatioThreshold = 3.0; // LAMBDA 成功率阈值
    double mArSigGate = 0.5; // 参与固定的模糊度标准差上限(m)

    double mFirstEpochPosVar = 0.0;


    // 接收机天线/参考点常值 ECEF 偏移自标定
    bool mEstimateConstOffset = false;
    Vector3d mConstOffset = Vector3d::Zero();

    double mPosNoise = 0.0;
    double mAmbNoise = 1.0E-8;
    double mClockNoise = 0.0;
    double mSigmaPhase = 0.005; // IF 相位量测噪声

    bool mHasTropo = true; // 估计 ZTD
    double mZtdInitValue = 0.0; // ZTD 初始均值
    double mZtdInitVar = 0.04; // ZTD 初始方差
    double mZtdProcNoise = 1.0E-6; // ZTD 随机游走过程噪声

    bool mFirstEpochSPP = false;

    bool mUseAR = false; // AR 开关
protected:
    virtual Vector3d recvAntennaOffsetECEF(const Vector3d &xyzEst, const CommonTime &epoch) const;

    virtual void recvWindupBasis(const SatID &sat, const Vector3d &recv,
                                 const CommonTime &epoch, Vector3d &exr, Vector3d &eyr) const;

    virtual Vector3d predictPosition(double dt, const CommonTime &epoch) const;


    virtual void afterMeasUpdate(ObsData &obsData);

    bool tryFixAmbiguities(ObsData &obsData);

    void computeSatPos(ObsData &obsData) override;

    void solve(ObsData &obsData) override;

    virtual void buildEquSys(ObsData &obsData, const Vector3d &xyzEst, VariableDataMap &csData, bool firstIter);

    virtual void bootstrapFirstEpoch(ObsData &obsData);

    void applyPriorClocks();

    Vector3d currentEstimatedPosition() const;

    void readback(const ObsData &obsData) override;

    /// 天线相位中心变化改正(m)：接收机 PCV 由本地天顶距+方位角查表，卫星 PCV 由星下点角查表
    double pcvCorrection(const SatID &sat, const FreqCombo &def, const PVT &pvt,
                         const Vector3d &xyzEst, const Vector3d &tideDisp,
                         const Vector3d &recvPCOe, double elev, double azim) const;

    void addSatelliteEquations(const SatID &sat, const FreqCombo &def, const TypeValueMap &tv,
                               const PVT &pvt, const Vector3d &xyzEst, double elev, double azim, double map,
                               const Variable &vclk, const Variable &vdx, const Variable &vdy,
                               const Variable &vdz, const Variable &vamb,
                               const Vector3d &tideDisp, const Vector3d &recvPCOe,
                               const CommonTime &epoch,
                               const Variable *vox = nullptr, const Variable *voy = nullptr,
                               const Variable *voz = nullptr);

    void resetSlipDetectors(const ObsData &obsData);

    std::string mRcvAntenna;
    std::map<SatID, double> mWindupPrev;
    std::map<SatID, double> mWindupAcc;

    Sp3OrbitReader mSp3;
    ClkReader mClk;
    AntxReader mAntx;
    OsbBias mOsb;

    GnssEKF ekf;
    EquSys mEq;
    VariableSet mVarSet; // EKF 状态参数集
    bool mEkfInit = false; // 首历元引导完成
    bool mFirstEpoch = true;
    Vector3d mRefPos = Vector3d::Zero();
    int mProcEpoch = 0; // 已处理历元计数
    static constexpr int GATE_WARMUP = 30; // 前 N 历元钟差/模糊度未收敛，禁用粗差门限

    int mMinSatsForUpdate = 0;
    bool mFilterFrozen = false;

    std::map<char, double> mClkBase;
    std::map<char, double> mClkEstimate;
    std::map<SatID, double> mAmbEstimate;
    double mZtdEstimate = 0.0;

    std::map<SatID, CycleSlip> mSlipDetectors;
    std::set<SatID> mPrevSatSet;
    bool mUseCycleSlip = true;
    bool mForceAmbReset = false;
    CommonTime mLastEpochTime; // 上一已处理历元时刻
    double mNominalInterval = -1.0; // 标称历元间隔(自适应学习，秒)
    bool mHasLastEpoch = false;
    bool mGapThisEpoch = false; // 观测中断恢复首历元标志
    long slipVerdicts = 0; // 本运行累计 Slip 判定数
    long ambResets = 0; // 本运行累计模糊度重置(newArc/Slip/Gap)数
    std::map<SatID, double> mAmbFixed; // 固定后的 IF 模糊度(m)
    double mArRatio = 0.0; // 最近一次 LAMBDA 成功率
    int mArNumFixed = 0; // 最近一次固定卫星数
    long mArAttempts = 0, mArSuccess = 0; // AR 累计计数
    // MW 宽巷平滑(跨历元 EMA)
    std::map<SatID, double> mNwSmooth;
    std::map<SatID, double> mNwVar;
};
