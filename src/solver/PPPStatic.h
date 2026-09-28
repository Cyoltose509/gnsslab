#pragma once

#include "Sp3OrbitReader.h"
#include "ClkReader.h"
#include "AntxReader.h"
#include "OsbBias.h"
#include "ErpReader.h"
#include "TroReader.h"
#include "IonexReader.h"
#include "SPP.h"
#include "GnssEKF.h"
#include "FreqCombo.h"
#include "CycleSlip.h"
#include <Eigen/Eigen>
#include <map>
#include <set>

class PPPStatic : public SPP {
public:
    // 测站对流层参数（每历元由 buildEquSys 算一次，所有星共享）
    struct TropoParams {
        double latDeg = 0.0; // 测站纬度 [deg]
        double hgt = 0.0;    // 测站高 [m]
        double doy = 0.0;    // 年积日（NMF 湿映射需要）
        double zhd = 0.0;    // Saastamoinen 天顶干延迟 [m]
    };

    PPPStatic() { mRequirePhaseForIF = true; }

    ~PPPStatic() override = default;

    bool loadSp3(const std::string &path);

    bool loadClk(const std::string &path);

    bool loadAtx(const std::string &path);

    bool loadOsb(const std::string &path);

    /// 加载**可选**的 ERP(地球自转参数)产品。失败/不加载都只返回 false，解算照常进行。
    bool loadErp(const std::string &path);

    /// 加载**可选**的对流层 TRO 产品(逐时 ZTD/梯度)。失败/不加载都只返回 false，解算照常进行。
    bool loadTro(const std::string &path);

    /// 加载**可选**的 IONEX GIM 电离层产品。失败/不加载都只返回 false，解算照常进行。
    bool loadIonex(const std::string &path);

    /// 与 SPP 同构的统一驱动入口；LEO 直接继承，不再另写一份。
    bool processEpoch(ObsData &obs) override;

    /// 对流层 ZTD 估计开关：打开即把 ZTD 作为状态量估计，初值/初方差/过程噪声一并给定。
    void setTropo(const bool tropo, const double initVal = 0.0, const double initVar = 0.04, const double ztdQ = 1.0E-6) {
        mHasTropo = tropo;
        mConfig.ztdInitValue = initVal;
        mConfig.ztdInitVar = initVar;
        mConfig.ztdProcNoise = ztdQ;
    }

    /// 观测噪声配置：IF 组合后的码/相位 sigma。码相权比直接决定 PPP 收敛速度
    /// （相位定形变、码定绝对尺度），按数据质量/接收机类型需要可调。
    void setMeasNoise(const double sigCode, const double sigPhase) {
        mConfig.sigIFCode = sigCode;
        mConfig.sigmaPhase = sigPhase;
    }

    /// 运动学噪声/先验旋钮（这些是 PPP 运动学通用参数，故挂在基类 public 接口上，
    /// 子类 PPPKinematic/LEO 构造函数里写死默认值，harness 用环境变量覆盖扫参）。
    /// 普通地面静态解算一般不调此方法，沿用 setMeasNoise/setTropo 等配置。
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

    /// 位置过程噪声(m^2/epoch)旋钮：运动学重线性化下，位置状态作为"紧随机常数"跨历元累积相位/模糊度
    /// （对标 rtkpost dynamics-off 的 cm 级跨弧积分），故运动学默认应取很小的值；保留为独立接口便于无界面 harness 扫参。
    void setPosProcNoise(const double v) { mConfig.posNoise = v; }

    // 静态/运动学的区分不再靠运行时开关，而是由派生类决定：PPPStatic=纯静态；
    // PPPKinematic(:PPPStatic) 在构造函数里固化运动学状态；LEO(:PPPKinematic) 再叠加星载特有项。

    bool mKinematic = false;
    bool mReanchor = false; // 运动学重锚定开关（随 setKinematic 一并开启；静态无需）
    bool mReanchorEveryEpoch = true; // 每历元重锚定(L=真，LEO/高速必开)；地面动态置 false ⇒ 仅首历元重锚、之后靠后验连续性(rtkpost dynamics-off 语义)
    // 平滑运动一致性阈值(m；0 = 关闭)：轨道运动应满足「上一历元后验 + 速度×dt」外推，若解偏离
    // 外推超过本阈值即判废本历元（脏解不输出）。动因：实测低冗余窗口(仅 4~5 颗双频星)下无冗余
    // 去挡单个不一致观测，误差在径向/沿迹爆到数百米(nsat=4 时 p95=19m/max=5km；nsat≥7 时 1.38m/10m)。
    // 只做"检验"不动线性化点，故对常态历元零影响；|v|>1km/s 与上一历元可信门限保证地面站不受影响。
    double mMotionThresh = 0.0;
    // code-SPP 重锚点与「运动外推点」的差异阈值(m；>1e8 = 关闭)。超出即判定码级重锚点不可用，
    // 改用外推点作本历元线性化点/先验（详见 LEO::predictPosition）。
    // 扫参(整日 Swarm-C)最优 300m：未裁剪 RMS(all) 25.53→2.63 m(9.7×，km 级尖刺基本消灭)，
    // median 0.728 / p95 1.49 / tail200 1.937 均不变，稳健 RMS 1.071→1.116(+4%)。
    // 阈值 ≥700 反而更差：只在极端分歧时才替换，此时外推点也已不可靠(n 掉到 5.1 万、尾段 1600m)。
    double mSppVsPredThresh = 300.0;
    bool mApplySatPCO = true; // 卫星天线 PCO
    bool mApplyRcvPCO = true; // 接收机天线 PCO
    bool mApplyPcv = true;    // 相位中心变化 PCV（接收机 + 卫星）
    bool mApplyWindup = true; // 相位缠绕
    bool mApplySolidTide = true; // 固体潮
    bool mApplyPoleTide = false;

    bool mUseTroZtd = false;

    bool mApplyHigherOrderIono = false;

    // 周跳检测（MW 码相组合 + GF 几何无关）总开关。MW 含码，会被码野值污染而误判周跳，
    // 低冗余/码质量差的数据集可关闭以对照（诊断用）。
    bool mUseCycleSlip = true;

    // 接收机天线/参考点常值 ECEF 偏移自标定。
    // **默认关闭**：它与位置的方程系数同为 LOS（见 addSatelliteEquations），静态 PPP 下
    // 位置亦为常值 ⇒ 6 个状态仅 3 个自由度，秩亏 3 阶、法方程奇异，改正量被任意拆分。
    // 仅当位置随时间变化（动态/LEO）时二者才可区分，故随 setKinematic 一同启停。
    // 实测（HKWS）：关闭后 tail RMS 0.2477→0.2358，末历元 N 分量 −5.1cm→+2.3mm。
    bool mEstimateConstOffset = false;
    Vector3d mConstOffset = Vector3d::Zero();

    bool mHasTropo = true; // 估计 ZTD
    double mZtdEstimate = 0.0;      // 当前 ZTD 状态估计(m)

    bool mFirstEpochSPP = false;

    bool mUseAR = false; // AR 开关
    // AR 诊断量（public，供无界面 harness 读取；声明位置不变，不影响类布局/ABI）
    double mArRatio = 0.0; // 最近一次 LAMBDA 成功率
    int mArNumFixed = 0;    // 最近一次固定卫星数
    long mArAttempts = 0, mArSuccess = 0; // AR 累计计数
    long mDivergeRecovers = 0;            // 运动学发散兜底触发次数（诊断）
    Vector3d linXyz() const { return mEpochXyz; }    // 本历元线性化点（诊断）
    long ambResetCount() const { return ambResets; } // 模糊度重置累计数（诊断）
protected:
    // 接收机速度(逐历元有限差分)，驱动运动学风缠绕基；由 PPPKinematic 每历元刷新，
    // 静态路径恒为 0 ⇒ recvWindupBasis 退回 ENU 基（地面站零回归）。
    Vector3d mRecvVel = Vector3d::Zero();

    virtual Vector3d recvAntennaOffsetECEF(const Vector3d &xyzEst, const CommonTime &epoch) const;

    // 相位缠绕基准：运动时用沿迹/轨道面基、静止时退回 ENU 基。卫星无关，无需 sat/epoch 参数。
    void recvWindupBasis(const Vector3d &recv, Vector3d &exr, Vector3d &eyr) const;

    // 单历元 SPP 重线性化（reanchorSPP）与运动学辅助量已下沉到 PPPKinematic，静态路径不进入。

    virtual Vector3d predictPosition(double dt, const CommonTime &epoch) const;

    // 模糊度固定(AR)：宽巷 EMA 平滑 + 窄巷/LAMBDA 整数搜索 + 一致性闸门 + 回代固定解。
    // 命名对齐 RTK::tryFixAmbiguities。
    bool tryFixAmbiguities(ObsData &obsData);

    void computeSatPos(ObsData &obsData) override;

    void solve(ObsData &obsData) override;

    virtual void buildEquSys(ObsData &obsData, const Vector3d &xyzEst, VariableDataMap &csData, bool firstIter);

    virtual void bootstrapFirstEpoch(ObsData &obsData);

    void applyPriorClocks();

    [[nodiscard]] Vector3d currentEstimatedPosition() const;

    void readback(const ObsData &obsData) override;

    /// 天线相位中心变化改正(m)：接收机 PCV 由本地天顶距+方位角查表，卫星 PCV 由星下点角查表
    [[nodiscard]] double pcvCorrection(const SatID &sat) const;

    void addSatelliteEquations(const SatID &sat, const TypeValueMap &tv,
                               double map, double tropoFixed);

    void resetSlipDetectors(const ObsData &obsData);

    /// 对流层 ZTD 状态 + 先验/外部产品约束方程（从 buildEquSys 抽出，缩短主函数）
    void addZtdConstraint();

    Vector3d computeTideDisplacement(const Vector3d &xyzEst, const CommonTime &epoch) const; // 固体潮 + 极潮

    // IF 组合绝对模糊度(m)：纯计算，供 resetAmbiguity 与首次出现初始化共用
    double computeIfAmbiguity(const SatID &sat, const TypeValueMap &tv,
                              double map) const;

    // 周跳/失锁重置 IF 模糊度；isSlipReset=false 即首次出现初始化
    void resetAmbiguity(const SatID &sat, const TypeValueMap &tv,
                        double map, bool isSlipReset);

    // 单星：周跳检测 + 模糊度初始化/重置 + 建方程（buildEquSys 主循环抽出）
    void processSatellite(const SatID &sat, const TypeValueMap &tv);

    // —— 以下为从主流程抽出的同文件 helper（仅拆函数、不拆类，降低单函数体量与重复）——
    // 单星周跳检测：输出 resetAmb / slipReset
    void detectCycleSlip(const SatID &sat, const TypeValueMap &tv,
                         bool &slipReset, bool &resetAmb);
    // 单星码/相位观测方程（去重：码方程与相位方程共用）
    void addObservationEquation(const EquID &eid, double prefit, const Vector3d &los,
                                const Weight &w, bool hasAmb, double elev);
    void computeDops();         // readback 中 PDOP/HDOP/VDOP 计算

    std::string mRcvAntenna;
    std::map<SatID, double> mWindupPrev;
    std::map<SatID, double> mWindupAcc;

    Sp3OrbitReader mSp3;
    ClkReader mClk;
    AntxReader mAntx;
    OsbBias mOsb;
    ErpReader mErp;
    TroReader mTro;
    IonexReader mIonex;

    GnssEKF ekf;
    EquSys mEq;
    VariableSet mVarSet; // EKF 状态参数集

    // 高度角随机模型（Weight 类）：构造一次、整轮复用，不再逐卫星方程内反复创建/销毁。
    // 码/相位 sigma 不同，各缓存一份；首历元 buildEquSys 惰性初始化。
    Weight mWeightCode;
    Weight mWeightPhase;
    bool mWeightInit = false;
    bool mEkfInit = false; // 首历元引导完成
    bool mFirstEpoch = true;
    Vector3d mRefPos = Vector3d::Zero();
    int mProcEpoch = 0; // 已处理历元计数
    static constexpr int GATE_WARMUP = 30; // 前 N 历元钟差/模糊度未收敛，禁用粗差门限
    bool mFilterFrozen = false;

    std::map<char, double> mClkBase;
    std::map<char, double> mClkEstimate;
    std::map<SatID, double> mAmbEstimate;

    // —— 每历元派生量：由 buildEquSys 填充，per-sat 函数直接读（与 satPVTRecTime/satElevData 同一模式）——
    const ObsData *mCurObs = nullptr;   // 当前历元观测（取 station / epoch）
    Vector3d mEpochXyz = Vector3d::Zero();  // 本历元线性化点
    Vector3d mEpochTide = Vector3d::Zero(); // 固体潮+极潮位移
    Vector3d mEpochPcoE = Vector3d::Zero(); // 接收机 PCO（ECEF）
    TropoParams mEpochTropo;                // 测站对流层参数（含 ZHD）
    bool mEpochFirstIter = true;            // 是否首次迭代

    // —— 遍历累加器：buildEquSys 单次遍历内由 per-sat 函数写入 ——
    std::map<SatID, double> mSatMap;   // 每星湿映射函数（与 satElevData 同类的 per-sat 缓存）
    std::set<char> mSysSeen;           // 本历元出现过的系统
    std::set<SatID> mCurrentSatSet;    // 本历元参与解算的星
    VariableDataMap *mCsData = nullptr;// 本历元待写入的状态量（由 solve 传入）
    CycleSlipEpoch mGfEpoch;           // GF 周跳共模裁决簿记
    std::set<SatID> mGfSlipSats;       // 本历元 GF 判跳的星

    std::map<SatID, CycleSlip> mSlipDetectors;
    std::set<SatID> mPrevSatSet;
    int mGfCommonMode = 2;    // GF 共模抑制阈值
    bool mForceAmbReset = false;
    CommonTime mLastEpochTime; // 上一已处理历元时刻
    double mNominalInterval = -1.0; // 标称历元间隔(自适应学习，秒)
    bool mHasLastEpoch = false;
    bool mGapThisEpoch = false; // 观测中断恢复首历元标志
    long slipVerdicts = 0; // 本运行累计 Slip 判定数
    long ambResets = 0; // 本运行累计模糊度重置(newArc/Slip/Gap)数
    std::map<SatID, double> mAmbFixed; // 固定后的 IF 模糊度(m)
    std::map<SatID, double> mNwSmooth; // 宽巷平滑值（跨历元 EMA）
    std::map<SatID, double> mNwVar;    // 宽巷平滑方差

};
