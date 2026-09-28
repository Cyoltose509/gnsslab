#include "PPPStatic.h"
#include "Const.h"
#include "ARLambda.h"
#include "Log.h"
#include "Geodesy.h"
#include "Troposphere.h"
#include "Ionosphere.h"
#include "CoordConvert.h"
#include "Weight.h"
#include "TimeConvert.h"
#include <cmath>
#include <Eigen/Eigen>
#include <map>
#include <set>
#include <algorithm>
#include <cstdlib>

bool PPPStatic::loadSp3(const std::string &path) {
    return mSp3.read(path);
}

bool PPPStatic::loadClk(const std::string &path) {
    return mClk.read(path);
}

bool PPPStatic::loadAtx(const std::string &path) {
    return mAntx.read(path);
}

bool PPPStatic::loadOsb(const std::string &path) {
    return mOsb.read(path);
}

bool PPPStatic::loadErp(const std::string &path) {
    return mErp.read(path);
}

bool PPPStatic::loadTro(const std::string &path) {
    return mTro.read(path);
}

bool PPPStatic::loadIonex(const std::string &path) {
    return mIonex.read(path);
}

bool PPPStatic::processEpoch(ObsData &obs) {
    try {
        if (mRcvAntenna.empty() && !obs.antType.empty()) mRcvAntenna = obs.antType;
        preprocess(obs);
        mFilterFrozen = false;
        solve(obs);
    } catch (const std::exception &e) {
        LOG_ERROR << "[PPPStatic] solve 异常: " << e.what();
        mGapThisEpoch = false;
        return false;
    }
    if (mFilterFrozen) return false;
    // 静态路径没有运动学专有逻辑（速度估计 / 发散兜底 / 重锚定刷新），整块由 PPPKinematic 接管。
    return true;
}

void PPPStatic::computeSatPos(ObsData &obsData) {
    satPVTTransTime.clear();
    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        if (!ifCodeTypes.count(sat.system)) continue;
        const FreqCombo &def = ifCodeTypes.at(sat.system);
        if (!codeList.count(def.code1) || !codeList.count(def.code2)) continue;

        const double obsVal = def.combineCodeFromObs(codeList);
        const double tau_total = (obsVal - rClockBias[sat.system]) / C_MPS;
        CommonTime t_emit = obsData.epoch;
        t_emit.m_sod -= tau_total;

        PVT pvt;
        if (const bool sp3HasClk = mSp3.hasClock(); mSp3.contains(sat) && (sp3HasClk || mClk.contains(sat))) {
            pvt = mSp3.getPVT(sat, t_emit); // SP3 自带钟差时已插值出 clockBias
            if (mClk.contains(sat)) pvt.clockBias = mClk.getClockBias(sat, t_emit);
            pvt.relativityCorrection = -2.0 * pvt.p.dot(pvt.v) / (C_MPS * C_MPS);
        } else {
            // 回退广播星历
            Ephemeris *eph = ephTable.find(sat, obsData.epoch);
            if (!eph) {
                auto itEph = ephMap.find(sat);
                if (itEph == ephMap.end()) continue;
                eph = itEph->second;
            }
            pvt = eph->getPVT(t_emit);
        }
        if (mApplySatPCO) mAntx.applySatPCO(sat, pvt.p, t_emit, def);
        satPVTTransTime[sat] = pvt;
    }
}

// 首历元引导：用 SPP 单历元伪距解估计接收机钟差
void PPPStatic::bootstrapFirstEpoch(ObsData &obsData) {
    if (!mFirstEpoch) return;
    // PPP 与基类共用同一份 mConfig，无需反向同步
    try { SPP::solve(obsData); } catch (...) {
    }
    for (const auto &[id, bias]: rClockBias) mClkBase[id] = bias;
    const Vector3d ap = obsData.antennaPosition;
    mRefPos = ap.norm() > 1e6 ? ap : xyz;
    mFirstEpoch = false;
}

// 设置各系统绝对接收机钟差先验：首历元用 SPP 引导的 mClkBase，其后用 KF 后验 mClkEstimate
void PPPStatic::applyPriorClocks() {
    for (const auto &[id, _]: ifCodeTypes) {
        const double absClk = mEkfInit
                                  ? (mClkEstimate.count(id) ? mClkEstimate.at(id) : 0.0)
                                  : mClkBase.count(id)
                                        ? mClkBase.at(id)
                                        : 0.0;
        rClockBias[id] = absClk;
    }
}

// 从 KF 携带的绝对后验均值读回 ECEF 位置（dX/dY/dZ 状态即绝对位置）。
Vector3d PPPStatic::currentEstimatedPosition() const {
    return {
        GnssEKF::getSolution(Parameter::dX, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dY, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dZ, ekf.currentUnkSet, ekf.solution)
    };
}

// 重置周跳探测器
void PPPStatic::resetSlipDetectors(const ObsData &obsData) {
    for (const auto &sat: mPrevSatSet) {
        if (obsData.satTypeValueData.find(sat) == obsData.satTypeValueData.end())
            mSlipDetectors[sat].reset();
    }
}


// 单星码/相位两个方程
double PPPStatic::pcvCorrection(const SatID &sat) const {
    if (!mApplyPcv) return 0.0;
    const FreqCombo &def = ifCodeTypes.at(sat.system);
    constexpr double R2D = 180.0 / PI;
    // 单星几何已在 computeElevAzim/computeSatPos 缓存到成员，无需再由调用方传入
    const double elev = satElevData.count(sat) ? satElevData.at(sat) : 0.0;
    const double azim = satAzimData.count(sat) ? satAzimData.at(sat) : 0.0;
    const PVT &pvt = satPVTRecTime.at(sat);
    // 接收机 PCV：以本地天顶距(90°-高度角)和方位角查表
    const double zenDeg = 90.0 - elev * R2D;
    const double azDeg = azim * R2D;
    // 卫星 PCV：以星下点角（卫星天底方向与"卫星->接收机"方向的夹角）查表
    const Vector3d recvEff = mEpochXyz + mEpochTide + mEpochPcoE;
    const Vector3d nadir = -pvt.p.normalized();
    const Vector3d toRecv = (recvEff - pvt.p).normalized();
    const double nadirDeg = std::acos(std::clamp(nadir.dot(toRecv), -1.0, 1.0)) * R2D;
    return mAntx.rcvPcvIF(mRcvAntenna, sat.system, def, zenDeg, azDeg)
           + mAntx.satPcvIF(sat, def, nadirDeg);
}

void PPPStatic::addSatelliteEquations(const SatID &sat, const TypeValueMap &tv,
                                      double map, double tropoFixed) {
    const FreqCombo &def = ifCodeTypes.at(sat.system);
    const Variable vclk(mCurObs->station, sat.system == 'G' ? Parameter::cdt : Parameter::cdt2);
    const Variable vamb = makeAmbiguityVar(mCurObs->station, sat, "IF");
    const PVT &pvt = satPVTRecTime.at(sat);
    const double elev = satElevData.count(sat) ? satElevData.at(sat) : 0.0;
    const double azim = satAzimData.count(sat) ? satAzimData.at(sat) : 0.0;
    const CommonTime &epoch = mCurObs->epoch;
    const double cb = mOsb.codeBias(def, sat);
    const double pb = mOsb.phaseBias(def, sat);
    const double pcv = pcvCorrection(sat);
    double ionoK = 0.0;
    if (mApplyHigherOrderIono && !mIonex.empty()) {
        const auto ipp = Ionosphere::ippGeometry(mEpochXyz, pvt.p, elev, azim, mIonex.heightKm());
        if (double vtec = 0.0; mIonex.getTec(epoch, ipp.latDeg, ipp.lonDeg, vtec))
            ionoK = Ionosphere::secondOrderK(vtec, ipp, def.f1, def.f2);
    }
    const double P_IF = def.combineCodeFromObs(tv) - cb + pcv + ionoK;
    const double L1m = tv.at(def.phase1);
    const double L2m = tv.at(def.phase2);
    const double L_IF = def.combinePhase(L1m, L2m) - pb + pcv - ionoK * 0.5;

    const double clkMean = mEkfInit ? (mClkEstimate.count(sat.system) ? mClkEstimate.at(sat.system) : 0.0) : 0.0;
    const double ztdMean = mEkfInit ? mZtdEstimate : 0.0;
    const double ambMean = mAmbEstimate.count(sat) ? mAmbEstimate.at(sat) : 0.0;

    const double dts = pvt.clockBias * C_MPS;
    const double rel = pvt.relativityCorrection * C_MPS;
    const Vector3d recvEff = mEpochXyz + mEpochTide + mEpochPcoE;
    double rho = 0.0;
    Vector3d los;
    geometricRangeAndLos(pvt.p, recvEff, rho, los);

    // 相位缠绕修正
    Vector3d exr, eyr;
    recvWindupBasis(mEpochXyz, exr, eyr);
    const double raw = Geodesy::phaseWindupCorrection(pvt.p, pvt.v, mEpochXyz, epoch, exr, eyr);
    const double prev = mWindupPrev.count(sat) ? mWindupPrev[sat] : raw;
    double dphi = raw - prev;
    while (dphi > 0.5) dphi -= 1.0;
    while (dphi < -0.5) dphi += 1.0;
    const double acc = (mWindupAcc.count(sat) ? mWindupAcc[sat] : 0.0) + dphi;
    mWindupPrev[sat] = raw;
    mWindupAcc[sat] = acc;
    const double windup_m = mApplyWindup ? def.lambdaN * acc : 0.0;

    // 粗差门限
    constexpr double rejCode = 1.0e4; //NOLINT
    constexpr double rejPhase = 1.0e4; //NOLINT
    const double codePrefit = P_IF - (rho - dts - rel) - clkMean - ztdMean * map - tropoFixed;
    const double phasePrefit = L_IF - (rho - dts - rel) - ambMean - clkMean - ztdMean * map - tropoFixed - windup_m; //NOLINT
    if (mProcEpoch > GATE_WARMUP && (std::abs(codePrefit) > rejCode || std::abs(phasePrefit) > rejPhase)) {
        satRejected.insert(sat);
        return;
    }
    {
        EquID eid(sat, def.code1 + "+" + def.code2);
        const double prefit = P_IF - (rho - dts - rel) - clkMean - ztdMean * map - tropoFixed;
        addObservationEquation(eid, prefit, los, mWeightCode,
                               false, elev);
    }

    {
        EquID eid(sat, def.phase1 + "+" + def.phase2);
        const double prefit = L_IF - (rho - dts - rel) - ambMean - clkMean - ztdMean * map - tropoFixed - windup_m;
        addObservationEquation(eid, prefit, los, mWeightPhase,
                               true, elev);
    }
}

void PPPStatic::addObservationEquation(const EquID &eid, double prefit, const Vector3d &los,
                                       const Weight &w, bool hasAmb, double elev) {
    // 位置/常值偏移状态变量恒由 station + 参数名确定，就地构造即可，无需跨函数共享
    const Variable vdx(mCurObs->station, Parameter::dX);
    const Variable vdy(mCurObs->station, Parameter::dY);
    const Variable vdz(mCurObs->station, Parameter::dZ);
    const Variable vclk(mCurObs->station, eid.sat.system == 'G' ? Parameter::cdt : Parameter::cdt2);
    EquData ed;
    ed.prefit = prefit;
    ed.varCoeffData[vdx] = los[0];
    ed.varCoeffData[vdy] = los[1];
    ed.varCoeffData[vdz] = los[2];
    ed.varCoeffData[vclk] = 1.0;
    if (hasAmb) {
        const Variable vamb = makeAmbiguityVar(mCurObs->station, eid.sat, "IF");
        ed.varCoeffData[vamb] = 1.0;
        mVarSet.insert(vamb);
    }
    if (mEstimateConstOffset) {
        ed.varCoeffData[Variable(mCurObs->station, Parameter::dOX)] = los[0];
        ed.varCoeffData[Variable(mCurObs->station, Parameter::dOY)] = los[1];
        ed.varCoeffData[Variable(mCurObs->station, Parameter::dOZ)] = los[2];
    }
    // 高度角随机模型（Weight 类，与 RTK/SPP 共用）：σ²=a²+(b/sinE)²。
    // 用成员式 mWeightCode/mWeightPhase（整轮复用，不再逐星重建）。
    ed.weight = w.weight(elev);
    mEq.obsEquData[eid] = ed;
}

void PPPStatic::recvWindupBasis(const Vector3d &recv, Vector3d &exr, Vector3d &eyr) const {
    // 运动学：接收机有明显速度(>1 m/s)时，用沿迹/轨道面基替代 ENU 基——快速运动体(LEO)的本地
    // 坐标系随速度方向旋转，ENU 基的"北/东"失去意义，沿用静态基会让相位缠绕算错。
    // 静态：速度≈0 ⇒ 退回 ENU 基（原行为，地面站零回归）。
    if (mRecvVel.squaredNorm() > 1.0) {
        const Vector3d ur = recv.normalized(); // 径向(地心向外)
        const Vector3d ut = mRecvVel.normalized(); // 沿迹(速度方向)
        Vector3d uc = ur.cross(ut);
        if (uc.squaredNorm() < 1e-6) uc = Vector3d(1, 0, 0);
        uc.normalize();
        exr = ut;
        eyr = -uc;
        return;
    }
    auto blh = XYZtoBLH(recv, Frame::WGS84);
    Matrix3d E = getBLMatrix(blh[0], blh[1]);
    exr = E.row(1).transpose();
    eyr = -E.row(0).transpose();
}

Vector3d PPPStatic::computeTideDisplacement(const Vector3d &xyzEst, const CommonTime &epoch) const {
    Vector3d d = Vector3d::Zero();
    if (mApplySolidTide) d = Geodesy::solidTideDisplacement(xyzEst, epoch);
    if (mApplyPoleTide && !mErp.empty()) {
        if (double xp = 0.0, yp = 0.0, ut1Utc = 0.0; mErp.getErp(epoch, xp, yp, ut1Utc))
            d += Geodesy::poleTideDisplacement(xyzEst, epoch, xp, yp);
    }
    return d;
}


// IF 组合绝对模糊度(m)：纯计算，供 resetAmbiguity 与首次出现初始化共用
double PPPStatic::computeIfAmbiguity(const SatID &sat, const TypeValueMap &tv,
                                     const double map) const {
    const FreqCombo &def = ifCodeTypes.at(sat.system);
    const PVT &pvt = satPVTRecTime.at(sat);
    const Vector3d recvEff = mEpochXyz + recvAntennaOffsetECEF(mEpochXyz, mCurObs->epoch);
    const double rho0 = (pvt.p - recvEff).norm();
    const double dts0 = pvt.clockBias * C_MPS;
    const double rel0 = pvt.relativityCorrection * C_MPS;
    const double L1m0 = tv.at(def.phase1), L2m0 = tv.at(def.phase2);
    const double L_IF0 = def.combinePhase(L1m0, L2m0) - mOsb.phaseBias(def, sat)
                         + pcvCorrection(sat);
    const double clk0 = mEkfInit ? (mClkEstimate.count(sat.system) ? mClkEstimate.at(sat.system) : 0.0) : 0.0;
    const double ztd0 = mEkfInit ? mZtdEstimate : 0.0;
    return L_IF0 - (rho0 - dts0 - rel0) - clk0 - ztd0 * map;
}

// 周跳/失锁重置 IF 模糊度；isSlipReset=false 即首次出现初始化（只算值、不强制 EKF）
void PPPStatic::resetAmbiguity(const SatID &sat, const TypeValueMap &tv,
                               const double map, const bool isSlipReset) {
    ++ambResets;
    const Variable vamb = makeAmbiguityVar(mCurObs->station, sat, "IF");
    (*mCsData)[vamb] = isSlipReset ? 2.0 : 1.0;
    if (isSlipReset) ++slipVerdicts;
    mAmbEstimate[sat] = computeIfAmbiguity(sat, tv, map);
    if (isSlipReset) ekf.forceValue(vamb, mAmbEstimate[sat]);
}

void PPPStatic::detectCycleSlip(const SatID &sat, const TypeValueMap &tv,
                                bool &slipReset, bool &resetAmb) {
    const FreqCombo &def = ifCodeTypes.at(sat.system);
    resetAmb = false;
    slipReset = false;
    if (!mEpochFirstIter) return;
    CycleSlip &det = mSlipDetectors[sat];
    det.configure(def);
    det.setGfThreshold(0.0);
    double osbP1 = 0.0, osbP2 = 0.0, osbL1 = 0.0, osbL2 = 0.0;
    mOsb.biases(def, sat, osbP1, osbP2, osbL1, osbL2);
    const double L1m = tv.at(def.phase1), L2m = tv.at(def.phase2);
    const double C1m = tv.at(def.code1), C2m = tv.at(def.code2);
    const CycleSlip::Verdict verdict = det.feed(L1m, L2m, C1m, C2m, true, osbP1, osbP2, osbL1, osbL2);
    const bool newArc = mPrevSatSet.find(sat) == mPrevSatSet.end();
    const bool mwSlip = det.lastMwSlip();
    const bool gfSlip = det.lastGfSlip();
    const bool gfArtifact = det.lastGfArtifact();
    if (gfSlip) mGfEpoch.record(sat, gfArtifact);
    resetAmb = verdict == CycleSlip::Verdict::Gap || newArc || (mUseCycleSlip && mwSlip);
    slipReset = mUseCycleSlip && mwSlip;
    if (mwSlip) ++slipVerdicts;
    if (gfSlip) mGfSlipSats.insert(sat);
}

// 单星：周跳检测 + 模糊度初始化/重置 + 建方程（每历元共享量已填充到成员 mEpoch* / mEpochTropo）
void PPPStatic::processSatellite(const SatID &sat, const TypeValueMap &tv) {
    if (!ifCodeTypes.count(sat.system)) return;
    if (const FreqCombo &def = ifCodeTypes.at(sat.system); !def.available(tv, true)) return;
    if (!satPVTRecTime.count(sat)) return;
    double elev = 0.0;
    if (const auto it = satElevData.find(sat); it != satElevData.end()) elev = it->second;
    if (elev < mConfig.cutoffElevRad) {
        satRejected.insert(sat);
        return;
    }

    double mW = 0.0;
    const double mH = Troposphere::tropoNMF(elev, mEpochTropo.latDeg, mEpochTropo.hgt, mEpochTropo.doy, &mW);
    const double map = mW;
    const double tropoFixed = mHasTropo ? mH * mEpochTropo.zhd : 0.0;
    mSatMap[sat] = map;
    mSysSeen.insert(sat.system);

    bool resetAmb = false;
    bool slipReset = false;
    detectCycleSlip(sat, tv, slipReset, resetAmb);
    if (mForceAmbReset) resetAmb = true;
    if (resetAmb) {
        resetAmbiguity(sat, tv, map, slipReset);
    } else if (!mAmbEstimate.count(sat)) {
        mAmbEstimate[sat] = computeIfAmbiguity(sat, tv, map);
    }
    mCurrentSatSet.insert(sat);
    addSatelliteEquations(sat, tv, map, tropoFixed);
}

void PPPStatic::buildEquSys(ObsData &obsData, const Vector3d &xyzEst, VariableDataMap &csData, bool firstIter) {
    mEq.reset();
    mEq.station = obsData.station;
    mVarSet.clear();
    csData.clear();

    // 高度角随机模型：a²+(b/sinE)²，码/相位用不同 sigma；构造一次整轮复用。
    // 配置在构造后才注入，故首历元惰性初始化。
    if (!mWeightInit) {
        const double minSin = std::sin(mConfig.cutoffElevRad);
        mWeightCode  = Weight(mConfig.sigIFCode  / std::sqrt(2.0), mConfig.sigIFCode  / std::sqrt(2.0), minSin);
        mWeightPhase = Weight(mConfig.sigmaPhase / std::sqrt(2.0), mConfig.sigmaPhase / std::sqrt(2.0), minSin);
        mWeightInit = true;
    }

    const Variable vdx(obsData.station, Parameter::dX);
    const Variable vdy(obsData.station, Parameter::dY);
    const Variable vdz(obsData.station, Parameter::dZ);
    mVarSet.insert(vdx);
    mVarSet.insert(vdy);
    mVarSet.insert(vdz);

    const Variable vox(obsData.station, Parameter::dOX);
    const Variable voy(obsData.station, Parameter::dOY);
    const Variable voz(obsData.station, Parameter::dOZ);
    if (mEstimateConstOffset) {
        mVarSet.insert(vox);
        mVarSet.insert(voy);
        mVarSet.insert(voz);
    }

    resetSlipDetectors(obsData);

    // 填充每历元共享派生量（成员式，替代 EpochContext 参数对象；与 satElevData 等同为 per-epoch 缓存）
    mCurObs = &obsData;
    mEpochXyz = xyzEst;
    mEpochTide = computeTideDisplacement(xyzEst, obsData.epoch);
    mEpochPcoE = recvAntennaOffsetECEF(xyzEst, obsData.epoch);
    mEpochFirstIter = firstIter;
    mSatMap.clear();
    mSysSeen.clear();
    mCurrentSatSet.clear();
    mCsData = &csData;
    mGfEpoch = CycleSlipEpoch{};
    mGfSlipSats.clear();

    // 对流层干/湿分离：ZHD=Saastamoinen，映射=NMF 干/湿，滤波状态只随湿部分变
    const auto blhEst = XYZtoBLH(xyzEst, Frame::WGS84);
    mEpochTropo.latDeg = blhEst.B() * 180.0 / PI;
    mEpochTropo.hgt = blhEst.H();
    mEpochTropo.doy = static_cast<double>(CommonTime2YDSTime(obsData.epoch).doy);
    mEpochTropo.zhd = Troposphere::tropoSaastamoinenZHD(blhEst.B(), blhEst.H());

    for (const auto &[sat, tv]: obsData.satTypeValueData)
        processSatellite(sat, tv);

    SatIDSet gfExcludeSet;
    const SatIDSet gfApplySet = mGfEpoch.decide(mGfCommonMode, gfExcludeSet);
    // GF 共模裁决通过后才回放重置：上下文(星历/仰角/方位/映射/观测)本历元已缓存，直接重取
    for (const auto &sat: gfApplySet) {
        const TypeValueMap &tv = obsData.satTypeValueData.at(sat);
        const double map = mSatMap.at(sat);
        resetAmbiguity(sat, tv, map, true);
    }

    for (auto &[eid, ed]: mEq.obsEquData)
        if (gfExcludeSet.count(eid.sat))
            ed.weight = 1.0e-20;
    mForceAmbReset = false;
    for (char sys: mSysSeen)
        mVarSet.insert(Variable(obsData.station, sys == 'G' ? Parameter::cdt : Parameter::cdt2));

    addZtdConstraint();

    mPrevSatSet = std::move(mCurrentSatSet);
}

// 对流层 ZTD 状态 + 先验/外部产品约束方程
void PPPStatic::addZtdConstraint() {
    if (!mHasTropo) return;
    const Variable vztd(mCurObs->station, Parameter::ztd);
    mVarSet.insert(vztd);
    for (auto &[eid, ed]: mEq.obsEquData) {
        const double map = mSatMap.count(eid.sat) ? mSatMap.at(eid.sat) : 1.0;
        ed.varCoeffData[vztd] = map;
    }

    double ztdTarget = 0.0;
    double ztdSigma = mConfig.ztdPriorSigma;
    bool useTro = false;
    if (mUseTroZtd && !mTro.empty() && mConfig.troZtdSigma > 0.0) {
        if (double ztdTro = 0.0, sigTro = 0.0; mTro.getZtd(mCurObs->station, mCurObs->epoch, ztdTro, sigTro)) {
            ztdTarget = ztdTro - mEpochTropo.zhd;
            ztdSigma = mConfig.troZtdSigma;
            useTro = true;
        }
    }
    if (ztdSigma > 0.0) {
        const EquID eid(SatID('T', 0), std::string(useTro ? "ZTD_TRO" : "ZTD_PRIOR"));
        EquData ed;
        const double ztdCur = mEkfInit ? mZtdEstimate : mConfig.ztdInitValue;
        ed.prefit = ztdTarget - ztdCur;
        ed.varCoeffData[vztd] = 1.0;
        ed.weight = 1.0 / (ztdSigma * ztdSigma);
        mEq.obsEquData[eid] = ed;
    }
}

// 地基接收机天线 PCO
Vector3d PPPStatic::recvAntennaOffsetECEF(const Vector3d &xyzEst, const CommonTime &epoch) const {
    (void) epoch;
    Vector3d recvPCOe = Vector3d::Zero();
    if (mApplyRcvPCO && !mRcvAntenna.empty()) {
        if (const Vector3d pcoENU = mAntx.getRcvPCOENU(mRcvAntenna); pcoENU.squaredNorm() > 1e-12) {
            const auto blh = XYZtoBLH(xyzEst, Frame::WGS84);
            const Matrix3d E = getBLMatrix(blh[0], blh[1]);
            recvPCOe = E.transpose() * pcoENU;
        }
    }
    return recvPCOe;
}

// 线性化/预测位置
Vector3d PPPStatic::predictPosition(const double dt, const CommonTime &epoch) const {
    (void) dt;
    (void) epoch;
    // 静态：线性化点 = 上一历元滤波后验（位置作紧随机常数），无需重锚定。
    return currentEstimatedPosition();
}


void PPPStatic::readback(const ObsData &obsData) {
    try { result.xyz = currentEstimatedPosition(); } catch (...) {
    }
    mConstOffset = Vector3d::Zero();
    GnssEKF::tryGetSolution(Parameter::dOX, ekf.currentUnkSet, ekf.solution, mConstOffset.x());
    GnssEKF::tryGetSolution(Parameter::dOY, ekf.currentUnkSet, ekf.solution, mConstOffset.y());
    GnssEKF::tryGetSolution(Parameter::dOZ, ekf.currentUnkSet, ekf.solution, mConstOffset.z());
    if (mEstimateConstOffset) result.xyz += mConstOffset;
    result.blh = XYZtoBLH(result.xyz, Frame::WGS84);
    result.numSats = static_cast<int>(mEq.obsEquData.size()) / 2;

    for (const auto &[id, _]: ifCodeTypes) {
        const Parameter clkPar = id == 'G' ? Parameter::cdt : Parameter::cdt2;
        GnssEKF::tryGetSolution(clkPar, ekf.currentUnkSet, ekf.solution, mClkEstimate[id]);
    }
    GnssEKF::tryGetSolution(Parameter::ztd, ekf.currentUnkSet, ekf.solution, mZtdEstimate);
    for (const auto &[sat, tv]: obsData.satTypeValueData) {
        if (!ifCodeTypes.count(sat.system)) continue;
        if (!ifCodeTypes.at(sat.system).available(tv, true)) continue;
        const Variable vamb = makeAmbiguityVar(obsData.station, sat, "IF");
        mAmbEstimate[sat] = ekf.getSolution(vamb); // 按 Variable 取：不在集中直接返回 0，不抛
    }

    // 逐星码残差（后验残差）
    if (const VectorXd postfit = ekf.postfitResidual; postfit.size() > 0) {
        int iobs = 0;
        for (const auto &[eid, ed]: mEq.obsEquData) {
            if (iobs < postfit.size() && !eid.obsType.empty() && eid.obsType[0] != 'L')
                result.postRes[eid.sat] = postfit(iobs);
            ++iobs;
        }
    }

    computeDops();

    const Variable vdx(obsData.station, Parameter::dX);
    const Variable vdy(obsData.station, Parameter::dY);
    const Variable vdz(obsData.station, Parameter::dZ);
    const Variable vox(obsData.station, Parameter::dOX);
    const Variable voy(obsData.station, Parameter::dOY);
    const Variable voz(obsData.station, Parameter::dOZ);
    const MatrixXd P = ekf.covMatrix;
    auto axisVar = [&](const Variable &pos, const Variable &off) -> double {
        if (!ekf.currentIndexData.count(pos)) return 0.0;
        const int ip = ekf.currentIndexData.at(pos);
        double v = P(ip, ip);
        if (ekf.currentIndexData.count(off)) {
            const int io = ekf.currentIndexData.at(off);
            v += P(io, io) + 2.0 * P(ip, io);
        }
        return v;
    };
    const double sx = std::sqrt(std::max(axisVar(vdx, vox), 0.0));
    const double sy = std::sqrt(std::max(axisVar(vdy, voy), 0.0));
    const double sz = std::sqrt(std::max(axisVar(vdz, voz), 0.0));
    result.sigmaP = std::sqrt(sx * sx + sy * sy + sz * sz);
    result.sigmaP = std::max(result.sigmaP, 0.01); // 绝对下限 1 cm，避免首历元零方差
}

void PPPStatic::computeDops() {
    MatrixXd H;
    int n = 0;
    for (const auto &[eid, ed]: mEq.obsEquData) {
        if (eid.obsType.empty() || eid.obsType[0] == 'L') continue;
        double hrow[4] = {0, 0, 0, 0};
        for (const auto &[var, coeff]: ed.varCoeffData) {
            if (const auto pt = var.getParaType(); pt == Parameter::dX) hrow[0] = coeff;
            else if (pt == Parameter::dY) hrow[1] = coeff;
            else if (pt == Parameter::dZ) hrow[2] = coeff;
            else if (pt == Parameter::cdt || pt == Parameter::cdt2) hrow[3] = coeff;
        }
        H.conservativeResize(n + 1, 4);
        H.row(n) << hrow[0], hrow[1], hrow[2], hrow[3];
        ++n;
    }
    if (n >= 4) {
        const MatrixXd HtH = H.transpose() * H;
        if (const FullPivLU<MatrixXd> lu(HtH); lu.isInvertible()) {
            const MatrixXd inv = lu.inverse();
            computePositionDops(result, inv.topLeftCorner(3, 3), inv(3, 3), result.blh);
        }
    }
}

void PPPStatic::solve(ObsData &obsData) {
    result.reset();
    mEq.reset();
    ++mProcEpoch;

    // 每历元就地探测 IF 组合
    detectIFCombinations(obsData);

    if (ifCodeTypes.empty()) return;

    if (mHasLastEpoch) {
        if (const double gap = std::fabs(obsData.epoch - mLastEpochTime);
            (mNominalInterval > 0.0 && gap > 1.5 * mNominalInterval) || gap > 60.0) {
            mForceAmbReset = true;
            mGapThisEpoch = true;
        } else if (mNominalInterval < 0.0 && gap >= 1.0 && gap <= 60.0) mNominalInterval = gap;
    }

    bootstrapFirstEpoch(obsData);
    applyPriorClocks();
    computeSatPos(obsData);
    earthRotation();
    computeElevAzim();
    mCurObs = &obsData; // 让 predictPosition 在调用前即指向当前历元（下沉重锚定时需当前观测）

    VariableDataMap csData;
    if (!mEkfInit) {
        const Vector3d xyzRef = mFirstEpochSPP
                                    ? predictPosition(0.0, obsData.epoch)
                                    : mRefPos;
        // 湿延迟(zwd)约定：状态先验≈0，初值由 setTropo 固定给 0.0（harness 默认即 0），
        // 不再用 Saastamoinen 总延迟回填——那样会变成"总 ZTD"约定，叠加多算 zhd·mW 致米级 U 偏。
        buildEquSys(obsData, xyzRef, csData, true);
        mFilterFrozen = static_cast<int>(mEq.obsEquData.size()) / 2 < mConfig.minSatsForUpdate;
        if (!mFilterFrozen) {
            // 不在此处硬写 ambNoise/ztdProcNoise：PPPStatic::Config 的默认值本就是 1e-8，
            // 硬写会静默覆盖 LEO 设定的 1e-7 以及 setTropo() 传入的 ztdQ。
            ekf.configure(mConfig.posNoise, mHasTropo, mConfig.ambNoise, mConfig.clockNoise);
            ekf.ztdInitVar = mConfig.ztdInitVar;
            ekf.ztdProcNoise = mConfig.ztdProcNoise;
            ekf.ztdInitValue = mConfig.ztdInitValue;
            ekf.timeUpdate(mVarSet, &csData);
            ekf.setBootstrapPosition(xyzRef, mConfig.firstEpochPosVar);
            ekf.setBootstrapAmbiguity(obsData.station, mAmbEstimate, &csData);
            ekf.measUpdate(mEq);
            mEkfInit = true;
        }
    } else {
        double dt = 0.0;
        if (mHasLastEpoch) {
            dt = obsData.epoch - mLastEpochTime;
            if (dt < 0.0) dt = 0.0;
        }
        const Vector3d xyzEst = predictPosition(dt, obsData.epoch);
        buildEquSys(obsData, xyzEst, csData, true);
        mFilterFrozen = static_cast<int>(mEq.obsEquData.size()) / 2 < mConfig.minSatsForUpdate;
        if (!mFilterFrozen) {
            ekf.timeUpdate(mVarSet, &csData);
            // 位置先验 = 本历元线性化点（同源；静态方差不动，运动学由派生类在 timeUpdate 前后另行调 setBootstrapPosition）。
            ekf.setBootstrapPosition(xyzEst);
            ekf.setBootstrapAmbiguity(obsData.station, mAmbEstimate, &csData);
            ekf.savePrior();
            ekf.measUpdate(mEq);
        }
    }
    if (mFilterFrozen) {
        result.xyz = Vector3d::Zero();
        result.numSats = 0;
        xyz = currentEstimatedPosition();
        computeElevAzim();
        return;
    }
    mLastEpochTime = obsData.epoch;
    mHasLastEpoch = true;
    mGapThisEpoch = false;

    if (mUseAR) tryFixAmbiguities(obsData);
    readback(obsData);
    xyz = currentEstimatedPosition();
    computeElevAzim();
}

bool PPPStatic::tryFixAmbiguities(ObsData &obsData) {
    mArRatio = 0.0;
    mArNumFixed = 0;
    result.xyzFixed = Vector3d::Zero();
    ++mArAttempts;
    if (!mEkfInit) return false;

    const VectorXd x = ekf.solution;
    const MatrixXd P = ekf.covMatrix;

    // 收集满足固定条件的卫星：IF 组合齐备、星历可用、模糊度已入状态、方差 < 阈值
    std::vector<SatID> sats;
    std::vector<int> idx;
    std::vector<double> Bf;
    for (const auto &[sat, tv]: obsData.satTypeValueData) {
        if (!ifCodeTypes.count(sat.system)) continue;
        if (const FreqCombo &def = ifCodeTypes.at(sat.system); !def.available(tv, true)) continue;
        if (satPVTRecTime.count(sat) == 0) continue;
        const Variable vamb = makeAmbiguityVar(obsData.station, sat, "IF");
        if (!ekf.currentIndexData.count(vamb)) continue;
        const int i = ekf.currentIndexData.at(vamb);
        if (const double sig = std::sqrt(P(i, i)); sig > mConfig.arSigGate) continue;
        sats.push_back(sat);
        idx.push_back(i);
        Bf.push_back(x(i));
    }
    const int n = static_cast<int>(sats.size());
    if (n < 5) return false;

    // 构造宽巷(Nw)/窄巷(N1)实数模糊度向量 a 与协方差 Q
    VectorXd a(2 * n);
    MatrixXd Q(2 * n, 2 * n);
    Q.setZero();
    std::vector<double> lam_n(n), varNw(n), Nw(n), cFac(n);
    for (int k = 0; k < n; ++k) {
        const SatID &sat = sats[k];
        const TypeValueMap &tv = obsData.satTypeValueData.at(sat);
        const FreqCombo &def = ifCodeTypes.at(sat.system);
        const double f1 = def.f1, f2 = def.f2;
        lam_n[k] = def.lambdaN;
        cFac[k] = f2 / (f1 - f2);
        const double lam1 = def.lambda1, lam2 = def.lambda2;
        double osbP1 = 0, osbP2 = 0, osbL1 = 0, osbL2 = 0;
        mOsb.biases(def, sat, osbP1, osbP2, osbL1, osbL2);
        const double L1m = tv.at(def.phase1), L2m = tv.at(def.phase2);
        const double C1m = tv.at(def.code1), C2m = tv.at(def.code2);
        const double NwInst = def.MW_cycle(C1m, C2m, L1m, L2m, osbP1, osbP2, osbL1, osbL2);
        const double a1 = 1.0 / lam1, a2 = -1.0 / lam2;
        const double r = f1 / f2 * (f1 / f2);
        const double K = -(a1 + a2);
        const double Gc = a1 + r * a2;
        const double b2 = (Gc - K) / (r - 1.0);
        const double b1 = K - b2;
        const double sL = mConfig.sigmaPhase, sP = mConfig.sigIFCode;
        const double varNwInst = a1 * a1 * sL * sL + a2 * a2 * sL * sL + b1 * b1 * sP * sP + b2 * b2 * sP * sP;
        // 宽巷弧段复位：几何无关组合 GF = L2 - L1 判跳则清空 EMA 缓冲，避免 Nw 跨弧段漂移
        const bool nwSlip = mSlipDetectors.count(sat)
                                ? mSlipDetectors.at(sat).gfWideLaneSlip(L1m, L2m)
                                : true;
        if (nwSlip) {
            mNwSmooth.erase(sat);
            mNwVar.erase(sat);
        }
        double NwVal, varNwVal;
        if (auto it = mNwSmooth.find(sat); it == mNwSmooth.end() || mNwVar.find(sat) == mNwVar.end()) {
            NwVal = NwInst;
            varNwVal = varNwInst;
            mNwSmooth[sat] = NwInst;
            mNwVar[sat] = varNwInst;
        } else {
            const double v0 = mNwVar[sat];
            const double Kg = v0 / (v0 + varNwInst);
            mNwSmooth[sat] += Kg * (NwInst - mNwSmooth[sat]);
            mNwVar[sat] = (1.0 - Kg) * v0;
            NwVal = mNwSmooth[sat];
            varNwVal = mNwVar[sat];
        }
        Nw[k] = NwVal;
        varNw[k] = varNwVal;
        const double N1 = Bf[k] / lam_n[k] - cFac[k] * NwVal;
        a(2 * k) = NwVal;
        a(2 * k + 1) = N1;
        Q(2 * k, 2 * k) = varNwVal;
        Q(2 * k + 1, 2 * k + 1) = P(idx[k], idx[k]) / (lam_n[k] * lam_n[k]) + cFac[k] * cFac[k] * varNwVal;
        Q(2 * k, 2 * k + 1) = Q(2 * k + 1, 2 * k) = -cFac[k] * varNwVal;
        for (int j = 0; j < n; ++j)
            if (j != k)
                Q(2 * k + 1, 2 * j + 1) = P(idx[k], idx[j]) / (lam_n[k] * lam_n[j]);
    }

    const auto [afix, ratio, fixed] = resolveWithRatio(a, Q, mConfig.arRatioThreshold);
    mArRatio = ratio;
    if (!fixed) return false;
    VectorXd z(n);
    for (int k = 0; k < n; ++k) {
        const SatID &sat = sats[k];
        const FreqCombo &def = ifCodeTypes.at(sat.system);
        const double f1 = def.f1, f2 = def.f2;
        const double N1f = afix(2 * k + 1), Nwf = afix(2 * k);
        const double Bfix = C_MPS * ((f1 - f2) * N1f + f2 * Nwf) / (f1 * f1 - f2 * f2);
        mAmbFixed[sat] = Bfix;
        z(k) = x(idx[k]) + (Bfix - Bf[k]);
    }

    // 固定解回代：以 varFix 强约束将整数模糊度解投影回全状态向量
    const int Nst = static_cast<int>(x.size());
    MatrixXd H(n, Nst);
    H.setZero();
    for (int k = 0; k < n; ++k) H(k, idx[k]) = 1.0;
    constexpr double varFix = 1.0e-10;
    const MatrixXd R = varFix * MatrixXd::Identity(n, n);
    const MatrixXd S = H * P * H.transpose() + R;
    const MatrixXd K = P * H.transpose() * S.inverse();
    const VectorXd xf = x + K * (z - H * x);

    Vector3d xyzFixed(
        xf(ekf.currentIndexData.at(Variable(obsData.station, Parameter::dX))),
        xf(ekf.currentIndexData.at(Variable(obsData.station, Parameter::dY))),
        xf(ekf.currentIndexData.at(Variable(obsData.station, Parameter::dZ))));
    const Vector3d xyzFloat(
        x(ekf.currentIndexData.at(Variable(obsData.station, Parameter::dX))),
        x(ekf.currentIndexData.at(Variable(obsData.station, Parameter::dY))),
        x(ekf.currentIndexData.at(Variable(obsData.station, Parameter::dZ))));
    // 一致性闸门：固定解与浮点解位置偏差过大视为假固定
    if ((xyzFixed - xyzFloat).norm() > 0.1) return false;
    result.xyzFixed = xyzFixed;
    result.ratio = ratio;
    mArNumFixed = n;
    ++mArSuccess;
    return true;
}
