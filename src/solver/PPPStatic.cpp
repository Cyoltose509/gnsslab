#include "PPPStatic.h"
#include "Const.h"
#include "SolverLSQ.h"
#include "ARLambda.h"
#include "Log.h"
#include "Geodesy.h"
#include "Troposphere.h"
#include "Ionosphere.h"
#include "CoordConvert.h"
#include "Weight.h"
#include "TimeConvert.h"
#include "TimeStruct.h"
#include <Eigen/Eigen>
#include <map>
#include <set>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <string>


bool PPPStatic::loadSp3(const std::string &path) {
    return mSp3.read(path);
}

bool PPPStatic::loadClk(const std::string &path, const std::set<char> &allowedSys) {
    return mClk.read(path, allowedSys);
}

bool PPPStatic::loadAtx(const std::string &path) {
    return mAntx.read(path);
}

bool PPPStatic::loadOsb(const std::string &path) {
    const bool ok = mOsb.read(path);
    mOsbSatSet = mOsb.satelliteSet();
    return ok;
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
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;
        const FreqCombo &def = *defp;
        if (!codeList.count(def.code1) || !codeList.count(def.code2)) continue;

        const double obsVal = def.combineCodeFromObs(codeList);
        CommonTime t0 = obsData.epoch;
        t0.m_sod -= obsVal / C_MPS;
        double dt_sv = 0.0;
        if (mClk.contains(sat) && !mClk.covers(sat, t0)) continue;
        if (mClk.contains(sat)) dt_sv = mClk.getClockBias(sat, t0); // 精密钟(秒)
        CommonTime t_emit = t0;
        t_emit.m_sod -= dt_sv;

        PVT pvt;
        if (const bool sp3HasClk = mSp3.hasClock();
            mSp3.contains(sat) && mSp3.covers(sat, t_emit) && (sp3HasClk || mClk.contains(sat))) {
            pvt = mSp3.getPVT(sat, t_emit); // SP3 自带钟差时已插值出 clockBias；相对论改正由读取器按 r·v 计算
            if (mClk.contains(sat)) pvt.clockBias = mClk.getClockBias(sat, t_emit);
        } else {
            // 回退广播星历（OEM7 内嵌星历已统一经 ephTable 传入）
            Ephemeris *eph = ephTable.find(sat, obsData.epoch);
            if (!eph) continue;
            if (dt_sv == 0.0) {
                // 无 CLK 时用广播钟差补一次发射时刻迭代
                const PVT pvt0 = eph->getPVT(t0);
                t_emit.m_sod -= pvt0.clockBias;
            }
            pvt = eph->getPVT(t_emit);
        }
        mAntx.applySatPCO(sat, pvt.p, t_emit, def); // 无 ATX 时内部查不到→自动 no-op
        satPVTTransTime[sat] = pvt;
    }
}

// 首历元引导：用 SPP 单历元伪距解估计接收机钟差
void PPPStatic::bootstrapFirstEpoch(ObsData &obsData) {
    if (!mFirstEpoch) return;
    // PPP 与基类共用同一份 mConfig，无需反向同步
    for (const auto &[id, bias]: rClockBias) mClkBase[id] = bias;
    const Eigen::Vector3d ap = obsData.antennaPosition;
    mRefPos = ap.norm() > 1e6 ? ap : xyz;
    xyz = mRefPos;
    mFirstEpoch = false;
}


void PPPStatic::applyPriorClocks() {
    for (const auto &[sys, par]: sysCdtParam) {
        const double absClk = mEkfInit
                                  ? (mClkEstimate.count(sys) ? mClkEstimate.at(sys) : 0.0)
                                  : mClkBase.count(sys)
                                        ? mClkBase.at(sys)
                                        : 0.0;
        rClockBias[sys] = absClk;
    }
}

bool PPPStatic::wantCommonClock(const ObsData &obs) const {
    if (mKinematic) return false;            // 运动学/LEO 不走公共钟+ISB
    if (ekf.isbInitVar <= 0.0) return false; // 未开启 ISB 估计
    bool gps = false;
    std::set<char> sys;
    for (const auto &[sat, tv]: obs.satTypeValueData) {
        (void) tv;
        sys.insert(sat.system);
        if (sat.system == 'G') gps = true;
    }
    for (char s : sys) {
        if (s != 'G' && sysCdtParam.find(s) == sysCdtParam.end())
            return false;
    }
    return gps && sys.size() >= 2; // 须含 GPS(公共钟载体)且至少两系统
}

Eigen::Vector3d PPPStatic::currentEstimatedPosition() const {
    return {
        GnssEKF::getSolution(Parameter::dX, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dY, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dZ, ekf.currentUnkSet, ekf.solution)
    };
}

// 首历元精确初位置
std::pair<Eigen::Vector3d, bool> PPPStatic::solveInitialPosition(ObsData &obsData) {
    Eigen::Vector3d xc = mRefPos;
    if (!isSaneECEF(xc)) {
        if (isSaneECEF(mCurObs ? mCurObs->antennaPosition : Eigen::Vector3d::Zero()))
            xc = mCurObs->antennaPosition;
        else
            return {Eigen::Vector3d::Zero(), false};
    }
    VariableDataMap csData;
    SolverLSQ solver;
    const int maxIter = mConfig.maxReanchorIter > 0 ? mConfig.maxReanchorIter : 8;
    for (int it = 0; it < maxIter; ++it) {
        buildEquSys(obsData, xc, csData, true);
        for (auto it2 = mEq.obsEquData.begin(); it2 != mEq.obsEquData.end();) {
            if (const auto &eid = it2->first; !eid.obsType.empty() && eid.obsType[0] == 'L') it2 = mEq.obsEquData.erase(it2);
            else {
                // 自动规则：OSB 已覆盖的系统不估内部码偏 → 从初始 LSQ 剥离其 codeBias 变量；否则保留
                for (auto vit = it2->second.varCoeffData.begin();
                     vit != it2->second.varCoeffData.end();) {
                    if (vit->first.getParaType() == Parameter::codeBias &&
                        osbCoversSystem(vit->first.sat.system))
                        vit = it2->second.varCoeffData.erase(vit);
                    else
                        ++vit;
                }
                ++it2;
            }
        }
        mVarSet.clear();
        for (const auto &[eid, ed]: mEq.obsEquData)
            for (const auto &[v, c]: ed.varCoeffData) mVarSet.insert(v);
        {
            const auto blhAnchor = XYZToBLH(xc, Frame::WGS84); // B/L 为弧度
            const double cB = std::cos(blhAnchor.B()), sB = std::sin(blhAnchor.B());
            const double cL = std::cos(blhAnchor.L()), sL = std::sin(blhAnchor.L());
            const Eigen::Vector3d up(cB * cL, cB * sL, sB); // 测地垂直基向量
            const Variable vdx(obsData.station, Parameter::dX);
            const Variable vdy(obsData.station, Parameter::dY);
            const Variable vdz(obsData.station, Parameter::dZ);
            if (mPinInitHeight) {
                EquData ed;
                ed.prefit = 0.0;
                ed.varCoeffData[vdx] = up.x();
                ed.varCoeffData[vdy] = up.y();
                ed.varCoeffData[vdz] = up.z();
                ed.weight = 1.0 / (1e-2 * 1e-2); // σ=1cm：高程仍钉住(防冷启动高度↔ZTD 下坠)
                mEq.obsEquData[EquID(SatID('P', 0), std::string("INIT_UP"))] = ed;
            }
            mVarSet.insert(vdx);
            mVarSet.insert(vdy);
            mVarSet.insert(vdz);
        }
        // 统计真实卫星数（已排除伪卫星）：<4 不足以可靠三维定权，直接推迟
        int nRealSat = 0;
        {
            std::set<SatID> satsInEq;
            for (const auto &[eid, ed]: mEq.obsEquData)
                satsInEq.insert(eid.sat);
            nRealSat = static_cast<int>(satsInEq.size());
        }
        if (nRealSat < 4) return {mRefPos, false};
        mEq.varSet = mVarSet; // buildEquSys 只维护成员 mVarSet；SolverLSQ 读 equSys.varSet，需同步
        try {
            solver.solve(mEq);
        } catch (...) {
            return {mRefPos, false};
        }
        const double dx = solver.getSolution(Parameter::dX);
        const double dy = solver.getSolution(Parameter::dY);
        const double dz = solver.getSolution(Parameter::dZ);
        if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz)) return {mRefPos, false};
        const Eigen::Vector3d d(dx, dy, dz);
        xc += d;
        if (d.norm() < mConfig.convEps) break;
    }
    for (const auto &[sys, par]: sysCdtParam) {
        try { mClkBase[sys] = solver.getSolution(par); } catch (...) { mClkBase[sys] = 0.0; }
    }
    try {
        mZtdBase = solver.getSolution(Parameter::ztd);
        mZtdBaseValid = std::isfinite(mZtdBase);
    } catch (...) {
        mZtdBaseValid = false;
    }
    return {xc, true};
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
    const FreqCombo &def = *freqComboFor(sat); // ATX 缺失时 rcvPcvIF/satPcvIF 自动返回 0
    const double elev = satElevData.count(sat) ? satElevData.at(sat) : 0.0;
    const double azim = satAzimData.count(sat) ? satAzimData.at(sat) : 0.0;
    const PVT &pvt = satPVTRecTime.at(sat);
    const double zenDeg = 90.0 - elev * RAD_TO_DEG;
    const double azDeg = azim * RAD_TO_DEG;
    const Eigen::Vector3d recvEff = mEpochXyz + mEpochTide + mEpochPcoE;
    const Eigen::Vector3d nadir = -pvt.p.normalized();
    const Eigen::Vector3d toRecv = (recvEff - pvt.p).normalized();
    const double nadirDeg = std::acos(std::clamp(nadir.dot(toRecv), -1.0, 1.0)) * RAD_TO_DEG;
    return -(mAntx.rcvPcvIF(mRcvAntenna, sat.system, def, zenDeg, azDeg) + mAntx.satPcvIF(sat, def, nadirDeg));
}

void PPPStatic::addSatelliteEquations(const SatID &sat, const TypeValueMap &tv,
                                      double mapW, double tropoFixed) {
    const double map = mapW; // 湿映射用于 ZTD 状态系数
    const FreqCombo &def = *freqComboFor(sat);
    const Variable vclk(mCurObs->station, sysCdtParam.at(sat.system));
    const Variable vamb = makeAmbiguityVar(mCurObs->station, sat, "IF");
    const PVT &pvt = satPVTRecTime.at(sat);
    const double elev = satElevData.count(sat) ? satElevData.at(sat) : 0.0;
    const double azim = satAzimData.count(sat) ? satAzimData.at(sat) : 0.0;
    const CommonTime &epoch = mCurObs->epoch;
    const double cb = -mOsb.codeBiasAligned(def, sat);
    const double pb = mOsb.phaseBias(def, sat);
    const double pcv = pcvCorrection(sat);
    double ionoK = 0.0;
    if (!mIonex.empty()) {
        const auto ipp = Ionosphere::ippGeometry(mEpochXyz, pvt.p, elev, azim, mIonex.heightKm());
        if (double vtec = 0.0; mIonex.getTec(epoch, ipp.latDeg, ipp.lonDeg, vtec))
            ionoK = Ionosphere::secondOrderK(vtec, ipp, def.f1, def.f2);
    }
    const double L1m = tv.at(def.phase1);
    const double L2m = tv.at(def.phase2);
    const double P_IF = def.combineCodeFromObs(tv) + cb + pcv + ionoK;
    const double L_IF = def.combinePhase(L1m, L2m) - pb + pcv - ionoK * 0.5;

    // 公共钟模型下，非参考系统的先验钟 = 参考系统(公共)钟 + 自身 ISB
    const double clkRefPrior = mEkfInit
        ? (mClkEstimate.count('G') ? mClkEstimate.at('G') : 0.0)
        : mClkBase.count('G') ? mClkBase.at('G') : 0.0;
    double clkMean = mEkfInit
        ? (mClkEstimate.count(sat.system) ? mClkEstimate.at(sat.system) : 0.0)
        : mClkBase.count(sat.system) ? mClkBase.at(sat.system) : 0.0;
    if (mCommonClockActive && sat.system != 'G')
        clkMean += clkRefPrior;
    const double ztdMean = mEkfInit ? mZtdEstimate : mZtdBaseValid ? mZtdBase : mConfig.ztdInitValue;
    const double ambMean = mAmbEstimate.count(sat) ? mAmbEstimate.at(sat) : 0.0;
    const bool cbActive = mEkfInit && !osbCoversSystem(sat.system) &&
                          !(sat.system == 'R' && !osbCovers(sat));
    const double cbMean = cbActive
                              ? (mCbEstimate.count(sat.system) ? mCbEstimate.at(sat.system) : 0.0)
                              : 0.0;
    // 逐 GLONASS 星 IFCB
    const double ifcbMean = sat.system == 'R' && !osbCovers(sat) && mEkfInit
                                ? (mIfcbEstimate.count(sat) ? mIfcbEstimate.at(sat) : 0.0)
                                : 0.0;

    const double dts = pvt.clockBias * C_MPS;
    const double rel = pvt.relativityCorrection * C_MPS;
    const Eigen::Vector3d recvEff = mEpochXyz + mEpochTide + mEpochPcoE;
    double rho = 0.0;
    Eigen::Vector3d los;
    geometricRangeAndLos(pvt.p, recvEff, rho, los);

    // 相位缠绕修正
    Eigen::Vector3d exr, eyr;
    recvWindupBasis(mEpochXyz, exr, eyr);
    const double raw = Geodesy::phaseWindupCorrection(pvt.p, mEpochXyz, epoch, exr, eyr);
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
    const double codePrefit = P_IF - (rho - dts - rel) - clkMean - ztdMean * map - tropoFixed - cbMean - ifcbMean;
    const double phasePrefit = L_IF - (rho - dts - rel) - ambMean - clkMean - ztdMean * map - tropoFixed - windup_m; //NOLINT


    if (mProcEpoch > GATE_WARMUP && (std::abs(codePrefit) > rejCode || std::abs(phasePrefit) > rejPhase)) {
        satRejected.insert(sat);
        return;
    }
    {
        EquID eid(sat, def.code1 + "+" + def.code2);
        addObservationEquation(eid, codePrefit, los, mWeightCode, false, elev);
    }

    {
        EquID eid(sat, def.phase1 + "+" + def.phase2);
        addObservationEquation(eid, phasePrefit, los, mWeightPhase, true, elev);
    }
}

void PPPStatic::addObservationEquation(const EquID &eid, double prefit, const Eigen::Vector3d &los,
                                       const Weight &w, bool hasAmb, double elev) {
    // 位置/常值偏移状态变量恒由 station + 参数名确定，就地构造即可，无需跨函数共享
    const Variable vdx(mCurObs->station, Parameter::dX);
    const Variable vdy(mCurObs->station, Parameter::dY);
    const Variable vdz(mCurObs->station, Parameter::dZ);
    EquData ed;
    ed.prefit = prefit;
    ed.varCoeffData[vdx] = los[0];
    ed.varCoeffData[vdy] = los[1];
    ed.varCoeffData[vdz] = los[2];
    // 钟差状态：mCommonClockActive ⇒ 所有系统共享公共钟(cdt)，非参考系统再叠加 ISB(cdt2~4)；
    // 否则每系统独立绝对钟(sysCdtParam)。GPS 在两种口径下都只含 cdt ⇒ GPS-only 行为不变。
    if (mCommonClockActive) {
        const Variable vcommon(mCurObs->station, Parameter::cdt);
        ed.varCoeffData[vcommon] = 1.0;
        if (const Parameter isbPar = sysCdtParam.at(eid.sat.system); isbPar != Parameter::cdt) {  // 非参考系统再叠加 ISB 状态
            const Variable visb(mCurObs->station, isbPar);
            ed.varCoeffData[visb] = 1.0;
        }
    } else {
        const Variable vclk(mCurObs->station, sysCdtParam.at(eid.sat.system));
        ed.varCoeffData[vclk] = 1.0;
    }
    if (hasAmb) {
        const Variable vamb = makeAmbiguityVar(mCurObs->station, eid.sat, "IF");
        ed.varCoeffData[vamb] = 1.0;
        mVarSet.insert(vamb);
    } else if (eid.sat.system == 'R' && !osbCovers(eid.sat)) {
        // 逐 GLONASS 星 IFCB
        const Variable vifb = makeIfbVar(mCurObs->station, eid.sat, "IF");
        ed.varCoeffData[vifb] = 1.0;
        mVarSet.insert(vifb);
    } else if (!osbCoversSystem(eid.sat.system)) {
        // 逐系统码偏：仅当 OSB 未覆盖该系统时（OSB 已减码偏则跳过，避免共线）。
        const Variable vcb = makeCodeBiasVar(mCurObs->station, eid.sat, "IF");
        ed.varCoeffData[vcb] = 1.0;
        mVarSet.insert(vcb);
    }
    ed.weight = w.weight(elev);
    mEq.obsEquData[eid] = ed;
}

void PPPStatic::recvWindupBasis(const Eigen::Vector3d &recv, Eigen::Vector3d &exr, Eigen::Vector3d &eyr) const {
    if (mRecvVel.squaredNorm() > 1.0) {
        const Eigen::Vector3d ur = recv.normalized(); // 径向(地心向外)
        const Eigen::Vector3d ut = mRecvVel.normalized(); // 沿迹(速度方向)
        Eigen::Vector3d uc = ur.cross(ut);
        if (uc.squaredNorm() < 1e-6) uc = Eigen::Vector3d(1, 0, 0);
        uc.normalize();
        exr = ut;
        eyr = -uc;
        return;
    }
    auto blh = XYZToBLH(recv, Frame::WGS84);
    Eigen::Matrix3d E = ECEFToENUDCM(blh[0], blh[1]);
    exr = E.row(1).transpose();
    eyr = -E.row(0).transpose();
}

Eigen::Vector3d PPPStatic::computeTideDisplacement(const Eigen::Vector3d &xyzEst, const CommonTime &epoch) const {
    Eigen::Vector3d d = Eigen::Vector3d::Zero();
    if (mApplySolidTide) d = Geodesy::solidTideDisplacement(xyzEst, epoch);
    if (!mErp.empty()) {
        if (double xp = 0.0, yp = 0.0, ut1Utc = 0.0; mErp.getErp(epoch, xp, yp, ut1Utc))
            d += Geodesy::poleTideDisplacement(xyzEst, epoch, xp, yp);
    }
    return d;
}


double PPPStatic::computeIfAmbiguity(const SatID &sat, const TypeValueMap &tv,
                                     const double map, const double tropoFixed) const {
    const FreqCombo &def = *freqComboFor(sat);
    const PVT &pvt = satPVTRecTime.at(sat);
    const Eigen::Vector3d recvEff = mEpochXyz + mEpochTide + mEpochPcoE;
    const double rho0 = (pvt.p - recvEff).norm();
    const double dts0 = pvt.clockBias * C_MPS;
    const double rel0 = pvt.relativityCorrection * C_MPS;
    const double L1m0 = tv.at(def.phase1), L2m0 = tv.at(def.phase2);
    const double L_IF0 = def.combinePhase(L1m0, L2m0) - mOsb.phaseBias(def, sat)
                         + pcvCorrection(sat);
    // 与观测方程同源：首历元用 LSQ 引导的钟差/ZTD 先验，保证模糊度初值与首历元相位预fit=0 严格互补
    const double clk0 = mEkfInit
                            ? (mClkEstimate.count(sat.system) ? mClkEstimate.at(sat.system) : 0.0)
                            : mClkBase.count(sat.system) ? mClkBase.at(sat.system) : 0.0;
    const double ztd0 = mEkfInit ? mZtdEstimate : mZtdBaseValid ? mZtdBase : mConfig.ztdInitValue;
    const double windup0 = mApplyWindup && mWindupAcc.count(sat) ? def.lambdaN * mWindupAcc.at(sat) : 0.0;
    return L_IF0 - (rho0 - dts0 - rel0) - clk0 - ztd0 * map - tropoFixed - windup0;
}

void PPPStatic::resetAmbiguity(const SatID &sat, const TypeValueMap &tv,
                               const double map, const double tropoFixed,
                               const bool isSlipReset) {
    ++ambResets;
    const Variable vamb = makeAmbiguityVar(mCurObs->station, sat, "IF");
    (*mCsData)[vamb] = isSlipReset ? 2.0 : 1.0;
    if (isSlipReset) ++slipVerdicts;
    mAmbEstimate[sat] = computeIfAmbiguity(sat, tv, map, tropoFixed);
    if (isSlipReset) ekf.forceValue(vamb, mAmbEstimate[sat]);
}

void PPPStatic::detectCycleSlip(const SatID &sat, const TypeValueMap &tv,
                                bool &slipReset, bool &resetAmb) {
    const FreqCombo &def = *freqComboFor(sat);
    resetAmb = false;
    slipReset = false;
    if (!mEpochFirstIter) return;
    CycleSlip &det = mSlipDetectors[sat];
    det.configure(def);
    det.setGfThreshold(0.0);
    double osbP1 = 0.0, osbP2 = 0.0, osbL1 = 0.0, osbL2 = 0.0;
    mOsb.biases(def, sat, osbP1, osbP2, osbL1, osbL2);
    osbP1 = -osbP1;
    osbP2 = -osbP2;
    const double L1m = tv.at(def.phase1), L2m = tv.at(def.phase2);
    const double C1m = tv.at(def.code1), C2m = tv.at(def.code2);
    const CycleSlip::Verdict verdict = det.feed(L1m, L2m, C1m, C2m, true, osbP1, osbP2, osbL1, osbL2);
    const bool newArc = mPrevSatSet.find(sat) == mPrevSatSet.end();
    const bool mwSlip = det.mLastMwSlip;
    const bool gfSlip = det.mLastGfSlip;
    const bool gfArtifact = det.mLastGfArtifact;
    if (gfSlip) mGfEpoch.record(sat, gfArtifact);
    resetAmb = verdict == CycleSlip::Verdict::Gap || newArc || (mUseCycleSlip && mwSlip);
    slipReset = mUseCycleSlip && mwSlip;
    if (mwSlip) ++slipVerdicts;
    if (gfSlip) mGfSlipSats.insert(sat);
}

void PPPStatic::processSatellite(const SatID &sat, const TypeValueMap &tv) {
    const FreqCombo *defp = freqComboFor(sat);
    if (!defp) return;
    if (!defp->available(tv, true)) return;
    if (!satPVTRecTime.count(sat)) return;
    double elev = 0.0;
    if (const auto it = satElevData.find(sat); it != satElevData.end()) elev = it->second;
    if (elev < mConfig.cutoffElevRad) {
        satRejected.insert(sat);
        return;
    }

    double mW = 0.0;
    double mH = 0.0;
    if (mTropoMap == "nmf")
        mH = Troposphere::tropoNMF(elev, mEpochTropo.latDeg, mEpochTropo.hgt, mEpochTropo.doy, &mW);
    else {
        const double latR = mEpochTropo.latDeg * DEG_TO_RAD;
        const double lonR = mEpochTropo.lonDeg * DEG_TO_RAD;
        Troposphere::tropoGMF(latR, lonR, mEpochTropo.hgt, mEpochTropo.mjd, elev, &mH, &mW);
    }
    const double map = mW;
    double tropoFixed = mHasTropo ? mH * mEpochTropo.zhd : 0.0;
    if (mHasTropo && mUseTroGrad && (mEpochTropo.gradN != 0.0 || mEpochTropo.gradE != 0.0)) {
        const Eigen::Vector3d &p = satPVTRecTime.at(sat).p;
        const double azim = azimuth(mEpochXyz, p); // 自北顺时针（CoordConvert 轮子）
        tropoFixed += (mEpochTropo.gradN * std::cos(azim) + mEpochTropo.gradE * std::sin(azim)) / std::sin(elev);
    }
    mSatMap[sat] = map;
    mSatTropoFixed[sat] = tropoFixed;
    mSysSeen.insert(sat.system);

    {
        bool resetAmb = false;
        bool slipReset = false;
        detectCycleSlip(sat, tv, slipReset, resetAmb);
        if (mForceAmbReset) resetAmb = true;
        if (resetAmb) {
            resetAmbiguity(sat, tv, map, tropoFixed, slipReset);
        } else if (!mAmbEstimate.count(sat)) {
            mAmbEstimate[sat] = computeIfAmbiguity(sat, tv, map, tropoFixed);
        }
    }
    mCurrentSatSet.insert(sat);
    addSatelliteEquations(sat, tv, mW, tropoFixed);
}

void PPPStatic::buildEquSys(ObsData &obsData, const Eigen::Vector3d &xyzEst, VariableDataMap &csData, bool firstIter) {
    mEq.reset();
    mEq.station = obsData.station;
    mVarSet.clear();
    csData.clear();

    if (!mWeightInit) {
        const double minSin = std::sin(mConfig.cutoffElevRad);
        mWeightCode = Weight(mConfig.sigIFCode / std::sqrt(2.0), mConfig.sigIFCode / std::sqrt(2.0), minSin);
        mWeightPhase = Weight(mConfig.sigmaPhase / std::sqrt(2.0), mConfig.sigmaPhase / std::sqrt(2.0), minSin);
        mWeightInit = true;
    }

    const Variable vdx(obsData.station, Parameter::dX);
    const Variable vdy(obsData.station, Parameter::dY);
    const Variable vdz(obsData.station, Parameter::dZ);
    mVarSet.insert(vdx);
    mVarSet.insert(vdy);
    mVarSet.insert(vdz);

    resetSlipDetectors(obsData);

    // 填充每历元共享派生量
    mCurObs = &obsData;
    mEpochXyz = xyzEst;
    mEpochTide = computeTideDisplacement(xyzEst, obsData.epoch);
    mEpochPcoE = recvAntennaOffsetECEF(xyzEst, obsData.epoch);
    mEpochFirstIter = firstIter;
    mSatMap.clear();
    mSatTropoFixed.clear();
    mSysSeen.clear();
    mCurrentSatSet.clear();
    // 本历元是否启用"公共接收机钟(cdt)+每系统 ISB(cdt2~4)"状态模型
    mCommonClockActive = wantCommonClock(obsData);
    mCsData = &csData;
    mGfEpoch = CycleSlipEpoch{};
    mGfSlipSats.clear();

    // 对流层干/湿分离
    const auto blhEst = XYZToBLH(xyzEst, Frame::WGS84);
    mEpochTropo.latDeg = blhEst.B() * RAD_TO_DEG;
    mEpochTropo.lonDeg = blhEst.L() * RAD_TO_DEG;
    mEpochTropo.hgt = blhEst.H();
    const auto ydsTropo = CommonTime2YDSTime(obsData.epoch);
    mEpochTropo.doy = static_cast<double>(ydsTropo.doy);
    mEpochTropo.year = ydsTropo.year;
    MJD mjdStruct;
    CommonTime2MJD(obsData.epoch, mjdStruct);
    mEpochTropo.mjd = mjdStruct.mjd;
    mEpochTropo.zhd = Troposphere::tropoSaastamoinenZHD(blhEst.B(), blhEst.H());
    // TRO 水平梯度
    mEpochTropo.gradN = 0.0;
    mEpochTropo.gradE = 0.0;
    if (!mTro.empty()) {
        if (double gn = 0.0, ge = 0.0, sgn = 0.0, sge = 0.0;
            mTro.getTropoGrad(mCurObs->station, obsData.epoch, gn, ge, sgn, sge)) {
            mEpochTropo.gradN = gn;
            mEpochTropo.gradE = ge;
        }
    }

    for (const auto &[sat, tv]: obsData.satTypeValueData)
        processSatellite(sat, tv);

    {
        SatIDSet gfExcludeSet;
        const SatIDSet gfApplySet = mGfEpoch.decide(mGfCommonMode, gfExcludeSet);
        for (const auto &sat: gfApplySet) {
            const TypeValueMap &tv = obsData.satTypeValueData.at(sat);
            const double map = mSatMap.at(sat);
            resetAmbiguity(sat, tv, map, mSatTropoFixed.at(sat), true);
        }

        for (auto &[eid, ed]: mEq.obsEquData)
            if (gfExcludeSet.count(eid.sat))
                ed.weight = 1.0e-20;
        mForceAmbReset = false;
    }
    for (char sys: mSysSeen)
        if (auto it = sysCdtParam.find(sys); it != sysCdtParam.end())
            mVarSet.insert(Variable(obsData.station, it->second));

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

    const double zwd = Troposphere::tropoZwdPrior(mEpochTropo.latDeg, mEpochTropo.hgt, mEpochTropo.doy);
    double ztdTarget = zwd;
    double ztdSigma = mConfig.ztdPriorSigma;
    auto src = "ZTD_PRIOR";
    if (!mTro.empty() && mConfig.troZtdSigma > 0.0) {
        if (double ztdTro = 0.0, sigTro = 0.0; mTro.getZtd(mCurObs->station, mCurObs->epoch, ztdTro, sigTro)) {
            ztdTarget = ztdTro - mEpochTropo.zhd;
            ztdSigma = mConfig.troZtdSigma;
            src = "ZTD_TRO";
        }
    }
    if (ztdSigma > 0.0) {
        const EquID eid(SatID('T', 0), std::string(src));
        EquData ed;
        const double ztdCur = mEkfInit
                                  ? mZtdEstimate
                                  : mZtdBaseValid ? mZtdBase : mConfig.ztdInitValue;
        ed.prefit = ztdTarget - ztdCur;
        ed.varCoeffData[vztd] = 1.0;
        ed.weight = 1.0 / (ztdSigma * ztdSigma);
        mEq.obsEquData[eid] = ed;
    }
}

// 地基接收机天线 PCO
Eigen::Vector3d PPPStatic::recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst, const CommonTime &epoch) const {
    (void) epoch;
    Eigen::Vector3d recvPCOe = Eigen::Vector3d::Zero();
    if (!mRcvAntenna.empty()) {
        if (const Eigen::Vector3d pcoENU = mAntx.getRcvPCOENU(mRcvAntenna); pcoENU.squaredNorm() > 1e-12) {
            const auto blh = XYZToBLH(xyzEst, Frame::WGS84);
            const Eigen::Matrix3d E = ECEFToENUDCM(blh[0], blh[1]);
            recvPCOe = E.transpose() * pcoENU;
        }
    }
    return recvPCOe;
}

Eigen::Vector3d PPPStatic::predictPosition(const double dt, const CommonTime &epoch) const {
    (void) dt;
    (void) epoch;
    return currentEstimatedPosition();
}


void PPPStatic::readback(const ObsData &obsData) {
    try { result.xyz = currentEstimatedPosition(); } catch (...) {
    }
    result.xyzFloat = result.xyz; // 保留浮点(EKF)解：AR 成功后下方 result.xyz 会被固定解覆盖，供 GUI 浮点曲线对比
    result.blh = XYZToBLH(result.xyz, Frame::WGS84);
    result.numSats = static_cast<int>(mEq.obsEquData.size()) / 2;

    for (const auto &[sys, par]: sysCdtParam) {
        GnssEKF::tryGetSolution(par, ekf.currentUnkSet, ekf.solution, mClkEstimate[sys]);
    }
    // 内部码偏/IFCB 仅在 OSB 未覆盖时估过，回读须与「自动规则」一致
    for (char sys: mSysSeen) {
        if (!osbCoversSystem(sys)) {
            const Variable vcb = makeCodeBiasVar(obsData.station, SatID(sys, -1), "IF");
            mCbEstimate[sys] = ekf.getSolution(vcb); // 按 Variable 取：不在集中直接返回 0，不抛
        }
    }
    // 逐 GLONASS 星 IFCB 回读
    for (const auto &[sat, tv]: obsData.satTypeValueData) {
        if (sat.system != 'R') continue;
        if (osbCovers(sat)) continue;
        if (const FreqCombo *defp = freqComboFor(sat); !defp || !defp->available(tv, true)) continue;
        const Variable vifb = makeIfbVar(obsData.station, sat, "IF");
        mIfcbEstimate[sat] = ekf.getSolution(vifb);
    }
    GnssEKF::tryGetSolution(Parameter::ztd, ekf.currentUnkSet, ekf.solution, mZtdEstimate);
    for (const auto &[sat, tv]: obsData.satTypeValueData) {
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;
        if (!defp->available(tv, true)) continue;
        const Variable vamb = makeAmbiguityVar(obsData.station, sat, "IF");
        mAmbEstimate[sat] = ekf.getSolution(vamb); // 按 Variable 取：不在集中直接返回 0，不抛
    }

    if (const Eigen::VectorXd postfit = ekf.postfitResidual; postfit.size() > 0) {
        int iobs = 0;
        for (const auto &[eid, ed]: mEq.obsEquData) {
            if (iobs < postfit.size() && !eid.obsType.empty()) {
                if (eid.obsType[0] != 'L')
                    result.postRes[eid.sat] = postfit(iobs); // 码(伪距)残差
                else
                    result.postResPhase[eid.sat] = postfit(iobs); // 相位残差
            }
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
    const Eigen::MatrixXd P = ekf.covMatrix;
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
    result.sigmaP = std::max(result.sigmaP, 1e-3);
    result.sigmaPFloat = result.sigmaP;
}

void PPPStatic::computeDops() {
    Eigen::MatrixXd H;
    int n = 0;
    for (const auto &[eid, ed]: mEq.obsEquData) {
        if (eid.obsType.empty() || eid.obsType[0] == 'L') continue;
        double hrow[4] = {0, 0, 0, 0};
        for (const auto &[var, coeff]: ed.varCoeffData) {
            if (const auto pt = var.getParaType(); pt == Parameter::dX) hrow[0] = coeff;
            else if (pt == Parameter::dY) hrow[1] = coeff;
            else if (pt == Parameter::dZ) hrow[2] = coeff;
            else if (pt == Parameter::cdt || pt == Parameter::cdt2 ||
                     pt == Parameter::cdt3 || pt == Parameter::cdt4)
                hrow[3] = coeff;
        }
        H.conservativeResize(n + 1, 4);
        H.row(n) << hrow[0], hrow[1], hrow[2], hrow[3];
        ++n;
    }
    if (n >= 4) {
        const Eigen::MatrixXd HtH = H.transpose() * H;
        if (const Eigen::FullPivLU<Eigen::MatrixXd> lu(HtH); lu.isInvertible()) {
            const Eigen::MatrixXd inv = lu.inverse();
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

    if (ifCodeTypes.empty() && mGloCombo.empty()) return;


    if (mHasLastEpoch) {
        if (const double gap = std::fabs(obsData.epoch - mLastEpochTime);
            (mNominalInterval > 0.0 && gap > 1.5 * mNominalInterval) || gap > 60.0) {
            // 间隙恢复：默认清全部模糊度（RTKLIB 口径）；keepAmbAcrossGap 时保留，
            // 只让 CycleSlip 检测器判真实周跳——用于"数据中断但未失锁"的场景。
            if (!(mConfig.keepAmbMaxGap > 0.0 && gap <= mConfig.keepAmbMaxGap)) mForceAmbReset = true;
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

    if (!mEkfInit && satPVTTransTime.size() < 4) {
        mFilterFrozen = true;
    } else if (!mEkfInit) {
        Eigen::Vector3d xyzRef = mRefPos;
        bool deferInit = false;
        {
            if (auto [pos, flag] = solveInitialPosition(obsData); !flag) deferInit = true;
            else xyzRef = pos;
            mAmbEstimate.clear();
        }
        if (deferInit) {
            mFilterFrozen = true;
        } else {
            buildEquSys(obsData, xyzRef, csData, true);
            mFilterFrozen = static_cast<int>(mEq.obsEquData.size()) / 2 < mConfig.minSatsForUpdate;
            if (!mFilterFrozen) {
                ekf.configure(mConfig.posNoise, mHasTropo, mConfig.ambNoise, mConfig.clockNoise);
                ekf.isbProcNoise = mConfig.isbProcNoise;   // ISB 过程噪声(0=常数)
                ekf.isbInitVar  = mConfig.isbInitVar;       // ISB 初方差
                ekf.posVarFloor = mConfig.posVarFloor; // 位置后验方差下限：防 East 近退化方向被钉初值偏置
                ekf.ztdInitVar = mConfig.ztdInitVar;
                ekf.ambInitVar = mConfig.ambInitVar; // 模糊度初方差(RTKLIB 用 3600，我们默认 100)
                ekf.ztdProcNoise = mConfig.ztdProcNoise;
                double fpvH = mConfig.firstEpochPosVar;
                ekf.posVarFloorVec = Eigen::Vector3d(mConfig.posVarFloor, mConfig.posVarFloor, mConfig.posVarFloor);
                ekf.ztdInitValue = mConfig.ztdInitValue;
                ekf.codeBiasProcNoise = mConfig.codeBiasProcNoise;
                ekf.codeBiasInitVar = mConfig.codeBiasInitVar;
                ekf.timeUpdate(mVarSet, &csData);
                ekf.setBootstrapPosition(xyzRef, Eigen::Vector3d(fpvH, fpvH, mConfig.firstEpochPosVar));
                {
                    double refClk = 0.0;
                    for (const auto &[s, p]: sysCdtParam)
                        if (p == Parameter::cdt && mClkBase.count(s)) { refClk = mClkBase.at(s); break; }
                    std::map<Variable, double> bootVals;
                    std::map<Variable, double> bootVars;
                    for (const auto &[sys, par]: sysCdtParam) {
                        if (const auto it = mClkBase.find(sys); it != mClkBase.end()) {
                            const double v = par == Parameter::cdt ? it->second : it->second - refClk;
                            bootVals[Variable(obsData.station, par)] = v;
                            // 参考系统钟差用 clockInitVar(白噪声)；ISB 载体(cdt2~4)用 isbInitVar(常数/随机游走)。
                            const bool isbOn = par != Parameter::cdt && ekf.isbInitVar > 0.0;
                            bootVars[Variable(obsData.station, par)] =
                                isbOn ? ekf.isbInitVar : ekf.clockInitVar;
                        }
                    }
                    if (mHasTropo) {
                        bootVals[Variable(obsData.station, Parameter::ztd)] =
                                mZtdBaseValid ? mZtdBase : mConfig.ztdInitValue;
                        bootVars[Variable(obsData.station, Parameter::ztd)] = mConfig.ztdInitVar;
                    }
                    if (!bootVals.empty()) ekf.setBootstrapStates(bootVals, &bootVars);
                }
                ekf.setBootstrapAmbiguity(obsData.station, mAmbEstimate, nullptr);
                ekf.measUpdate(mEq);
                mEkfInit = true;
            }
        }
    } else {
        double dt = 0.0;
        if (mHasLastEpoch) {
            dt = obsData.epoch - mLastEpochTime;
            if (dt < 0.0) dt = 0.0;
        }
        const Eigen::Vector3d xyzEst = predictPosition(dt, obsData.epoch);
        buildEquSys(obsData, xyzEst, csData, true);
        mFilterFrozen = static_cast<int>(mEq.obsEquData.size()) / 2 < mConfig.minSatsForUpdate;
        if (!mFilterFrozen) {
            ekf.timeUpdate(mVarSet, &csData);
            {
                // 第一遍：各系统用自身码观测求重锚量 dclk
                std::map<char, double> sysDclk;
                double dclkRef = 0.0;
                bool haveRef = false;
                for (const auto &[sys, par]: sysCdtParam) {
                    double sw = 0.0, swp = 0.0;
                    for (const auto &[eid, ed]: mEq.obsEquData) {
                        if (eid.sat.system != sys) continue;
                        if (eid.sat.id == 0) continue; // 伪观测(ZTD 先验等)
                        if (eid.obsType.empty() || eid.obsType[0] == 'L') continue; // 仅码
                        sw += ed.weight;
                        swp += ed.weight * ed.prefit;
                    }
                    if (!(sw > 0.0)) continue;
                    const double dclk = swp / sw;
                    if (!std::isfinite(dclk) || std::fabs(dclk) > 1.0e4) continue; // 防护：异常历元不重锚
                    if (std::fabs(dclk) < 1e-12) continue;
                    sysDclk[sys] = dclk;
                    if (par == Parameter::cdt) { dclkRef = dclk; haveRef = true; }
                }
                if (!haveRef && !sysDclk.empty()) {
                    double sw = 0.0, swp = 0.0;
                    for (const auto &[s, d]: sysDclk) { sw += 1.0; swp += d; }
                    dclkRef = swp / sw;
                }
                // 第二遍：预fit 扣减 + 重锚
                if (mCommonClockActive) {
                    // 公共钟 + ISB 重锚
                    const Variable vcommon(obsData.station, Parameter::cdt);
                    for (auto &[eid, ed]: mEq.obsEquData) {
                        if (const auto it = ed.varCoeffData.find(vcommon); it != ed.varCoeffData.end())
                            ed.prefit -= dclkRef * it->second; // 所有星方程均含 cdt(系数1)，整体扣 dclkRef
                    }
                    ekf.shiftState(vcommon, dclkRef);
                    for (char sys: mSysSeen) {
                        const Parameter par = sysCdtParam.at(sys);
                        if (par == Parameter::cdt) continue; // 参考系统无 ISB
                        const auto itd = sysDclk.find(sys);
                        if (itd == sysDclk.end()) continue;
                        const double disb = itd->second - dclkRef; // 相对参考系统的钟差偏差
                        const Variable visb(obsData.station, par);
                        for (auto &[eid, ed]: mEq.obsEquData) {
                            if (eid.sat.system != sys) continue;
                            if (const auto it = ed.varCoeffData.find(visb); it != ed.varCoeffData.end())
                                ed.prefit -= disb * it->second;
                        }
                        ekf.shiftState(visb, disb);
                    }
                } else {
                    // 旧口径：各系统独立绝对钟
                    for (const auto &[sys, par]: sysCdtParam) {
                        const Variable vclk(obsData.station, par);
                        for (auto &[eid, ed]: mEq.obsEquData) {
                            if (const auto it = ed.varCoeffData.find(vclk); it != ed.varCoeffData.end())
                                ed.prefit -= dclkRef * it->second;
                        }
                        ekf.shiftState(vclk, dclkRef);
                    }
                }
            }
            // 位置先验 = 本历元线性化点（同源；静态方差不动，运动学由派生类在 timeUpdate 前后另行调 setBootstrapPosition）。
            ekf.setBootstrapPosition(xyzEst, 0.0);
            ekf.setBootstrapAmbiguity(obsData.station, mAmbEstimate, &csData);
            ekf.savePrior();
            ekf.measUpdate(mEq);
        }
    }
    if (mFilterFrozen) {
        result.xyz = Eigen::Vector3d::Zero();
        result.numSats = 0;
        xyz = currentEstimatedPosition();
        computeElevAzim();
        return;
    }
    mLastEpochTime = obsData.epoch;
    mHasLastEpoch = true;
    mGapThisEpoch = false;


    const bool arFixed = mUseAR && tryFixAmbiguities(obsData);
    result.stat = arFixed ? 1 : 2; // 1=Fixed 2=Float


    readback(obsData);

    if (arFixed) result.xyz = result.xyzFixed; // AR 成功后输出固定解（cm 级）
    xyz = result.xyz;
    computeElevAzim();
}


bool PPPStatic::tryFixAmbiguities(ObsData &obsData) {
    mArRatio = 0.0;
    mArNumFixed = 0;
    result.xyzFixed = Eigen::Vector3d::Zero();
    ++mArAttempts;
    if (!mEkfInit) {
        return false;
    }

    const Eigen::VectorXd x = ekf.solution;
    const Eigen::MatrixXd P = ekf.covMatrix;

    // 收集候选星：IF 齐备、星历可用、模糊度已入状态、方差低于门限
    std::vector<SatID> sats;
    std::vector<int> idx;
    std::vector<double> Bf;
    int inState = 0;
    double sMin = 1e9, sMax = -1.0, sSum = 0.0;
    for (const auto &[sat, tv]: obsData.satTypeValueData) {
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;
        if (!defp->available(tv, true)) continue;
        if (satPVTRecTime.count(sat) == 0) continue;
        const Variable vamb = makeAmbiguityVar(obsData.station, sat, "IF");
        if (!ekf.currentIndexData.count(vamb)) continue;
        const int i = ekf.currentIndexData.at(vamb);
        const double sig = std::sqrt(P(i, i));
        ++inState;
        if (sig < sMin) sMin = sig;
        if (sig > sMax) sMax = sig;
        sSum += sig;
        if (sig > mConfig.arSigGate) continue;
        sats.push_back(sat);
        idx.push_back(i);
        Bf.push_back(x(i));
    }
    const int n = static_cast<int>(sats.size());
    if (n < 5) return false;

    Eigen::VectorXd a(2 * n);
    Eigen::MatrixXd Q(2 * n, 2 * n);
    Q.setZero();
    std::vector<double> lam_n(n), cFac(n); // cFac = f2/(f1−f2)
    for (int k = 0; k < n; ++k) {
        const SatID &sat = sats[k];
        const TypeValueMap &tv = obsData.satTypeValueData.at(sat);
        const FreqCombo &def = *freqComboFor(sat);
        const double f1 = def.f1, f2 = def.f2;
        lam_n[k] = def.lambdaN; // 窄巷波长(m)
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
        if (std::abs(Bf[k]) < 0.5) {
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
        const double N1 = Bf[k] / lam_n[k] - cFac[k] * NwVal;
        a(2 * k) = NwVal;
        a(2 * k + 1) = N1;
        Q(2 * k, 2 * k) = varNwVal;
        Q(2 * k + 1, 2 * k + 1) = P(idx[k], idx[k]) / (lam_n[k] * lam_n[k]);
        for (int j = 0; j < n; ++j)
            if (j != k)
                Q(2 * k + 1, 2 * j + 1) = P(idx[k], idx[j]) / (lam_n[k] * lam_n[j]);
    }

    const auto [afix, ratio, fixed] = resolveWithRatio(a, Q, mConfig.arRatioThreshold);
    mArRatio = ratio;
    result.ratio = ratio;
    if (!fixed) return false;

    Eigen::VectorXd z(n);
    for (int k = 0; k < n; ++k) {
        const SatID &sat = sats[k];
        const FreqCombo &def = *freqComboFor(sat);
        const double f1 = def.f1, f2 = def.f2;
        const double N1f = afix(2 * k + 1), Nwf = afix(2 * k);
        const double Bfix = C_MPS * ((f1 - f2) * N1f + f2 * Nwf) / (f1 * f1 - f2 * f2);
        z(k) = Bfix;
    }

    const int Nst = static_cast<int>(x.size());
    Eigen::MatrixXd H(n, Nst);
    H.setZero();
    for (int k = 0; k < n; ++k) H(k, idx[k]) = 1.0;
    constexpr double varFix = 1.0e-10;
    const Eigen::MatrixXd R = varFix * Eigen::MatrixXd::Identity(n, n);
    const Eigen::MatrixXd S = H * P * H.transpose() + R;
    const Eigen::MatrixXd Km = P * H.transpose() * S.inverse();
    const Eigen::VectorXd xf = x + Km * (z - H * x);

    Eigen::Vector3d xyzFixed(
        xf(ekf.currentIndexData.at(Variable(obsData.station, Parameter::dX))),
        xf(ekf.currentIndexData.at(Variable(obsData.station, Parameter::dY))),
        xf(ekf.currentIndexData.at(Variable(obsData.station, Parameter::dZ))));
    result.xyzFixed = xyzFixed;
    {
        const int ix = ekf.currentIndexData.at(Variable(obsData.station, Parameter::dX));
        const int iy = ekf.currentIndexData.at(Variable(obsData.station, Parameter::dY));
        const int iz = ekf.currentIndexData.at(Variable(obsData.station, Parameter::dZ));
        const Eigen::MatrixXd Pf = (Eigen::MatrixXd::Identity(Nst, Nst) - Km * H) * P;
        const double vpos = Pf(ix, ix) + Pf(iy, iy) + Pf(iz, iz);
        result.sigmaPFixed = std::sqrt(std::max(vpos, 0.0));
    }
    mArNumFixed = n;
    ++mArSuccess;
    return true;
}

PPPStatic::~PPPStatic() = default;
