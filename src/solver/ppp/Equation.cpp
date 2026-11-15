#include "Static.h"
#include "core/Const.h"
#include "core/geom/Geodesy.h"
#include "core/atmo/Ionosphere.h"
#include "core/geom/CoordConvert.h"
#include "core/obs/Weight.h"
#include <algorithm>
#include <cmath>

void PPPStatic::resetSlipDetectors() {
    for (const auto &sat: mPrevSatSet) {
        if (mCurObs->satTypeValueData.find(sat) == mCurObs->satTypeValueData.end())
            mSlipDetectors[sat].reset();
    }
}

double PPPStatic::pcvCorrection(const SatID &sat) const {
    const FreqCombo &def = *freqComboFor(sat);
    const double elev = satElevData.count(sat) ? satElevData.at(sat) : 0.0;
    const double azim = satAzimData.count(sat) ? satAzimData.at(sat) : 0.0;
    const PVT &pvt = satPVTRecTime.at(sat);
    const double zenDeg = 90.0 - elev * RAD_TO_DEG;
    const double azDeg = azim * RAD_TO_DEG;
    const Eigen::Vector3d recvEff = recvEffFor(sat);
    const Eigen::Vector3d nadir = -pvt.p.normalized();
    const Eigen::Vector3d toRecv = (recvEff - pvt.p).normalized();
    const double nadirDeg = std::acos(std::clamp(nadir.dot(toRecv), -1.0, 1.0)) * RAD_TO_DEG;
    const CommonTime t = mCurObs ? mCurObs->epoch : CommonTime();
    return -(prod->atx.rcvPCVIF(mRcvAntenna, sat.system, def, zenDeg, azDeg, t)
             + prod->atx.satPCVIF(sat, def, nadirDeg, t));
}

void PPPStatic::addSatEquation(const SatID &sat, const TypeValueMap &tv, const bool probe) {
    const FreqCombo &def = *freqComboFor(sat);
    const double mapW = mSatMap.at(sat);
    const double tropoFixed = mSatTropoFixed.at(sat);
    const Variable vamb = makeAmbiguityVar(mCurObs->station, sat, "IF");
    const PVT &pvt = satPVTRecTime.at(sat);
    const double elev = satElevData.count(sat) ? satElevData.at(sat) : 0.0;
    const double azim = satAzimData.count(sat) ? satAzimData.at(sat) : 0.0;
    const CommonTime &epoch = mCurObs->epoch;
    const double cb = -prod->osb.codeBiasAligned(def, sat, osbStation());
    const double pb = prod->osb.phaseBias(def, sat, osbStation());
    const double pcv = pcvCorrection(sat);
    double ionoK = 0.0;
    if (!prod->ion.empty()) {
        const auto ipp = Ionosphere::ippGeometry(mEpochXyz, pvt.p, elev, azim, prod->ion.heightKm());
        if (double vtec = 0.0; prod->ion.getTec(epoch, ipp.latDeg, ipp.lonDeg, vtec))
            ionoK = Ionosphere::secondOrderK(vtec, ipp, def.f1, def.f2);
    }
    const double L1m = tv.at(def.phase1);
    const double L2m = tv.at(def.phase2);
    const double P_IF = def.combineCodeFromObs(tv) + cb + pcv + ionoK;
    const double L_IF = def.combinePhase(L1m, L2m) - pb + pcv - ionoK * 0.5;

    const double clkRefPrior = clockPrior(mClockRefSys);
    double clkMean = clockPrior(sat.system);
    if (mCommonClockActive && sat.system != mClockRefSys)
        clkMean += clkRefPrior;
    const double ztdMean = ztdPrior();
    const double ambMean = mAmbEstimate.count(sat) ? mAmbEstimate.at(sat) : 0.0;
    const bool cbActive = mFilterInit && !osbCoversSystem(sat.system) &&
                          !((sat.system == 'R' || sat.system == 'C') && !osbCovers(sat));
    const double cbMean = cbActive
                              ? (mCbEstimate.count(sat.system) ? mCbEstimate.at(sat.system) : 0.0)
                              : 0.0;
    // 逐 GLONASS/BDS 星 IFCB：GLONASS 是 FDMA 逐通道偏差，BDS-3 的 B1I+B2b 配对与精密钟的
    // B1I/B3I 基准差一个不可广播改正的硬件延迟——两者都以逐星状态吸收，不让偏置漏进位置。
    const double ifcbMean = (sat.system == 'R' || sat.system == 'C') && !osbCovers(sat) && mFilterInit
                                ? (mIfcbEstimate.count(sat) ? mIfcbEstimate.at(sat) : 0.0)
                                : 0.0;

    const double dts = pvt.clockBias * C_MPS;
    const double rel = pvt.relativityCorrection * C_MPS;
    const Eigen::Vector3d recvEff = recvEffFor(sat);
    double rho = 0.0;
    Eigen::Vector3d los;
    Coord::geometricRangeAndLos(pvt.p, recvEff, rho, los);

    Eigen::Vector3d exr, eyr;
    recvWindupBasis(mEpochXyz, exr, eyr);
    DCM attM;
    const DCM *att = prod->att.bodyDCM(sat, mEpochTropo.mjd, attM) ? &attM : nullptr;
    const double raw = Geodesy::phaseWindupCorrection(pvt.p, mEpochXyz, epoch, exr, eyr, att);
    const double prev = mWindupPrev.count(sat) ? mWindupPrev[sat] : raw;
    double dphi = raw - prev;
    while (dphi > 0.5) dphi -= 1.0;
    while (dphi < -0.5) dphi += 1.0;
    const double acc = (mWindupAcc.count(sat) ? mWindupAcc[sat] : 0.0) + dphi;
    mWindupPrev[sat] = raw;
    mWindupAcc[sat] = acc;
    const double windup_m = mApplyWindup ? def.lambdaN * acc : 0.0;

    const double codePrefit = P_IF - (rho - dts - rel) - clkMean - ztdMean * mapW - tropoFixed - cbMean - ifcbMean;
    const double phasePrefit = L_IF - (rho - dts - rel) - ambMean - clkMean - ztdMean * mapW - tropoFixed - windup_m;

    if (probe) {
        mPrefitProbe[sat] = PrefitRec{codePrefit, phasePrefit,
                                      mCsData && mCsData->count(vamb) > 0};
        return;
    }

    if (mPrefitBad.count(sat)) {
        satRejected.insert(sat);
        return;
    }
    {
        EquID eid(sat, def.code1 + "+" + def.code2);
        addObservationEquation(eid, codePrefit, los, mWeightCode, false, elev);
    }

    if (mPrefitPhaseBad.count(sat)) {
        satRejected.insert(sat);
        return;
    }
    {
        EquID eid(sat, def.phase1 + "+" + def.phase2);
        addObservationEquation(eid, phasePrefit, los, mWeightPhase, true, elev);
    }
}

void PPPStatic::addObservationEquation(const EquID &eid, double prefit, const Eigen::Vector3d &los,
                                       const Weight &w, bool hasAmb, double elev) {
    const Variable vdx(mCurObs->station, Parameter::dX);
    const Variable vdy(mCurObs->station, Parameter::dY);
    const Variable vdz(mCurObs->station, Parameter::dZ);
    EquData ed;
    ed.prefit = prefit;
    ed.varCoeffData[vdx] = los[0];
    ed.varCoeffData[vdy] = los[1];
    ed.varCoeffData[vdz] = los[2];
    addClockTerms(ed, eid.sat);
    if (hasAmb) {
        const Variable vamb = makeAmbiguityVar(mCurObs->station, eid.sat, "IF");
        ed.varCoeffData[vamb] = 1.0;
        curEq.varSet.insert(vamb);
    } else if ((eid.sat.system == 'R' || eid.sat.system == 'C') && !osbCovers(eid.sat)) {
        // 逐 GLONASS/BDS 星 IFCB
        const Variable vifb = makeIfbVar(mCurObs->station, eid.sat, "IF");
        ed.varCoeffData[vifb] = 1.0;
        curEq.varSet.insert(vifb);
    } else if (!osbCoversSystem(eid.sat.system)) {
        // 逐系统码偏
        const Variable vcb = makeCodeBiasVar(mCurObs->station, eid.sat, "IF");
        ed.varCoeffData[vcb] = 1.0;
        curEq.varSet.insert(vcb);
    }
    ed.weight = w.weight(elev);
    curEq.obsEquData[eid] = ed;
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
    auto blh = Coord::XYZToBLH(recv, Frame::WGS84);
    const DCM E = DCM::fromECEFToENU(blh[0], blh[1]);
    exr = E.row(1);
    eyr = -E.row(0);
}

Eigen::Vector3d PPPStatic::computeTideDisplacement(const Eigen::Vector3d &xyzEst, const CommonTime &epoch) const {
    Eigen::Vector3d d = Eigen::Vector3d::Zero();
    if (mApplySolidTide) d = Geodesy::solidTideDisplacement(xyzEst, epoch);
    if (!prod->erp.empty()) {
        if (double xp = 0.0, yp = 0.0, ut1Utc = 0.0; prod->erp.getERP(epoch, xp, yp, ut1Utc))
            d += Geodesy::poleTideDisplacement(xyzEst, epoch, xp, yp);
    }
    return d;
}

double PPPStatic::computeIfAmbiguity(const SatID &sat, const TypeValueMap &tv,
                                     const double map, const double tropoFixed) const {
    const FreqCombo &def = *freqComboFor(sat);
    const PVT &pvt = satPVTRecTime.at(sat);
    const Eigen::Vector3d recvEff = recvEffFor(sat);
    const double rho0 = (pvt.p - recvEff).norm();
    const double dts0 = pvt.clockBias * C_MPS;
    const double rel0 = pvt.relativityCorrection * C_MPS;
    const double L1m0 = tv.at(def.phase1), L2m0 = tv.at(def.phase2);
    const double L_IF0 = def.combinePhase(L1m0, L2m0) - prod->osb.phaseBias(def, sat, osbStation())
                         + pcvCorrection(sat);
    // 模糊度初值与首历元相位预fit 严格互补。
    const double clk0 = clockPrior(sat.system) +
                         ((mCommonClockActive && sat.system != mClockRefSys) ? clockPrior(mClockRefSys) : 0.0);
    const double ztd0 = ztdPrior();
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
    CycleSlip &det = mSlipDetectors[sat];
    det.configure(def, 0.0, mGfSlipMult); // PPP 默认仅用 MW（GF 关闭）
    double osbP1 = 0.0, osbP2 = 0.0, osbL1 = 0.0, osbL2 = 0.0;
    prod->osb.biases(def, sat, osbP1, osbP2, osbL1, osbL2, osbStation());
    osbP1 = -osbP1;
    osbP2 = -osbP2;

    const CycleSlip::Result r = det.feed(sat, tv, mCurObs, osbP1, osbP2, osbL1, osbL2);
    mGfEpoch.record(sat, r);
    const bool newArc = mPrevSatSet.find(sat) == mPrevSatSet.end();
    resetAmb = r.gap || newArc || (mUseCycleSlip && r.slip);
    slipReset = mUseCycleSlip && r.slip;
    if (r.slip) ++slipVerdicts;
}

void PPPStatic::prepareSatellite(const SatID &sat, const TypeValueMap &tv) {
    const FreqCombo *defp = freqComboFor(sat);
    if (!defp) return;
    if (!defp->available(tv, true)) return;
    if (!satPVTRecTime.count(sat)) return;
    double elev = 0.0;
    if (const auto it = satElevData.find(sat); it != satElevData.end()) elev = it->second;
    if (elev < mConfig.cutoffElevRad && !mSkipElevCutoff) {
        satRejected.insert(sat);
        return;
    }

    double mW = 0.0;
    double map = 0.0;
    double tropoFixed = 0.0;
    if (mHasTropo) {
        const double azim = mUseTroGrad && (mEpochTropo.gradN != 0.0 || mEpochTropo.gradE != 0.0)
                                ? Coord::azimuth(mEpochXyz, satPVTRecTime.at(sat).p)
                                : 0.0;
        tropoFixed = mEpochTropo.dryDelay(mConfig.tropoMap, elev, &mW, azim, mUseTroGrad);
    } else {
        mW = mEpochTropo.mapping(mConfig.tropoMap, elev);
    }
    map = mW;
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
}

// 接收机 PCO 按该卫星系统的 IF 组合逐星取。
Eigen::Vector3d PPPStatic::recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst, const CommonTime &epoch,
                                                  const SatID &sat) const {
    if (mRcvAntenna.empty() || prod->atx.empty()) return Eigen::Vector3d::Zero();
    const FreqCombo *combo = freqComboFor(sat);
    if (!combo) return Eigen::Vector3d::Zero();
    const Eigen::Vector3d pcoENU = prod->atx.getRcvPCOENU(mRcvAntenna, sat.system, *combo, epoch);
    if (pcoENU.squaredNorm() < 1e-12) return Eigen::Vector3d::Zero();
    const auto blh = Coord::XYZToBLH(xyzEst, Frame::WGS84);
    return DCM::fromECEFToENU(blh[0], blh[1]).transpose() * pcoENU;
}

Eigen::Vector3d PPPStatic::recvEffFor(const SatID &sat) const {
    const CommonTime epoch = mCurObs ? mCurObs->epoch : CommonTime();
    return mEpochXyz + mEpochTide + recvAntennaOffsetECEF(mEpochXyz, epoch, sat);
}
