#include "PPP.h"
#include "Const.h"
#include "ARLambda.h"
#include "Log.h"
#include "Geodesy.h"
#include "CoordConvert.h"
#include <Eigen/Eigen>
#include <map>
#include <set>
#include <algorithm>

bool PPP::loadSp3(const std::string &path) {
    return mSp3.read(path);
}

bool PPP::loadClk(const std::string &path) {
    return mClk.read(path);
}

bool PPP::loadAtx(const std::string &path) {
    return mAntx.read(path);
}

bool PPP::loadOsb(const std::string &path) {
    return mOsb.read(path);
}

bool PPP::process(ObsData &obs) {
    try {
        if (mRcvAntenna.empty() && !obs.antType.empty()) mRcvAntenna = obs.antType;
        preprocess(obs);
        mFilterFrozen = false;
        solve(obs);
    } catch (const std::exception &e) {
        LOG_ERROR << "[PPP] solve 异常: " << e.what();
        mGapThisEpoch = false;
        return false;
    }
    if (mFilterFrozen) return false;
    return true;
}

void PPP::computeSatPos(ObsData &obsData) {
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
        //todo 超快sp3里有钟差数据，这个需要兼容
        if (mSp3.contains(sat) && mClk.contains(sat)) {
            pvt = mSp3.getPVT(sat, t_emit);
            pvt.clockBias = mClk.getClockBias(sat, t_emit);
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
void PPP::bootstrapFirstEpoch(ObsData &obsData) {
    if (!mFirstEpoch) return;
    try { SPP::solve(obsData); } catch (...) {
    }
    for (const auto &[id, bias]: rClockBias) mClkBase[id] = bias;
    const Vector3d ap = obsData.antennaPosition;
    mRefPos = ap.norm() > 1e6 ? ap : xyz;
    mFirstEpoch = false;
}

// 设置各系统绝对接收机钟差先验：首历元用 SPP 引导的 mClkBase，其后用 KF 后验 mClkEstimate
void PPP::applyPriorClocks() {
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
Vector3d PPP::currentEstimatedPosition() const {
    return {
        GnssEKF::getSolution(Parameter::dX, ekf.currentUnkSet, ekf.getState()),
        GnssEKF::getSolution(Parameter::dY, ekf.currentUnkSet, ekf.getState()),
        GnssEKF::getSolution(Parameter::dZ, ekf.currentUnkSet, ekf.getState())
    };
}

// 重置周跳探测器
void PPP::resetSlipDetectors(const ObsData &obsData) {
    for (const auto &sat: mPrevSatSet) {
        if (obsData.satTypeValueData.find(sat) == obsData.satTypeValueData.end())
            mSlipDetectors[sat].reset();
    }
}

// 单星码/相位两个方程
double PPP::pcvCorrection(const SatID &sat, const FreqCombo &def, const PVT &pvt,
                          const Vector3d &xyzEst, const Vector3d &tideDisp,
                          const Vector3d &recvPCOe, double elev, double azim) const {
    if (!mApplyPcv) return 0.0;
    constexpr double R2D = 180.0 / PI;
    // 接收机 PCV：以本地天顶距(90°-高度角)和方位角查表
    const double zenDeg = 90.0 - elev * R2D;
    const double azDeg = azim * R2D;
    // 卫星 PCV：以星下点角（卫星天底方向与"卫星->接收机"方向的夹角）查表
    const Vector3d recvEff = xyzEst + tideDisp + recvPCOe;
    const Vector3d nadir = -pvt.p.normalized();
    const Vector3d toRecv = (recvEff - pvt.p).normalized();
    const double nadirDeg = std::acos(std::clamp(nadir.dot(toRecv), -1.0, 1.0)) * R2D;
    return mAntx.rcvPcvIF(mRcvAntenna, sat.system, def, zenDeg, azDeg)
           + mAntx.satPcvIF(sat, def, nadirDeg);
}

void PPP::addSatelliteEquations(const SatID &sat, const FreqCombo &def, const TypeValueMap &tv,
                                const PVT &pvt, const Vector3d &xyzEst, double elev, double azim, double map,
                                const Variable &vclk, const Variable &vdx, const Variable &vdy,
                                const Variable &vdz, const Variable &vamb,
                                const Vector3d &tideDisp, const Vector3d &recvPCOe,
                                const CommonTime &epoch,
                                const Variable *vox, const Variable *voy, const Variable *voz) {
    const double cb = mOsb.codeBias(def, sat);
    const double pb = mOsb.phaseBias(def, sat);
    const double pcv = pcvCorrection(sat, def, pvt, xyzEst, tideDisp, recvPCOe, elev, azim);
    const double P_IF = def.combineCodeFromObs(tv) - cb + pcv;
    const double L1m = tv.at(def.phase1);
    const double L2m = tv.at(def.phase2);
    const double L_IF = def.combinePhase(L1m, L2m) - pb + pcv;

    const double clkMean = mEkfInit ? (mClkEstimate.count(sat.system) ? mClkEstimate.at(sat.system) : 0.0) : 0.0;
    // 首历元用 ZTD 先验(= Hopfield 干+湿天顶延迟)建模对流层，避免 ztd 误差灌进位置导致 UP 大幅偏差；
    // 收敛后(mEkfInit)沿用 ZTD 后验 mZtdEstimate。模糊度 bootstrap 的 ztd0 保持 0，ztd 只经此处 prefit，不双重计数。
    const double ztdMean = mEkfInit ? mZtdEstimate : mZtdInitValue;
    const double ambMean = mAmbEstimate.count(sat) ? mAmbEstimate.at(sat) : 0.0;

    const double dts = pvt.clockBias * C_MPS;
    const double rel = pvt.relativityCorrection * C_MPS;
    const Vector3d recvEff = xyzEst + tideDisp + recvPCOe;
    const double rho = (pvt.p - recvEff).norm();
    const Vector3d los = -(pvt.p - recvEff) / rho;

    // 相位缠绕修正
    Vector3d exr, eyr;
    recvWindupBasis(sat, xyzEst, epoch, exr, eyr);
    const double raw = Geodesy::phaseWindupCorrection(pvt.p, pvt.v, xyzEst, epoch, exr, eyr);
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
    const double codePrefit = P_IF - (rho - dts - rel) - clkMean - ztdMean * map;
    const double phasePrefit = L_IF - (rho - dts - rel) - ambMean - clkMean - ztdMean * map - windup_m; //NOLINT
    if (mProcEpoch > GATE_WARMUP && (std::abs(codePrefit) > rejCode || std::abs(phasePrefit) > rejPhase)) {
        satRejected.insert(sat);
        return;
    }
    {
        EquID eid(sat, def.code1 + "+" + def.code2);
        EquData ed;
        ed.prefit = P_IF - (rho - dts - rel) - clkMean - ztdMean * map;
        ed.varCoeffData[vdx] = los[0];
        ed.varCoeffData[vdy] = los[1];
        ed.varCoeffData[vdz] = los[2];
        ed.varCoeffData[vclk] = 1.0;
        if (vox) {
            ed.varCoeffData[*vox] = los[0];
            ed.varCoeffData[*voy] = los[1];
            ed.varCoeffData[*voz] = los[2];
        }
        double w = 1.0 / (sigIFCode * sigIFCode);
        if (elev < PI / 6) {
            const double s = std::sin(elev);
            w *= s * s;
        }
        ed.weight = w;
        mEq.obsEquData[eid] = ed;
    }

    {
        EquID eid(sat, def.phase1 + "+" + def.phase2);
        EquData ed;
        ed.prefit = L_IF - (rho - dts - rel) - ambMean - clkMean - ztdMean * map - windup_m;
        ed.varCoeffData[vdx] = los[0];
        ed.varCoeffData[vdy] = los[1];
        ed.varCoeffData[vdz] = los[2];
        ed.varCoeffData[vclk] = 1.0;
        ed.varCoeffData[vamb] = 1.0;
        if (vox) {
            ed.varCoeffData[*vox] = los[0];
            ed.varCoeffData[*voy] = los[1];
            ed.varCoeffData[*voz] = los[2];
        }
        mVarSet.insert(vamb);
        double w = 1.0 / (mSigmaPhase * mSigmaPhase);
        if (elev < PI / 6) {
            const double s = std::sin(elev);
            w *= s * s;
        }
        ed.weight = w;
        mEq.obsEquData[eid] = ed;
    }
}

void PPP::recvWindupBasis(const SatID &, const Vector3d &recv, const CommonTime &,
                          Vector3d &exr, Vector3d &eyr) const {
    auto blh = XYZtoBLH(recv, Frame::WGS84);
    Matrix3d E = getBLMatrix(blh[0], blh[1]);
    exr = E.row(1).transpose();
    eyr = -E.row(0).transpose();
}

void PPP::buildEquSys(ObsData &obsData, const Vector3d &xyzEst, VariableDataMap &csData, bool firstIter) {
    mEq.reset();
    mEq.station = obsData.station;
    mVarSet.clear();
    csData.clear();

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

    std::set<char> sysSeen;
    std::map<SatID, double> satMap;
    std::set<SatID> currentSatSet;

    resetSlipDetectors(obsData);

    Vector3d tideDisp = Vector3d::Zero();
    if (mApplySolidTide)
        tideDisp = Geodesy::solidTideDisplacement(xyzEst, obsData.epoch);

    Vector3d recvPCOe = recvAntennaOffsetECEF(xyzEst, obsData.epoch);

    for (const auto &[sat, tv]: obsData.satTypeValueData) {
        if (!ifCodeTypes.count(sat.system)) continue;
        const FreqCombo &def = ifCodeTypes.at(sat.system);
        if (!def.available(tv, true)) continue; // 需码 + 相位齐备（单频 BDS 自动排除）
        if (!satPVTRecTime.count(sat)) continue;
        double elev = 0.0;
        if (auto it = satElevData.find(sat); it != satElevData.end()) elev = it->second;
        if (elev < cutOffElev) {
            satRejected.insert(sat);
            continue;
        }

        const PVT &pvt = satPVTRecTime.at(sat);
        double azim = 0.0;
        if (auto it = satAzimData.find(sat); it != satAzimData.end()) azim = it->second;
        const double map = 1.0 / std::sin(std::sqrt(std::max(elev, 0.02) * std::max(elev, 0.02) + TROPO_MAP_EPS));
        satMap[sat] = map;
        const Parameter clkPar = sat.system == 'G' ? Parameter::cdt : Parameter::cdt2;
        const Variable vclk(obsData.station, clkPar);
        sysSeen.insert(sat.system);

        const double C1m = tv.at(def.code1);
        const double C2m = tv.at(def.code2);
        const double L1m = tv.at(def.phase1);
        const double L2m = tv.at(def.phase2);
        const Variable vamb(obsData.station, sat, Parameter::ambiguity,
                            ObsID(std::string(1, sat.system), "IF"));
        bool resetAmb = false;
        if (firstIter) {
            CycleSlip &det = mSlipDetectors[sat];
            det.configure(def);
            double osbP1 = 0.0, osbP2 = 0.0, osbL1 = 0.0, osbL2 = 0.0;
            mOsb.biases(def, sat, osbP1, osbP2, osbL1, osbL2);
            const CycleSlip::Verdict verdict = det.feed(L1m, L2m, C1m, C2m, true, osbP1, osbP2, osbL1, osbL2);
            const bool newArc = mPrevSatSet.find(sat) == mPrevSatSet.end();
            resetAmb = verdict == CycleSlip::Verdict::Gap || newArc ||
                       (mUseCycleSlip && verdict == CycleSlip::Verdict::Slip);
            if (verdict == CycleSlip::Verdict::Slip) ++slipVerdicts;
        }
        if (mForceAmbReset) resetAmb = true;
        if (resetAmb) {
            ++ambResets;
            csData[vamb] = 1.0;
        }
        // 模糊度均值初始化
        if (resetAmb || !mAmbEstimate.count(sat)) {
            const Vector3d recvEff = xyzEst + recvAntennaOffsetECEF(xyzEst, obsData.epoch);
            const double rho0 = (pvt.p - recvEff).norm();
            const double dts0 = pvt.clockBias * C_MPS;
            const double rel0 = pvt.relativityCorrection * C_MPS;
            const double L1m0 = tv.at(def.phase1), L2m0 = tv.at(def.phase2);
            const double L_IF0 = def.combinePhase(L1m0, L2m0) - mOsb.phaseBias(def, sat)
                                 + pcvCorrection(sat, def, pvt, xyzEst, tideDisp, recvPCOe, elev, azim);
            const double clk0 = mEkfInit ? (mClkEstimate.count(sat.system) ? mClkEstimate.at(sat.system) : 0.0) : 0.0;
            const double ztd0 = mEkfInit ? mZtdEstimate : 0.0;
            const double map0 = 1.0 / std::sin(std::sqrt(std::max(elev, 0.02) * std::max(elev, 0.02) + TROPO_MAP_EPS));
            mAmbEstimate[sat] = L_IF0 - (rho0 - dts0 - rel0) - clk0 - ztd0 * map0;
        }
        currentSatSet.insert(sat);

        addSatelliteEquations(sat, def, tv, pvt, xyzEst, elev, azim, map, vclk, vdx, vdy, vdz, vamb,
                              tideDisp, recvPCOe, obsData.epoch,
                              mEstimateConstOffset ? &vox : nullptr,
                              mEstimateConstOffset ? &voy : nullptr,
                              mEstimateConstOffset ? &voz : nullptr);
    }
    mForceAmbReset = false;
    for (char sys: sysSeen)
        mVarSet.insert(Variable(obsData.station, sys == 'G' ? Parameter::cdt : Parameter::cdt2));
    if (mHasTropo) {
        const Variable vztd(obsData.station, Parameter::ztd);
        mVarSet.insert(vztd);
        for (auto &[eid, ed]: mEq.obsEquData) {
            const double map = satMap.count(eid.sat) ? satMap.at(eid.sat) : 1.0;
            ed.varCoeffData[vztd] = map;
        }
    }

    mPrevSatSet = std::move(currentSatSet);
}

// 地基接收机天线 PCO
Vector3d PPP::recvAntennaOffsetECEF(const Vector3d &xyzEst, const CommonTime &epoch) const {
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
Vector3d PPP::predictPosition(const double dt, const CommonTime &epoch) const {
    (void) dt;
    (void) epoch;
    return currentEstimatedPosition();
}

void PPP::readback(const ObsData &obsData) {
    result.xyz = currentEstimatedPosition();
    mConstOffset = Vector3d::Zero();
    try { mConstOffset.x() = GnssEKF::getSolution(Parameter::dOX, ekf.currentUnkSet, ekf.getState()); } catch (...) {
    }
    try { mConstOffset.y() = GnssEKF::getSolution(Parameter::dOY, ekf.currentUnkSet, ekf.getState()); } catch (...) {
    }
    try { mConstOffset.z() = GnssEKF::getSolution(Parameter::dOZ, ekf.currentUnkSet, ekf.getState()); } catch (...) {
    }
    if (mEstimateConstOffset) result.xyz += mConstOffset;
    result.blh = XYZtoBLH(result.xyz, Frame::WGS84);
    result.numSats = static_cast<int>(mEq.obsEquData.size()) / 2;

    for (const auto &[id, _]: ifCodeTypes) {
        const Parameter clkPar = id == 'G' ? Parameter::cdt : Parameter::cdt2;
        try { mClkEstimate[id] = GnssEKF::getSolution(clkPar, ekf.currentUnkSet, ekf.getState()); } catch (...) {
        }
    }
    try { mZtdEstimate = GnssEKF::getSolution(Parameter::ztd, ekf.currentUnkSet, ekf.getState()); } catch (...) {
    }
    for (const auto &[sat, tv]: obsData.satTypeValueData) {
        if (!ifCodeTypes.count(sat.system)) continue;
        if (!ifCodeTypes.at(sat.system).available(tv, true)) continue;
        const Variable vamb(obsData.station, sat, Parameter::ambiguity,
                            ObsID(std::string(1, sat.system), "IF"));
        try { mAmbEstimate[sat] = ekf.getSolution(vamb); } catch (...) {
        }
    }

    // 逐星码残差（后验残差）
    if (const VectorXd postfit = ekf.getPostfitResidual(); postfit.size() > 0) {
        int iobs = 0;
        for (const auto &[eid, ed]: mEq.obsEquData) {
            if (iobs < postfit.size() && !eid.obsType.empty() && eid.obsType[0] != 'L')
                result.postRes[eid.sat] = postfit(iobs);
            ++iobs;
        }
    }

    // PDOP
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
        if (FullPivLU<MatrixXd> lu(HtH); lu.isInvertible()) {
            const MatrixXd inv = lu.inverse();
            result.pdop = std::sqrt(inv(0, 0) + inv(1, 1) + inv(2, 2));
            result.gdop = std::sqrt(inv.trace());
            result.tdop = std::sqrt(inv(3, 3));
            const Matrix3d C_ecef = inv.topLeftCorner(3, 3);
            const Matrix3d R = getBLMatrix(result.blh[0], result.blh[1]);
            const Matrix3d C_enu = R * C_ecef * R.transpose();
            result.hdop = std::sqrt(C_enu(0, 0) + C_enu(1, 1));
            result.vdop = std::sqrt(C_enu(2, 2));
        }
    }

    const Variable vdx(obsData.station, Parameter::dX);
    const Variable vdy(obsData.station, Parameter::dY);
    const Variable vdz(obsData.station, Parameter::dZ);
    const MatrixXd P = ekf.getCovMatrix();
    double sx = 0, sy = 0, sz = 0;
    if (ekf.currentIndexData.count(vdx))
        sx = std::sqrt(P(ekf.currentIndexData.at(vdx), ekf.currentIndexData.at(vdx)));
    if (ekf.currentIndexData.count(vdy))
        sy = std::sqrt(P(ekf.currentIndexData.at(vdy), ekf.currentIndexData.at(vdy)));
    if (ekf.currentIndexData.count(vdz))
        sz = std::sqrt(P(ekf.currentIndexData.at(vdz), ekf.currentIndexData.at(vdz)));
    result.sigmaP = std::sqrt(sx * sx + sy * sy + sz * sz);
    result.sigmaP = std::max(result.sigmaP, result.pdop * 0.3);
}

void PPP::solve(ObsData &obsData) {
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

    VariableDataMap csData;
    if (!mEkfInit) {
        const Vector3d xyzRef = mFirstEpochSPP
                                    ? predictPosition(0.0, obsData.epoch)
                                    : mRefPos;
        // 对流层 ZTD 先验：默认用 Hopfield 干+湿天顶延迟（与站高相关）作初值，
        // 避免 ZTD 从 0 起始造成 UP 方向十几米级的收敛摆荡。用户显式 setZtdInitValue(>0) 时保留其设定。
        if (mZtdInitValue <= 0.0) {
            const double H = XYZtoBLH(xyzRef, Frame::WGS84).H();
            mZtdInitValue = tropoHopfieldDry(H) + tropoHopfieldWet(H);
        }
        buildEquSys(obsData, xyzRef, csData, true);
        mFilterFrozen = static_cast<int>(mEq.obsEquData.size()) / 2 < mMinSatsForUpdate;
        if (!mFilterFrozen) {
            ekf.configure(mPosNoise, mHasTropo, mAmbNoise, mClockNoise);
            ekf.setZtdInitVar(mZtdInitVar);
            ekf.setZtdProcNoise(mZtdProcNoise);
            ekf.setZtdInitValue(mZtdInitValue);
            ekf.timeUpdate(mVarSet, &csData);
            ekf.setBootstrapPosition(xyzRef, mFirstEpochPosVar);
            ekf.setBootstrapAmbiguity(obsData.station, mAmbEstimate);
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
        mFilterFrozen = static_cast<int>(mEq.obsEquData.size()) / 2 < mMinSatsForUpdate;
        if (!mFilterFrozen) {
            ekf.timeUpdate(mVarSet, &csData);
            ekf.setBootstrapPosition(xyzEst);
            ekf.setBootstrapAmbiguity(obsData.station, mAmbEstimate);
            ekf.measUpdate(mEq);
            readback(obsData);
        }
    }
    if (mFilterFrozen) {
        result.xyz = Vector3d::Zero();
        result.numSats = 0;
        xyz = currentEstimatedPosition();
        computeElevAzim();
        return;
    }
    mLastEpochTime = obsData.epoch; // 记录本历元时刻，供下次 dt 计算与观测中断检测
    mHasLastEpoch = true;
    mGapThisEpoch = false; // 标志已被 LEO::predictPosition 消费，复位供下历元重新判定

    afterMeasUpdate(obsData); // STEP2：AR 固定（默认空；mUseAR 时写入 result.xyzFixed）
    readback(obsData);
    xyz = currentEstimatedPosition();
    computeElevAzim();
}

void PPP::afterMeasUpdate(ObsData &obsData) {
    if (!mUseAR) return;
    tryFixAmbiguities(obsData);
}

bool PPP::tryFixAmbiguities(ObsData &obsData) {
    mArRatio = 0.0;
    mArNumFixed = 0;
    result.xyzFixed = Vector3d::Zero();
    ++mArAttempts;
    if (!mEkfInit) return false;

    const VectorXd x = ekf.getState();
    const MatrixXd P = ekf.getCovMatrix();
    std::vector<SatID> sats;
    std::vector<int> idx;
    std::vector<double> Bf;
    std::vector<Variable> vambVec;
    for (const auto &[sat, tv]: obsData.satTypeValueData) {
        if (!ifCodeTypes.count(sat.system)) continue;
        if (const FreqCombo &def = ifCodeTypes.at(sat.system); !def.available(tv, true)) continue;
        if (satPVTRecTime.count(sat) == 0) continue;
        const Variable vamb(obsData.station, sat, Parameter::ambiguity, ObsID(std::string(1, sat.system), "IF"));
        if (!ekf.currentIndexData.count(vamb)) continue;
        const int i = ekf.currentIndexData.at(vamb);
        if (const double sig = std::sqrt(P(i, i)); sig > mArSigGate) continue;
        sats.push_back(sat);
        idx.push_back(i);
        Bf.push_back(x(i));
        vambVec.push_back(vamb);
    }
    const int n = static_cast<int>(sats.size());
    if (n < 5) return false;

    VectorXd a(2 * n);
    MatrixXd Q(2 * n, 2 * n);
    Q.setZero();
    std::vector<double> lam_n(n), varNw(n), Nw(n), cFac(n); // cFac = f2/(f1−f2)
    for (int k = 0; k < n; ++k) {
        const SatID &sat = sats[k];
        const TypeValueMap &tv = obsData.satTypeValueData.at(sat);
        const FreqCombo &def = ifCodeTypes.at(sat.system);
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
        const double sL = mSigmaPhase, sP = sigIFCode;
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
        Nw[k] = NwVal;
        varNw[k] = varNwVal;
        const double N1 = Bf[k] / lam_n[k] - cFac[k] * NwVal;
        a(2 * k) = NwVal;
        a(2 * k + 1) = N1;
        Q(2 * k, 2 * k) = varNwVal;
        Q(2 * k + 1, 2 * k + 1) = P(idx[k], idx[k]) / (lam_n[k] * lam_n[k]);
        for (int j = 0; j < n; ++j)
            if (j != k)
                Q(2 * k + 1, 2 * j + 1) = P(idx[k], idx[j]) / (lam_n[k] * lam_n[j]);
    }

    ARLambda ar;
    const VectorXd afix = ar.resolve(a, Q);
    mArRatio = ar.squaredRatio;
    if (!ar.isFixed(mArRatioThreshold)) return false;
    VectorXd z(n);
    for (int k = 0; k < n; ++k) {
        const SatID &sat = sats[k];
        const FreqCombo &def = ifCodeTypes.at(sat.system);
        const double f1 = def.f1, f2 = def.f2;
        const double N1f = afix(2 * k + 1), Nwf = afix(2 * k);
        //const double N2f = N1f - Nwf;
        const double Bfix = C_MPS * ((f1 - f2) * N1f + f2 * Nwf) / (f1 * f1 - f2 * f2);
        mAmbFixed[sat] = Bfix;
        z(k) = Bfix;
    }

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
    result.xyzFixed = xyzFixed;
    result.ratio = ar.squaredRatio;
    mArNumFixed = n;
    ++mArSuccess;
    return true;
}
