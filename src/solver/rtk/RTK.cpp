#include "RTK.h"
#include "core/equ/Precision.h"
#include "core/obs/Weight.h"
#include "core/Const.h"
#include "rinex/RINEXNavStore.h"
#include "ARLambda.h"
#include "SPP.h"
#include "core/geom/CoordConvert.h"
#include "core/geom/CoordStruct.h"
#include "Log.h"

#include <algorithm>

namespace {
    /// 常速度预测的最长跨历元间隔：超过此时速度已不代表本历元运动状态（与 SPP 同口径）
    /// 引导历元位置先验 σ：松到让首历元量测主导，又不至于让 CV 的零速度初值拖住解
    constexpr double kBootPosVar = 10.0 * 10.0;
    /// 滤波档抗差门限（按新息 σ 的倍数）：先验位置由运动模型给，运动时它本身就差几十厘米，
    /// 用固定米级门限剔星等于把整批观测当粗差丢掉。按 sqrt(先验方差+观测方差) 定倍数才有意义。
    constexpr double kPrefitGateSigma = 4.0;
    constexpr std::size_t kGateMinObs = 8;
}


Eigen::Vector3d RTK::estimateApproxPosition(const ObsData &obs, const EphemerisTable &eph,
                                            const std::set<char> &systems, const double cutoffElevDeg) {
    if (eph.gps.empty() && eph.bds.empty()) return Eigen::Vector3d::Zero();
    SPP spp;
    spp.prod->eph = eph;
    spp.setEnabledSystems(systems);
    spp.setCutoffElevDeg(cutoffElevDeg);
    ObsData work = obs;
    work.antennaPosition = Eigen::Vector3d::Zero();
    try { if (spp.processEpoch(work)) return spp.result.xyz; } catch (...) {
    }
    return Eigen::Vector3d::Zero();
}

bool RTK::processEpoch(const ObsData &base, const ObsData &rover, EphemerisTable &eph) {
    result.reset(); // 早退分支不能留下上一历元的结果
    mWeightReady = false; // 随机模型按本历元卫星集合重算
    if (!mUseFilter && mFilterInit) resetFilterState();

    if (mBaseUser.norm() > 1e3) {
        mBase = mBaseUser;
    } else if (base.antennaPosition.norm() > 1e3) {
        mBase = base.antennaPosition;
    } else {
        if (const std::size_t es = eph.gps.size() + eph.bds.size() + eph.gal.size() + eph.glo.size();
            mBase.norm() < 1e3 || es > mBaseEphSize) {
            const Eigen::Vector3d p = estimateApproxPosition(base, eph, mConfig.enabledSystems,
                                                             mConfig.cutoffElevRad * RAD_TO_DEG);
            if (p.norm() > 1e3) {
                mBase = p;
                mBaseEphSize = es;
            }
        }
    }
    if (mBase.norm() < 1e3) {
        LOG_WARN << "[RTK] base position not available (no BESTPOS/header and SPP fallback failed)";
        return false;
    }

    if (rover.antennaPosition.norm() > 1e3) {
        mRoverApprox = rover.antennaPosition;
    } else if (mUseFilter && mFilterInit && xyz.norm() > 1e3) {
        mRoverApprox = xyz;
    } else {
        if (const std::size_t es = eph.gps.size() + eph.bds.size() + eph.gal.size() + eph.glo.size();
            mRoverApprox.norm() < 1e3 || es > mRoverEphSize) {
            const Eigen::Vector3d p = estimateApproxPosition(rover, eph, mConfig.enabledSystems,
                                                             mConfig.cutoffElevRad * RAD_TO_DEG);
            if (p.norm() > 1e3) {
                mRoverApprox = p;
                mRoverEphSize = es;
            }
        }
    }

    mRtkBase = &base;
    mCurObs = &rover;
    mEph = &eph;

    detectFreqCombinations();
    detectSlips();

    prepareGeometry();
    if (static_cast<int>(mUsedSats.size()) < mConfig.minSats) return false; // 参与双差的星不足，本历元不给解
    computeSdVariance();

    solve();
    if (result.stat == 0) return false;
    mPrevRefSat = mRefSat;
    return true;
}

void RTK::publishResult() {
    if (xyz.norm() < 1.0e6) return;

    result.xyz = xyz;
    result.blh = Coord::XYZToBLH(xyz, Frame::WGS84);
    result.xyzFloat = xyz;
    result.numSats = static_cast<int>(mUsedSats.size());
    mBaselineFloat = xyz - mBase;

    const Eigen::Matrix3d Q = posCovariance();
    result.sigmaXYZ = Eigen::Vector3d(std::sqrt((std::max)(Q(0, 0), 0.0)),
                                      std::sqrt((std::max)(Q(1, 1), 0.0)),
                                      std::sqrt((std::max)(Q(2, 2), 0.0)));
    result.sigmaP = positionSigma(Q, sigma0);
    result.sigmaPFloat = result.sigmaP;
    result.ratio = 0.0;
    result.stat = 2;

    readback(curEq, ekf.postfitResidual, xyz);

    if (mConfig.useAR) {
        Eigen::VectorXd b;
        Eigen::MatrixXd Qb;
        std::vector<Variable> aVars;
        extractAmbiguities(b, Qb, aVars);
        if (tryFixAmbiguities(aVars, b, Qb)) {
            applyFixedSolution(sigma0);
            return;
        }
    }
    result.xyzFixed = result.xyz;
    result.blhFixed = result.blh;
}

void RTK::resetFilterState() {
    ekf.reset();
    mFilterInit = false;
    xyz.setZero();
    mPrevRefSat.clear();
}

void RTK::initFilter() {
    Eigen::Vector3d baseline;
    if (!solveFloat(baseline)) {
        xyz.setZero();
        return;
    }
    xyz = mBase + baseline;

    ekf.reset();
    ekf.configure(mPosProcSigma, false, mConfig.ambProcSigma, 0.0);
    ekf.accProcSigma = 0.0;
    ekf.ambInitSigma = mConfig.ambInitSigma;

    AmbSeed seed;
    VariableSet varSet;
    collectEpochStates(varSet, seed);
    ekf.timeUpdate(varSet, nullptr);
    ekf.setBootstrapPosition(xyz, Eigen::Vector3d::Constant(kBootPosVar));
    ekf.setBootstrapStates(seed.init);

    buildEquSys(curEq, xyz, false);
    ekf.measUpdate(curEq, mConfig.fullCovariance ? &mWeightMatrix : nullptr);
    xyz = filteredPosition();
    sigma0 = posteriorSigma0(ekf.postfitResidual, curEq.weightVector(),
                             static_cast<int>(ekf.currentUnkSet.size()));
    mFilterInit = true;
}

void RTK::rebaseAmbiguities() {
    if (mPrevRefSat.empty() || ekf.currentUnkSet.empty()) return;

    const VariableSet &oldSet = ekf.currentUnkSet;
    std::map<Variable, int> oldIndex;
    for (const auto &v: oldSet) oldIndex[v] = ekf.indexOf(v);

    const auto refChanged = [&](const char sys) {
        const auto n = mRefSat.find(sys), o = mPrevRefSat.find(sys);
        return n != mRefSat.end() && o != mPrevRefSat.end() && n->second != o->second;
    };

    std::map<Variable, std::pair<int, int> > rowMap;
    //先给所有非模糊度变量和未变系统的模糊度写恒等行
    for (const auto &v: oldSet) {
        if (v.par != Parameter::ambiguity || !refChanged(v.obsID.satSys[0]))
            rowMap[v] = {oldIndex.at(v), -1};
    }

    bool anyChanged = false;
    for (const auto &[sys, sats]: mPartSats) {
        if (!refChanged(sys)) continue;
        anyChanged = true;
        const auto b = mRefSat.at(sys);
        const auto a = mPrevRefSat.at(sys);
        for (int f = 1; f <= 2; ++f) {
            const auto itb = oldIndex.find(ambVar(b, f));
            if (itb == oldIndex.end()) continue; // 新参考星上历元无状态：整系统按新弧段重开
            for (const SatID &sat: sats) {
                if (sat == b) continue;
                const Variable vi = ambVar(sat, f);
                if (sat == a) {
                    rowMap[vi] = {-1, itb->second};
                } else if (const auto iti = oldIndex.find(vi); iti != oldIndex.end()) {
                    rowMap[vi] = {iti->second, itb->second};
                }
            }
        }
    }
    if (!anyChanged) return;

    VariableSet newOrder;
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(static_cast<int>(rowMap.size()), static_cast<int>(oldSet.size()));
    int r = 0;
    for (const auto &[v, cols]: rowMap) {
        newOrder.insert(v);
        const auto [plusCol, minusCol] = cols;
        if (plusCol >= 0) T(r, plusCol) = 1.0;
        if (minusCol >= 0) T(r, minusCol) = -1.0;
        ++r;
    }
    ekf.reparameterize(newOrder, T);
}

void RTK::updateFilter() {
    rebaseAmbiguities();
    AmbSeed seed;
    VariableSet varSet;
    collectEpochStates(varSet, seed);
    for (const auto &[v, n]: seed.init) ekf.forceValue(v, n);
    ekf.timeUpdate(varSet, &seed.novar);
    const Eigen::Vector3d xPred = predictedPosition();
    buildEquSys(curEq, xPred, false);
    // 粗差门限按星成组剔
    const Eigen::VectorXd sigPred = ekf.priorInnovationStd(curEq);
    std::set<SatID> bad;
    int irow = 0;
    for (const auto &[eid, ed]: curEq.obsEquData) {
        const double sigObs = std::sqrt(1.0 / ed.weight);
        if (const double sig = irow < sigPred.size() ? std::hypot(sigPred(irow), sigObs) : sigObs;
            std::fabs(ed.prefit) > kPrefitGateSigma * sig)
            bad.insert(eid.sat);
        ++irow;
    }
    if (!bad.empty()) {
        EquSys gated = curEq;
        for (auto it = gated.obsEquData.begin(); it != gated.obsEquData.end();)
            it = bad.count(it->first.sat) ? gated.obsEquData.erase(it) : std::next(it);
        if (gated.obsEquData.size() >= kGateMinObs) {
            curEq = std::move(gated);
            mWeightReady = false; // 行数变了，权阵必须按新行序重建
            buildWeightMatrix(curEq);
        }
    }
    ekf.measUpdate(curEq, mConfig.fullCovariance ? &mWeightMatrix : nullptr);
    xyz = filteredPosition();
    sigma0 = posteriorSigma0(ekf.postfitResidual, curEq.weightVector(),
                             static_cast<int>(ekf.currentUnkSet.size()));
}

void RTK::applyFixedSolution(const double s0) {
    result.xyz = mBase + mBaselineFixed;
    result.blh = Coord::XYZToBLH(result.xyz, Frame::WGS84);
    result.xyzFixed = result.xyz;
    result.blhFixed = result.blh;
    result.sigmaXYZ = mBaselineSigmaFixed;
    Eigen::Matrix3d Qfix = Eigen::Matrix3d::Zero();
    for (int k = 0; k < 3; ++k) Qfix(k, k) = mBaselineSigmaFixed[k] * mBaselineSigmaFixed[k];
    result.sigmaP = positionSigma(Qfix, s0);
    result.sigmaPFixed = result.sigmaP; // 固定 σ 单独留存
    result.stat = 1; // Fixed
}

Eigen::Vector3d RTK::filteredPosition() const {
    return {
        GnssEKF::getSolution(Parameter::dX, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dY, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dZ, ekf.currentUnkSet, ekf.solution)
    };
}

Eigen::Vector3d RTK::predictedPosition() const {
    return {
        ekf.priorOf(Variable("rover", Parameter::dX)),
        ekf.priorOf(Variable("rover", Parameter::dY)),
        ekf.priorOf(Variable("rover", Parameter::dZ))
    };
}

void RTK::extractAmbiguities(Eigen::VectorXd &b, Eigen::MatrixXd &Qb, std::vector<Variable> &aVars) const {
    // 只取本历元确实有观测的星：门限剔除后留下的变量没有方程，取整会让 LAMBDA 选错
    std::set<SatID> used;
    for (const auto &[eid, ed]: curEq.obsEquData) used.insert(eid.sat);

    aVars.clear();
    std::vector<int> idx;
    for (const auto &v: curEq.varSet) {
        if (v.par != Parameter::ambiguity || !used.count(v.sat)) continue;
        const int i = ekf.indexOf(v);
        if (i < 0) continue;
        aVars.push_back(v);
        idx.push_back(i);
    }

    const int M = static_cast<int>(aVars.size());
    b.resize(M);
    Qb.resize(M, M);
    for (int i = 0; i < M; ++i) {
        b(i) = ekf.solution(idx[i]);
        for (int j = 0; j < M; ++j) Qb(i, j) = ekf.Q(idx[i], idx[j]);
    }
}

void RTK::detectFreqCombinations() {
    mRtkCombos.clear();
    for (char sys: mConfig.enabledSystems) {
        SysCombo sc;
        sc.base = FreqCombo::detect(sys, *mRtkBase, true);
        sc.rover = FreqCombo::detect(sys, *mCurObs, true);
        if (!sc.base.code1.empty() && !sc.rover.code1.empty()) sc.ok = true;
        if (!sc.ok) continue;
        mRtkCombos[sys] = sc;
    }
}

//构建双差几何、选参考星、记录参与卫星
void RTK::prepareGeometry() {
    mUsedSats.clear();
    satElevData.clear();
    satAzimData.clear();
    satBaseElevData.clear();
    satPVTTransTime.clear();
    satPVTRecTime.clear();
    satPVTBaseRecTime.clear();
    mSdVar.clear();
    mPartSats.clear();
    mRefSat.clear();

    // 流动站概略坐标拿不到时按基准站处理，与 buildEquSys 的迭代起点一致
    const Eigen::Vector3d &roverRef = mRoverApprox.norm() > 1e3 ? mRoverApprox : mBase;

    // 一趟收候选星与发射时刻 PVT：组合可用性、周跳剔除、星历健康都在这里定
    std::map<char, std::vector<SatID> > cand;
    for (char sys: mConfig.enabledSystems) {
        auto itc = mRtkCombos.find(sys);
        if (itc == mRtkCombos.end() || !itc->second.ok) continue;
        const SysCombo &sc = itc->second;

        for (const auto &[sat, tvb]: mRtkBase->satTypeValueData) {
            if (sat.system != sys) continue;
            auto itr = mCurObs->satTypeValueData.find(sat);
            if (itr == mCurObs->satTypeValueData.end()) continue;
            if (const TypeValueMap &tvr = itr->second; !sc.base.available(tvb) || !sc.rover.available(tvr)) continue;
            if (mRtkSlipExclude.count(sat)) continue; // 周跳剔除：不参与本历元双差

            Ephemeris *e = mEph->find(sat, mRtkBase->epoch);
            if (!e) continue;
            if (ephUnhealthy(sat, e, mConfig.useHealth)) continue;

            // 发射时刻（用基准站 L1 伪距做传播时延修正）
            CommonTime t_emit = mRtkBase->epoch;
            t_emit.m_sod -= tvb.at(sc.base.code1) / C_MPS;
            satPVTTransTime[sat] = e->getPVT(t_emit, &sc.base);
            cand[sys].push_back(sat);
        }
    }

    // 基类三表存流动站，基准站另立一套供双差随机模型用
    earthRotation(satPVTRecTime, roverRef);
    earthRotation(satPVTBaseRecTime, mBase);
    computeElevAzim(satPVTRecTime, roverRef, satElevData, &satAzimData);
    computeElevAzim(satPVTBaseRecTime, mBase, satBaseElevData, nullptr);

    // 基准站仰角截止：筛掉低星再按系统选参考星
    for (auto &[sys, part]: cand) {
        for (auto it = part.begin(); it != part.end();) {
            if (satBaseElevData.at(*it) >= mConfig.cutoffElevRad) {
                ++it;
                continue;
            }
            satPVTTransTime.erase(*it);
            satPVTRecTime.erase(*it);
            satPVTBaseRecTime.erase(*it);
            satElevData.erase(*it);
            satAzimData.erase(*it);
            satBaseElevData.erase(*it);
            it = part.erase(it);
        }
        if (part.size() < 2) continue; // 双差至少需 2 颗
        const std::optional<SatID> ref = selectRefSatellite(part);
        if (!ref) continue;
        mRefSat[sys] = *ref;
        mPartSats[sys] = std::move(part);
    }

    // 记录实际参与双差解算的卫星集合（一颗星只属于一个系统，集合大小就是双差方程覆盖的星数）
    for (auto &[sys, vec]: mPartSats)
        for (auto &sat: vec) mUsedSats.insert(sat);
}

// 站间单差方差
void RTK::computeSdVariance() {
    for (const auto &[sys, vec]: mPartSats) {
        const SatID &ref = mRefSat.at(sys);
        for (const SatID &sat: vec) {
            const double phase = mConfig.phaseWeight.variance(satBaseElevData.at(sat))
                                 + mConfig.phaseWeight.variance(satElevData.at(sat));
            mSdVar[sat] = SdVar{phase, phase * mConfig.codeOverPhaseSigma * mConfig.codeOverPhaseSigma};
        }
        const double refPhase = mConfig.phaseWeight.variance(satBaseElevData.at(ref))
                                + mConfig.phaseWeight.variance(satElevData.at(ref));
        mSdVar[ref] = SdVar{refPhase, refPhase * mConfig.codeOverPhaseSigma * mConfig.codeOverPhaseSigma};
    }
}

std::optional<SatID> RTK::selectRefSatellite(const std::vector<SatID> &candidates) const {
    std::vector<SatID> cands;
    for (const SatID &s: candidates) {
        if (mConfig.excludeGeoRef && mEph) {
            if (const Ephemeris *e = mEph->find(s, mRtkBase->epoch); e && e->isGEO()) continue;
        }
        cands.push_back(s);
    }
    if (cands.empty()) return std::nullopt;
    return *std::max_element(cands.begin(), cands.end(),
                             [&](const SatID &a, const SatID &b) {
                                 return satElevData.at(a) < satElevData.at(b);
                             });
}

void RTK::buildStochasticModel(const EquSys &eq) {
    const int m = static_cast<int>(eq.obsEquData.size());
    mWeightMatrix = Eigen::MatrixXd::Zero(m, m);
    if (m == 0) return;

    std::vector<EquID> rows(m);
    int k = 0;
    for (const auto &[eid, ed]: eq.obsEquData) rows[k++] = eid;

    auto selfVar = [&](const EquID &eid) -> double {
        const auto &[phase, code] = mSdVar.at(eid.sat);
        return eid.obsType[0] == 'L' ? phase : code;
    };
    auto refVar = [&](const EquID &eid) -> double {
        const auto &[phase, code] = mSdVar.at(mRefSat.at(eid.sat.system));
        return eid.obsType[0] == 'L' ? phase : code;
    };

    Eigen::MatrixXd Dv = Eigen::MatrixXd::Zero(m, m);
    std::map<std::pair<char, std::string>, std::vector<int> > blocks;
    for (int i = 0; i < m; ++i)
        blocks[{rows[i].sat.system, rows[i].obsType}].push_back(i);

    for (auto &[_, idx]: blocks) {
        const double b = refVar(rows[idx[0]]);
        for (int a = 0; a < static_cast<int>(idx.size()); ++a) {
            const double va = selfVar(rows[idx[a]]);
            for (int c = 0; c < static_cast<int>(idx.size()); ++c)
                Dv(idx[a], idx[c]) = a == c ? va + b : b;
        }
    }
    mWeightMatrix = Dv.inverse();
}

void RTK::buildWeightMatrix(const EquSys &eq) {
    if (!mConfig.fullCovariance || mWeightReady) return;
    computeSdVariance();
    buildStochasticModel(eq);
    mWeightReady = true;
}

void RTK::solveIteration(SolverLSQ &solver, EquSys &eq) const {
    if (mConfig.fullCovariance)
        solver.solve(eq, mWeightMatrix, false);
    else
        solver.solve(eq, eq.weightVector().asDiagonal().toDenseMatrix(), false);
}

void RTK::readback(const EquSys &eqFloat, const Eigen::VectorXd &residual, const Eigen::Vector3d &posEcef) {
    const Eigen::Vector3d blh = Coord::XYZToBLH(posEcef, Frame::WGS84);
    geometryDOPs(result, eqFloat, blh);

    int i = 0;
    for (const auto &[eid, ed]: eqFloat.obsEquData) {
        if (i >= residual.size()) break;
        // obsType 以 'C' 开头=码(伪距)双差；以 'L' 开头=相位双差(L1/L2)
        if (eid.obsType[0] != 'L')
            result.postRes.set(eid.sat, residual(i)); // 码(伪距)双差残差
        else
            result.postResPhase.set(eid.sat, residual(i)); // 相位双差残差
        ++i;
    }
}

// 周跳探测
void RTK::detectSlips() {
    mRtkSlipExclude.clear();
    if (!mConfig.useCycleSlip) {
        mSlipDetBase.clear();
        mSlipDetRover.clear();
        return;
    }
    CycleSlipEpoch gfEpoch;

    std::set<SatID> fedBase, fedRover; // 已喂入有效观测的检测器
    const double gfThr = mConfig.useGfCycleSlip ? 1.0 : 0.0; // >0 开启 GF
    const double gfMult = mConfig.gfSlipMult; // GF 周跳阈值倍数

    const auto feedSlot = [&](std::map<SatID, CycleSlip> &dets, const FreqCombo &combo,
                              const SatID &sat, const TypeValueMap &tv, const ObsData *obs) {
        CycleSlip &det = dets[sat];
        det.configure(combo, gfThr, gfMult);
        const CycleSlip::Result r = det.feed(sat, tv, obs);
        gfEpoch.record(sat, r);
        return r.slip;
    };

    for (char sys: mConfig.enabledSystems) {
        auto itc = mRtkCombos.find(sys);
        if (itc == mRtkCombos.end() || !itc->second.ok) continue;
        const SysCombo &sc = itc->second;

        for (const auto &[sat, tvb]: mRtkBase->satTypeValueData) {
            if (sat.system != sys) continue;
            const bool baseOk = sc.base.available(tvb);
            auto itr = mCurObs->satTypeValueData.find(sat);
            const bool roverOk = itr != mCurObs->satTypeValueData.end() && sc.rover.available(itr->second);
            if (!baseOk && !roverOk) continue;

            bool slip = false;
            if (baseOk) {
                if (feedSlot(mSlipDetBase, sc.base, sat, tvb, mRtkBase)) slip = true;
                fedBase.insert(sat);
            }
            if (roverOk) {
                if (feedSlot(mSlipDetRover, sc.rover, sat, itr->second, mCurObs)) slip = true;
                fedRover.insert(sat);
            }
            if (baseOk && roverOk && slip) mRtkSlipExclude.insert(sat);
        }
    }

    auto feedGap = [&](std::map<SatID, CycleSlip> &m, const std::set<SatID> &fed) {
        for (auto &[sat, det]: m)
            if (!fed.count(sat)) det.feedGap();
    };
    feedGap(mSlipDetBase, fedBase);
    feedGap(mSlipDetRover, fedRover);
    SatIDSet gfExclude;
    const SatIDSet gfApply = gfEpoch.decide(mGfCommonMode, gfExclude);
    for (const auto &s: gfApply) mRtkSlipExclude.insert(s);
    for (const auto &s: gfExclude) mRtkSlipExclude.insert(s);
}


void RTK::addDoubleDiffEquation(EquSys &eq, const SatID &sat, const Eigen::Vector3d &ddLos, const double ddGeom,
                                const int freq, const bool isPhase, const bool fixed) const {
    const char sys = sat.system;
    const SatID &ref = mRefSat.at(sys);
    const SysCombo &sc = mRtkCombos.at(sys);
    const std::string &keyB = isPhase ? (freq == 1 ? sc.base.phase1 : sc.base.phase2)
                                      : (freq == 1 ? sc.base.code1 : sc.base.code2);
    const std::string &keyR = isPhase ? (freq == 1 ? sc.rover.phase1 : sc.rover.phase2)
                                      : (freq == 1 ? sc.rover.code1 : sc.rover.code2);
    const double lam = isPhase ? (freq == 1 ? sc.base.lambda1 : sc.base.lambda2) : 0.0;
    const std::string label = freqLabel(freq, isPhase);
    const double dd = mCurObs->satTypeValueData.at(sat).at(keyR) - mRtkBase->satTypeValueData.at(sat).at(keyB)
                      - (mCurObs->satTypeValueData.at(ref).at(keyR) - mRtkBase->satTypeValueData.at(ref).at(keyB));

    EquData ed;
    ed.prefit = dd - ddGeom;
    const SdVar &sv = mSdVar.at(sat);
    const SdVar &svRef = mSdVar.at(ref);
    const double var = isPhase ? svRef.phase + sv.phase : svRef.code + sv.code;
    ed.weight = 1.0 / var;
    const Variable dx("rover", Parameter::dX);
    const Variable dy("rover", Parameter::dY);
    const Variable dz("rover", Parameter::dZ);
    ed.varCoeffData[dx] = ddLos.x();
    ed.varCoeffData[dy] = ddLos.y();
    ed.varCoeffData[dz] = ddLos.z();
    if (isPhase) {
        if (fixed) {
            const auto &fm = mRtkFixedMap.at(sys).at(label);
            const auto itRef = fm.find(ref);
            ed.prefit += lam * (fm.at(sat) - (itRef == fm.end() ? 0.0 : itRef->second));
        } else {
            const Variable amb = ambVar(sat, freq);
            ed.varCoeffData[amb] = -lam;
            if (ekf.currentUnkSet.count(amb)) {
                ed.prefit += lam * ekf.priorOf(amb);
            }
        }
    }
    eq.obsEquData[EquID(sat, label)] = ed;
}

bool RTK::solveFloat(Eigen::Vector3d &baseline) {
    Eigen::Vector3d roverPos = mRoverApprox.norm() > 1e3 ? mRoverApprox : mBase;
    SolverLSQ solver;
    for (int iter = 0; iter < mConfig.maxIterFloat; ++iter) {
        buildEquSys(curEq, roverPos, false);
        if (curEq.obsEquData.size() < 4) return false;
        solveIteration(solver, curEq);
        auto dx = Eigen::Vector3d(solver.getSolution(Parameter::dX),
                                  solver.getSolution(Parameter::dY),
                                  solver.getSolution(Parameter::dZ));
        roverPos += dx;
        if (dx.norm() < mConfig.convEps) break;
    }
    baseline = roverPos - mBase;
    return true;
}

// 组装双差方程
void RTK::collectVars(EquSys &eq, const bool fixed) const {
    eq.varSet.clear();
    eq.varSet.insert(Variable("rover", Parameter::dX));
    eq.varSet.insert(Variable("rover", Parameter::dY));
    eq.varSet.insert(Variable("rover", Parameter::dZ));
    if (fixed) return; // 固定解把整周当已知，方程里不再有模糊度
    // 待估模糊度就是站星间双差，LSQ 档与滤波档共用同一份（课件第 19 页的参数化）
    for (const auto &[sys, vec]: mPartSats)
        for (const SatID &sat: vec)
            if (sat != mRefSat.at(sys)) {
                eq.varSet.insert(ambVar(sat, 1));
                eq.varSet.insert(ambVar(sat, 2));
            }
}

// 时间更新的状态集合按参与双差的星收集；新弧段/周跳的模糊度初值在收集时一并反算
void RTK::collectEpochStates(VariableSet &varSet, AmbSeed &seed) const {
    varSet.insert(Variable("rover", Parameter::dX));
    varSet.insert(Variable("rover", Parameter::dY));
    varSet.insert(Variable("rover", Parameter::dZ));
    for (const auto &[sys, vec]: mPartSats) {
        const SatID &ref = mRefSat.at(sys);
        const SysCombo &sc = mRtkCombos.at(sys);
        const TypeValueMap &tvbRef = mRtkBase->satTypeValueData.at(ref);
        const TypeValueMap &tvrRef = mCurObs->satTypeValueData.at(ref);
        const double refB[4] = {tvbRef.at(sc.base.phase1), tvbRef.at(sc.base.phase2),
                                tvbRef.at(sc.base.code1), tvbRef.at(sc.base.code2)};
        const double refR[4] = {tvrRef.at(sc.rover.phase1), tvrRef.at(sc.rover.phase2),
                                tvrRef.at(sc.rover.code1), tvrRef.at(sc.rover.code2)};
        for (const SatID &sat: vec) {
            if (sat == ref) continue;
            const TypeValueMap &tvb = mRtkBase->satTypeValueData.at(sat);
            const TypeValueMap &tvr = mCurObs->satTypeValueData.at(sat);
            const double obsB[4] = {tvb.at(sc.base.phase1), tvb.at(sc.base.phase2),
                                    tvb.at(sc.base.code1), tvb.at(sc.base.code2)};
            const double obsR[4] = {tvr.at(sc.rover.phase1), tvr.at(sc.rover.phase2),
                                    tvr.at(sc.rover.code1), tvr.at(sc.rover.code2)};
            for (int f = 0; f < 2; ++f) {
                const Variable amb = ambVar(sat, f + 1);
                varSet.insert(amb);
                const double lam = f == 0 ? sc.base.lambda1 : sc.base.lambda2;
                if (ekf.currentUnkSet.count(amb) || !(lam > 0.0)) continue;
                // 新弧段初值：站星双差的「相位减码」再除波长
                const double sdSat = (obsR[f] - obsB[f] - (obsR[f + 2] - obsB[f + 2])) / lam;
                const double sdRef = (refR[f] - refB[f] - (refR[f + 2] - refB[f + 2])) / lam;
                seed.init[amb] = -(sdSat - sdRef);
                seed.novar[amb] = 2.0;
            }
        }
    }
}

void RTK::buildEquSys(EquSys &eq, const Eigen::Vector3d &roverPos, const bool fixed) {
    eq.reset();
    eq.station = "rtk";
    collectVars(eq, fixed);

    for (const auto &[sys, vec]: mPartSats) {
        const SatID &ref = mRefSat.at(sys);
        // 参考星几何：站端各用自己旋转后的星位，迭代内随 roverPos 重算
        double rhoBref = 0.0, rhoRref = 0.0;
        Eigen::Vector3d losRref;
        Coord::geometricRangeAndLos(satPVTBaseRecTime.at(ref).p, mBase, rhoBref, losRref);
        Coord::geometricRangeAndLos(satPVTRecTime.at(ref).p, roverPos, rhoRref, losRref);

        for (const SatID &sat: vec) {
            if (sat == ref) continue;
            // 固定解只用被 AR 取到的星：整周缺项（秩不足被跳过）的星整组不进方程
            if (fixed) {
                const auto its = mRtkFixedMap.find(sys);
                bool covered = its != mRtkFixedMap.end();
                for (const char *lb: {"L1", "L2"})
                    covered = covered && its->second.count(lb) && its->second.at(lb).count(sat);
                if (!covered) continue;
            }
            double rhoB = 0.0, rhoR = 0.0;
            Eigen::Vector3d losR;
            Coord::geometricRangeAndLos(satPVTBaseRecTime.at(sat).p, mBase, rhoB, losR);
            Coord::geometricRangeAndLos(satPVTRecTime.at(sat).p, roverPos, rhoR, losR);

            const double ddGeom = rhoR - rhoB - (rhoRref - rhoBref);
            const Eigen::Vector3d ddLos = losR - losRref;

            // 四条双差观测
            addDoubleDiffEquation(eq, sat, ddLos, ddGeom, 1, true, fixed);
            addDoubleDiffEquation(eq, sat, ddLos, ddGeom, 2, true, fixed);
            addDoubleDiffEquation(eq, sat, ddLos, ddGeom, 1, false, fixed);
            addDoubleDiffEquation(eq, sat, ddLos, ddGeom, 2, false, fixed);
        }
    }
    buildWeightMatrix(eq); // 随机模型只依赖卫星集合与仰角，迭代内复用
}


bool RTK::tryFixAmbiguities(const std::vector<Variable> &aVars, const Eigen::VectorXd &b,
                            const Eigen::MatrixXd &Qb) {
    const int M = static_cast<int>(aVars.size());
    if (M < 2 || b.size() != M || Qb.rows() != M) return false;

    //LAMBDA 整数最小二乘搜索
    const auto [z, ratio, fixed] = resolveWithRatio(b, Qb, mConfig.arThreshold);
    result.ratio = ratio;
    if (!fixed) return false;

    // 固定解：存相对参考星的双差整周，参考星按 0 登记
    std::map<char, std::map<std::string, std::map<SatID, double> > > fixedMap;
    for (int i = 0; i < M; ++i) {
        const Variable &v = aVars[i];
        fixedMap[v.obsID.satSys[0]][v.obsID.obsType][v.sat] = z(i);
    }
    for (auto &[sys, byType]: fixedMap) {
        const auto itRef = mRefSat.find(sys);
        if (itRef == mRefSat.end()) continue;
        for (auto &[ot, bySat]: byType) bySat.emplace(itRef->second, 0.0);
    }
    mRtkFixedMap = std::move(fixedMap);

    EquSys eqFix;
    SolverLSQ solverFix;
    Eigen::Vector3d roverPos = mBase + mBaselineFloat; // 从浮点基线起步
    mWeightReady = false; // 固定解重新建一次，确保 P 与方程行序严格对齐
    for (int iter = 0; iter < mConfig.maxIterFix; ++iter) {
        buildEquSys(eqFix, roverPos, true);
        solveIteration(solverFix, eqFix);
        auto dx = Eigen::Vector3d(solverFix.getSolution(Parameter::dX),
                                  solverFix.getSolution(Parameter::dY),
                                  solverFix.getSolution(Parameter::dZ));
        roverPos += dx;
        if (dx.norm() < mConfig.convEps) break;
    }
    solverFix.computeCovariance(); // 用最终迭代分解补算协方差
    mBaselineFixed = roverPos - mBase;
    mBaselineSigmaFixed = Eigen::Vector3d(std::sqrt(solverFix.Q(0, 0)),
                                          std::sqrt(solverFix.Q(1, 1)),
                                          std::sqrt(solverFix.Q(2, 2)));
    return true;
}

void RTK::applyOptions(const SolverOptions &o) {
    mConfig = o.rtk;
    mConfig.cutoffElevRad = o.cutoffDeg * DEG_TO_RAD;
    mConfig.enabledSystems = o.systems;
    mUseFilter = o.filter != SolverOptions::FilterMode::LSQ;
    mPosProcSigma = o.posProcSigma;
    if (!mUseFilter) resetFilterState();
}
