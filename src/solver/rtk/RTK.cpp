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

#include "SolverLSQ.h"


bool RTK::processEpoch(const ObsData &base, const ObsData &rover, EphemerisTable &eph) {
    result.reset(); // 早退分支不能留下上一历元的结果
    mWeightReady = false; // 随机模型按本历元卫星集合重算
    if (!mUseFilter && mFilterInit) resetFilterState();
    if (!preparePositions(base, rover, eph)) return false;

    mBaseObs = &base;
    mCurObs = &rover;
    mEph = &eph;
    detectFreqCombinations();
    detectSlips();

    prepareGeometry();
    if (static_cast<int>(mUsedSats.size()) < mConfig.minSats) return false; // 参与双差的星不足，本历元不给解
    computeSdVariance();

    solve();
    return result.stat != 0;
}

void RTK::applyOptions(const SolverOptions &o) {
    mConfig = o.rtk;
    mConfig.cutoffElevRad = o.cutoffDeg * DEG_TO_RAD;
    mConfig.enabledSystems = o.systems;
    mUseCycleSlip = mConfig.useCycleSlip;
    mUseGFCycleSlip = mConfig.useGFCycleSlip;
    mGFSlipMult = mConfig.GFSlipMult;
    mUseFilter = o.filter != SolverOptions::FilterMode::LSQ;
    mPosProcSigma = o.posProcSigma;
    if (!mUseFilter) resetFilterState();
}

void RTK::resetFilterState() {
    ekf.reset();
    mFilterInit = false;
    xyz.setZero();
    mPrevRefSat.clear();
}

bool RTK::preparePositions(const ObsData &base, const ObsData &rover, const EphemerisTable &eph) {
    if (isSaneECEF(mBaseUser)) {
        mBase = mBaseUser;
    } else if (isSaneECEF(base.antennaPosition)) {
        mBase = base.antennaPosition;
    } else {
        // 在星历数量变多时重算
        if (const std::size_t es = eph.size(); !isSaneECEF(mBase) || es > mBaseEphSize) {
            if (const Eigen::Vector3d p = estimateApproxPosition(base, eph); isSaneECEF(p)) {
                mBase = p;
                mBaseEphSize = es;
            }
        }
    }
    if (!isSaneECEF(mBase)) {
        LOG_WARN << "[RTK] base position not available (no BESTPOS/header and SPP fallback failed)";
        return false;
    }

    if (isSaneECEF(rover.antennaPosition)) {
        mRoverApprox = rover.antennaPosition;
    } else if (isSaneECEF(mLastXYZ)) {
        mRoverApprox = mLastXYZ;
    } else {
        // 在星历数量变多时重算
        if (const std::size_t es = eph.size(); !isSaneECEF(mRoverApprox) || es > mRoverEphSize) {
            if (const Eigen::Vector3d p = estimateApproxPosition(rover, eph); isSaneECEF(p)) {
                mRoverApprox = p;
                mRoverEphSize = es;
            }
        }
    }
    // 若流动站概略坐标仍不可用，则退回基准站坐标
    mRoverApprox = isSaneECEF(mRoverApprox) ? mRoverApprox : mBase;
    return true;
}

// SPP 反算概略坐标
Eigen::Vector3d RTK::estimateApproxPosition(const ObsData &obs, const EphemerisTable &eph) const {
    if (eph.empty()) return Eigen::Vector3d::Zero();
    SPP spp = {};
    spp.prod->eph = eph;
    spp.setEnabledSystems(mConfig.enabledSystems);
    spp.setCutoffElevDeg(mConfig.cutoffElevRad * RAD_TO_DEG);
    ObsData work = obs;
    work.antennaPosition = Eigen::Vector3d::Zero();
    try { if (spp.processEpoch(work)) return spp.result.xyz; } catch (...) {
    }
    return Eigen::Vector3d::Zero();
}

void RTK::detectFreqCombinations() {
    mSysCombos.clear();
    for (SatSystem sys: mConfig.enabledSystems) {
        if (sys == SatSystem::R) continue; // GLONASS FDMA 双差模糊度无整数性，整系统不参与
        // 收集双频，用于 MW & GF 周跳探测
        SysCombo sc;
        sc.base = FreqCombo::detect(sys, FreqCombo::collectCodes(*mBaseObs, sys), 0, true);
        sc.rover = FreqCombo::detect(sys, FreqCombo::collectCodes(*mCurObs, sys), 0, true);
        if (!sc.base.code1.empty() && !sc.rover.code1.empty()) sc.ok = true;
        if (!sc.ok) continue;
        mSysCombos[sys] = sc;
    }
}

// 周跳探测
void RTK::detectSlips() {
    mSlipExclude.clear();
    if (!mUseCycleSlip) {
        mSlipDetBase.clear();
        mSlipDetRover.clear();
        return;
    }
    SlipEpoch GFEpoch;

    std::set<SatID> fedBase, fedRover; // 已喂入有效观测的检测器

    const auto feedSlot = [&](std::map<SatID, SlipDetector> &dets, const FreqCombo &combo,
                              const SatID &sat, const TypeValueMap &tv, const ObsData *obs) {
        SlipDetector &det = dets[sat];
        det.configure(combo, mUseGFCycleSlip, mGFSlipMult);
        const SlipDetector::Result r = det.phaseCycle(sat, tv, obs);
        GFEpoch.record(sat, r);
        return r.slip;
    };

    for (SatSystem sys: mConfig.enabledSystems) {
        auto itc = mSysCombos.find(sys);
        if (itc == mSysCombos.end() || !itc->second.ok) continue;
        const SysCombo &sc = itc->second;

        for (const auto &[sat, tvb]: mBaseObs->satTypeValueData) {
            if (sat.system != sys) continue;
            const bool baseOk = sc.base.available(tvb); // 基准站该星有可用频点观测
            auto itr = mCurObs->satTypeValueData.find(sat);
            const bool roverOk = itr != mCurObs->satTypeValueData.end() && sc.rover.available(itr->second); // 流动站该星有可用频点观测
            if (!baseOk && !roverOk) continue;

            bool slip = false;
            if (baseOk) {
                if (feedSlot(mSlipDetBase, sc.base, sat, tvb, mBaseObs)) slip = true;
                fedBase.insert(sat);
            }
            if (roverOk) {
                if (feedSlot(mSlipDetRover, sc.rover, sat, itr->second, mCurObs)) slip = true;
                fedRover.insert(sat);
            }
            // 任一端报跳就必须在双差里剔掉它
            if (baseOk && roverOk && slip) mSlipExclude.insert(sat);
        }
    }

    // 断流即清弧段
    auto clearIdle = [&](std::map<SatID, SlipDetector> &m, const std::set<SatID> &fed) {
        for (auto &[sat, det]: m)
            if (!fed.count(sat)) det.setCycleGap();
    };
    clearIdle(mSlipDetBase, fedBase);
    clearIdle(mSlipDetRover, fedRover);
    std::set<SatID> GFExclude;
    const std::set<SatID> GFApply = GFEpoch.decide(mGFCommonModeMinSats, GFExclude);
    for (const auto &s: GFApply) mSlipExclude.insert(s);
    for (const auto &s: GFExclude) mSlipExclude.insert(s);
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


    std::map<SatSystem, std::vector<SatID> > candidates;
    for (SatSystem sys: mConfig.enabledSystems) {
        auto itc = mSysCombos.find(sys);
        if (itc == mSysCombos.end() || !itc->second.ok) continue;
        const SysCombo &sc = itc->second;

        for (const auto &[sat, tvb]: mBaseObs->satTypeValueData) {
            if (sat.system != sys) continue;
            auto itr = mCurObs->satTypeValueData.find(sat);
            if (itr == mCurObs->satTypeValueData.end()) continue;
            if (const TypeValueMap &tvr = itr->second;
                !sc.base.available(tvb) || !sc.rover.available(tvr))
                continue;
            if (mSlipExclude.count(sat)) continue; // 周跳剔除

            Ephemeris *e = mEph->find(sat, mBaseObs->epoch);
            if (!e) continue;
            if (ephUnhealthy(sat, e, mConfig.useHealth)) continue; // 剔除不健康的卫星

            // 发射时刻
            CommonTime t_emit = mBaseObs->epoch;
            t_emit.m_sod -= tvb.at(sc.base.code1) / C_MPS;
            satPVTTransTime[sat] = e->getPVT(t_emit, &sc.base);
            candidates[sys].push_back(sat);
        }
    }
    SatValueMap baseAzim; // 没有用，用局部变量装
    // 先前没施加地球自转改正导致固定率下降
    earthRotation(satPVTRecTime, mRoverApprox);
    earthRotation(satPVTBaseRecTime, mBase);
    computeElevAzim(satPVTRecTime, mRoverApprox, satElevData, satAzimData);
    computeElevAzim(satPVTBaseRecTime, mBase, satBaseElevData, baseAzim);

    // 基准站仰角截止：筛掉低星再按系统选参考星
    for (auto &[sys, part]: candidates) {
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

std::optional<SatID> RTK::selectRefSatellite(const std::vector<SatID> &part) const {
    // 按系统在各自内部选高度角最高的星作参考
    std::vector<SatID> candidates;
    for (const SatID &s: part) {
        if (mConfig.excludeGeoRef && mEph) {
            if (const Ephemeris *e = mEph->find(s, mBaseObs->epoch); e && e->isGEO()) continue;
        }
        candidates.push_back(s);
    }
    if (candidates.empty()) return std::nullopt;
    return *std::max_element(candidates.begin(), candidates.end(),
                             [&](const SatID &a, const SatID &b) {
                                 return satElevData.at(a) < satElevData.at(b);
                             });
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

void RTK::initFilter() {
    if (!bootstrapSolution()) {
        xyz.setZero();
        return;
    }

    ekf.reset();
    // 理想状态下模糊度应该是常数模型，σ=0 才对，过程噪声本来是给漏检周跳和大气残差兜底的权宜之计
    ekf.configure(mPosProcSigma, false, mConfig.ambProcSigma, 0.0);
    ekf.accProcSigma = 0.0;
    ekf.ambInitSigma = mConfig.ambInitSigma;

    std::vector<AmbReset> resets;
    VariableSet varSet;
    collectFilterVariables(varSet, resets);
    ekf.timeUpdate(varSet, {});
    ekf.setPosition(xyz, GnssEKF::kInitPosVar);
    std::map<Variable, double> ambInit;
    for (const auto &r: resets) ambInit[r.var] = r.initValue;
    ekf.setStates(ambInit);
    buildEquSys(curEq, xyz, false);
    ekf.measUpdate(curEq, mConfig.fullCovariance ? mWeightMatrix : curEq.weightVector().asDiagonal().toDenseMatrix());
    xyz = getEKFPosition();
    sigma0 = posteriorSigma0(ekf.postfitResidual, curEq.weightVector(), static_cast<int>(ekf.currentUnkSet.size()));
    mPrevRefSat = mRefSat;
    mFilterInit = true;
}


// 首历元迭代最小二乘 无滤波时总是执行
bool RTK::bootstrapSolution() {
    Eigen::Vector3d roverPos = mRoverApprox;
    for (int iter = 0; iter < mConfig.maxIterFloat; ++iter) {
        buildEquSys(curEq, roverPos, false);
        if (curEq.obsEquData.size() < 4) return false;
        lsq.solve(curEq, mConfig.fullCovariance ? mWeightMatrix : curEq.weightVector().asDiagonal().toDenseMatrix(), false);
        const auto dX = Eigen::Vector3d(lsq.getSolution(Parameter::dX),
                                        lsq.getSolution(Parameter::dY),
                                        lsq.getSolution(Parameter::dZ));
        roverPos += dX;
        if (dX.norm() < mConfig.convEps) break;
    }
    xyz = roverPos;
    return true;
}


void RTK::collectFilterVariables(VariableSet &varSet, std::vector<AmbReset> &resets) const {
    static const char *ambLabels[2] = {"L1", "L2"};
    // 3+2K 未知数，4K 方程数
    // 3 个流动站坐标 dX/dY/dZ
    varSet.insert(Variable(kRover, Parameter::dX));
    varSet.insert(Variable(kRover, Parameter::dY));
    varSet.insert(Variable(kRover, Parameter::dZ));
    for (const auto &[sys, vec]: mPartSats) {
        const SatID &ref = mRefSat.at(sys);
        const SysCombo &sc = mSysCombos.at(sys);
        const TypeValueMap &tvbRef = mBaseObs->satTypeValueData.at(ref);
        const TypeValueMap &tvrRef = mCurObs->satTypeValueData.at(ref);
        // 参考星→基站观测值
        const double refB[4] = {
            tvbRef.at(sc.base.phase1), tvbRef.at(sc.base.phase2),
            tvbRef.at(sc.base.code1), tvbRef.at(sc.base.code2)
        };
        // 参考星→流动站观测值
        const double refR[4] = {
            tvrRef.at(sc.rover.phase1), tvrRef.at(sc.rover.phase2),
            tvrRef.at(sc.rover.code1), tvrRef.at(sc.rover.code2)
        };
        for (const SatID &sat: vec) {
            if (sat == ref) continue;
            for (int f = 0; f < 2; ++f) {
                //  每颗非参考星 L1/L2 各一个模糊度
                const Variable amb = Variable::makeAmbiguity(kRover, sat, ambLabels[f]);
                varSet.insert(amb);
                const double lam = f == 0 ? sc.base.lambda1 : sc.base.lambda2;
                if (ekf.currentUnkSet.count(amb) || !(lam > 0.0)) continue;

                const TypeValueMap &tvb = mBaseObs->satTypeValueData.at(sat);
                const TypeValueMap &tvr = mCurObs->satTypeValueData.at(sat);
                // 当前卫星→基站观测值
                const double obsB[4] = {
                    tvb.at(sc.base.phase1), tvb.at(sc.base.phase2),
                    tvb.at(sc.base.code1), tvb.at(sc.base.code2)
                };
                // 当前卫星→流动站观测值
                const double obsR[4] = {
                    tvr.at(sc.rover.phase1), tvr.at(sc.rover.phase2),
                    tvr.at(sc.rover.code1), tvr.at(sc.rover.code2)
                };
                // 只要状态里没有这个模糊度，它必定是新弧段、周跳、或参考星切换——这三种情况都要求立刻给一个可靠初值
                const double sdSat = (obsR[f] - obsB[f] - (obsR[f + 2] - obsB[f + 2])) / lam;
                const double sdRef = (refR[f] - refB[f] - (refR[f + 2] - refB[f + 2])) / lam;
                resets.push_back({amb, AmbReset::Kind::Slip, -(sdSat - sdRef)});
            }
        }
    }
}

void RTK::rebaseAmbiguities() {
    // 参考星变更时，模糊度状态需要重新参数化，旧参考星的模糊度状态被新参考星的模糊度状态替代
    if (mPrevRefSat.empty() || ekf.currentUnkSet.empty()) return;

    const VariableSet &oldSet = ekf.currentUnkSet;
    std::map<Variable, int> oldIndex;
    for (const auto &v: oldSet) oldIndex[v] = ekf.indexOf(v);

    const auto refChanged = [&](const SatSystem sys) {
        const auto n = mRefSat.find(sys), o = mPrevRefSat.find(sys);
        return n != mRefSat.end() && o != mPrevRefSat.end() && n->second != o->second;
    };

    // pair 第一个值代表正值位，第二个值代表负值位，-1代表无值
    std::map<Variable, std::pair<int, int> > rowMap;
    //先给所有非模糊度变量和未变系统的模糊度写恒等行
    for (const auto &v: oldSet) {
        if (v.par != Parameter::ambiguity || !refChanged(v.sat.system))
            rowMap[v] = {oldIndex.at(v), -1};
    }

    bool anyChanged = false;
    for (const auto &[sys, sats]: mPartSats) {
        if (!refChanged(sys)) continue;
        anyChanged = true;
        const auto b = mRefSat.at(sys);
        const auto a = mPrevRefSat.at(sys);
        for (const char *lb: {"L1", "L2"}) {
            const auto itb = oldIndex.find(Variable::makeAmbiguity(kRover, b, lb));
            if (itb == oldIndex.end()) continue; // 新参考星上历元无状态：整系统按新弧段重开
            for (const SatID &sat: sats) {
                if (sat == b) continue;
                const Variable vi = Variable::makeAmbiguity(kRover, sat, lb);
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
    // 将 rowMap 映射成 T 矩阵
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
    mPrevRefSat = mRefSat;
    std::vector<AmbReset> resets;
    VariableSet varSet;
    collectFilterVariables(varSet, resets);
    ekf.timeUpdate(varSet, resets);
    const Eigen::Vector3d xPred = {
        ekf.getMinus(Variable(kRover, Parameter::dX)),
        ekf.getMinus(Variable(kRover, Parameter::dY)),
        ekf.getMinus(Variable(kRover, Parameter::dZ))
    };
    buildEquSys(curEq, xPred, false);
    // 粗差门限按星成组剔（同星四个观测共享几何误差，删单条会破坏双差结构）
    const Eigen::VectorXd residuals = ekf.getMinusResiduals(curEq);
    std::set<SatID> bad;
    int irow = 0;
    for (const auto &[eid, ed]: curEq.obsEquData) {
        const double sigObs = std::sqrt(1.0 / ed.weight);
        if (const double sig = irow < residuals.size() ? std::hypot(residuals(irow), sigObs) : sigObs;
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
    ekf.measUpdate(curEq, mConfig.fullCovariance ? mWeightMatrix : curEq.weightVector().asDiagonal().toDenseMatrix());
    xyz = getEKFPosition();
    sigma0 = posteriorSigma0(ekf.postfitResidual, curEq.weightVector(),
                             static_cast<int>(ekf.currentUnkSet.size()));
}

void RTK::buildEquSys(EquSys &eq, const Eigen::Vector3d &roverPos, const bool fixed) {
    eq.reset();
    eq.station = "rtk";
    eq.varSet.clear();
    eq.varSet.insert(Variable(kRover, Parameter::dX));
    eq.varSet.insert(Variable(kRover, Parameter::dY));
    eq.varSet.insert(Variable(kRover, Parameter::dZ));
    if (!fixed) {
        // 待估模糊度就是站星间双差
        for (const auto &[sys, vec]: mPartSats)
            for (const SatID &sat: vec)
                if (sat != mRefSat.at(sys)) {
                    eq.varSet.insert(Variable::makeAmbiguity(kRover, sat, "L1"));
                    eq.varSet.insert(Variable::makeAmbiguity(kRover, sat, "L2"));
                }
    }
    for (const auto &[sys, vec]: mPartSats) {
        const SatID &ref = mRefSat.at(sys);
        // 参考星几何：站端各用自己旋转后的星位，迭代内随 roverPos 重算
        const double rhoBref = (mBase - satPVTBaseRecTime.at(ref).p).norm();
        const Eigen::Vector3d dRef = roverPos - satPVTRecTime.at(ref).p;
        const double rhoRref = dRef.norm();
        const Eigen::Vector3d losRref = dRef.normalized();

        for (const SatID &sat: vec) {
            if (sat == ref) continue;
            const Variable amb1 = Variable::makeAmbiguity(kRover, sat, "L1");
            const Variable amb2 = Variable::makeAmbiguity(kRover, sat, "L2");
            // 固定解只用被 AR 取到的星：整周缺项（秩不足被跳过）的星整组不进方程
            if (fixed && (!mFixedMap.count(amb1) || !mFixedMap.count(amb2))) continue;
            const double rhoB = (mBase - satPVTBaseRecTime.at(sat).p).norm();
            const Eigen::Vector3d dSat = roverPos - satPVTRecTime.at(sat).p;
            const double rhoR = dSat.norm();
            const Eigen::Vector3d losR = dSat.normalized();

            const double ddGeom = rhoR - rhoB - (rhoRref - rhoBref);
            const Eigen::Vector3d ddLos = losR - losRref;

            const SysCombo &sc = mSysCombos.at(sys);
            const auto &[svPhase, svCode] = mSdVar.at(sat);
            const auto &[rvPhase, rvCode] = mSdVar.at(ref);
            const Variable dx(kRover, Parameter::dX);
            const Variable dy(kRover, Parameter::dY);
            const Variable dz(kRover, Parameter::dZ);
            auto add = [&](const char *label, const std::string &keyB, const std::string &keyR,
                           const double variance, const bool isPhase) -> EquData & {
                const double dd = mCurObs->satTypeValueData.at(sat).at(keyR) - mBaseObs->satTypeValueData.at(sat).at(keyB)
                                  - (mCurObs->satTypeValueData.at(ref).at(keyR) - mBaseObs->satTypeValueData.at(ref).at(keyB));
                EquData ed;
                ed.isPhase = isPhase;
                ed.prefit = dd - ddGeom;
                ed.weight = 1.0 / variance;
                ed.varCoefficient[dx] = ddLos.x();
                ed.varCoefficient[dy] = ddLos.y();
                ed.varCoefficient[dz] = ddLos.z();
                return eq.obsEquData[EquID(sat, label)] = ed;
            };
            add("C1", sc.base.code1, sc.rover.code1, rvCode + svCode, false);
            add("C2", sc.base.code2, sc.rover.code2, rvCode + svCode, false);
            EquData &phase1 = add("L1", sc.base.phase1, sc.rover.phase1, rvPhase + svPhase, true);
            EquData &phase2 = add("L2", sc.base.phase2, sc.rover.phase2, rvPhase + svPhase, true);
            if (fixed) {
                phase1.prefit += sc.base.lambda1 * mFixedMap.at(amb1);
                phase2.prefit += sc.base.lambda2 * mFixedMap.at(amb2);
            } else {
                phase1.varCoefficient[amb1] = -sc.base.lambda1;
                if (ekf.currentUnkSet.count(amb1))
                    phase1.prefit += sc.base.lambda1 * ekf.getMinus(amb1);
                phase2.varCoefficient[amb2] = -sc.base.lambda2;
                if (ekf.currentUnkSet.count(amb2))
                    phase2.prefit += sc.base.lambda2 * ekf.getMinus(amb2);
            }
        }
    }
    buildWeightMatrix(eq);
}

void RTK::buildWeightMatrix(const EquSys &eq) {
    if (!mConfig.fullCovariance || mWeightReady) return;
    const int m = static_cast<int>(eq.obsEquData.size());
    mWeightMatrix = Eigen::MatrixXd::Zero(m, m);
    mWeightReady = true;
    if (m == 0) return;

    std::map<std::pair<SatSystem, std::string>, std::vector<int> > groups;
    std::vector<SatID> rowSat(m);
    int row = 0;
    for (const auto &[eid, ed]: eq.obsEquData) {
        rowSat[row] = eid.sat;
        // 建立该系统该观测类型的方程索引
        groups[{eid.sat.system, eid.obsType}].push_back(row++);
    }

    for (const auto &[group, idx]: groups) {
        const bool isPhase = group.second[0] == 'L';
        const auto &[phaseRef, codeRef] = mSdVar.at(mRefSat.at(group.first));
        const double refVar = isPhase ? phaseRef : codeRef;
        const int n = static_cast<int>(idx.size());

        Eigen::VectorXd satVars(n);
        Eigen::VectorXi ii(n);
        for (int a = 0; a < n; ++a) {
            const auto &[phaseSat, codeSat] = mSdVar.at(rowSat[idx[a]]);
            satVars(a) = isPhase ? phaseSat : codeSat;
            ii(a) = idx[a];
        }
        mWeightMatrix(ii, ii) = satVars.asDiagonal().toDenseMatrix() + refVar * Eigen::MatrixXd::Ones(n, n);
    }
    mWeightMatrix = mWeightMatrix.inverse();
}


void RTK::publishResult() {
    if (xyz.norm() < 1.0e6) return;

    result.xyz = xyz;
    result.blh = Coord::XYZToBLH(xyz, Frame::WGS84);
    result.xyzFloat = xyz;
    result.numSats = static_cast<int>(mUsedSats.size());
    mBaselineFloat = xyz - mBase;

    const Eigen::Matrix3d Q = posCovariance();
    result.sigmaXYZ = Q.diagonal().array().max(0.0).sqrt().matrix() * sigma0;
    result.sigmaP = positionSigma(Q, sigma0);
    result.sigmaPFloat = result.sigmaP;
    result.ratio = 0.0;
    result.stat = 2;

    // DOP 与后验残差随浮点解出：残差按观测类型分码/相位两张表
    geometryDOPs(result, curEq, result.blh);
    readbackResidual(curEq, ekf.postfitResidual);

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
        idx.push_back(i);// 记录索引
    }

    const int M = static_cast<int>(aVars.size());
    b.resize(M);
    Qb.resize(M, M);
    for (int i = 0; i < M; ++i) {
        b(i) = ekf.solution(idx[i]); // 提取模糊度浮点解
        for (int j = 0; j < M; ++j) Qb(i, j) = ekf.Q(idx[i], idx[j]); // 提取模糊度协方差
    }
}


bool RTK::tryFixAmbiguities(const std::vector<Variable> &aVars, const Eigen::VectorXd &b,
                            const Eigen::MatrixXd &Qb) {
    const int M = static_cast<int>(aVars.size());
    if (M < 2 || b.size() != M || Qb.rows() != M) return false;

    //LAMBDA 整数最小二乘搜索
    const auto [z, ratio, fixed] = resolveWithRatio(b, Qb, mConfig.arThreshold);
    result.ratio = ratio;
    if (!fixed) return false;

    mFixedMap.clear();
    for (int i = 0; i < M; ++i)
        mFixedMap[aVars[i]] = z(i);

    EquSys eqFix;
    Eigen::Vector3d roverPos = mBase + mBaselineFloat; // 从浮点基线起步
    mWeightReady = false; // 固定解重新建一次，确保 P 与方程行序严格对齐
    for (int iter = 0; iter < mConfig.maxIterFix; ++iter) {
        buildEquSys(eqFix, roverPos, true);
        lsq.solve(eqFix, mConfig.fullCovariance ? mWeightMatrix : eqFix.weightVector().asDiagonal().toDenseMatrix(), false);
        auto dX = Eigen::Vector3d(lsq.getSolution(Parameter::dX),
                                  lsq.getSolution(Parameter::dY),
                                  lsq.getSolution(Parameter::dZ));
        roverPos += dX;
        if (dX.norm() < mConfig.convEps) break;
    }
    lsq.computeCovariance(); // 用最终迭代分解补算协方差
    mBaselineFixed = roverPos - mBase;
    mBaselineSigmaFixed = Eigen::Vector3d(std::sqrt(lsq.Q(0, 0)),
                                          std::sqrt(lsq.Q(1, 1)),
                                          std::sqrt(lsq.Q(2, 2)));
    return true;
}

void RTK::applyFixedSolution(const double s0) {
    result.xyz = mBase + mBaselineFixed;
    result.blh = Coord::XYZToBLH(result.xyz, Frame::WGS84);
    result.xyzFixed = result.xyz;
    result.blhFixed = result.blh;
    result.sigmaXYZ = mBaselineSigmaFixed * s0;
    Eigen::Matrix3d Qfix = Eigen::Matrix3d::Zero();
    for (int k = 0; k < 3; ++k) Qfix(k, k) = mBaselineSigmaFixed[k] * mBaselineSigmaFixed[k];
    result.sigmaP = positionSigma(Qfix, s0);
    result.sigmaPFixed = result.sigmaP; // 固定 σ 单独留存
    result.stat = 1; // Fixed
}

Eigen::Vector3d RTK::getEKFPosition() const {
    return {
        GnssEKF::getSolution(Parameter::dX, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dY, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dZ, ekf.currentUnkSet, ekf.solution)
    };
}
