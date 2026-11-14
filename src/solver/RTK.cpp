#include "RTK.h"
#include "core/equ/Precision.h"
#include "core/obs/Weight.h"
#include "core/Const.h"
#include "rinex/RINEXNavStore.h"
#include "ARLambda.h"
#include "SPP.h"
#include "core/geom/CoordConvert.h"
#include "core/geom/CoordStruct.h"
#include "Velocity.h"
#include "Log.h"

#include <algorithm>
#include <cmath>

namespace {
    /// 常速度预测的最长跨历元间隔：超过此时速度已不代表本历元运动状态（与 SPP 同口径）
    /// 引导历元位置先验 σ：松到让首历元量测主导，又不至于让 CV 的零速度初值拖住解
    constexpr double kBootPosVar = 10.0 * 10.0;
    /// 滤波档抗差门限（按新息 σ 的倍数）：先验位置由运动模型给，运动时它本身就差几十厘米，
    /// 用固定米级门限剔星等于把整批观测当粗差丢掉。按 sqrt(先验方差+观测方差) 定倍数才有意义。
    constexpr double kPrefitGateSigma = 4.0;
    constexpr std::size_t kGateMinObs = 8;    // 剔完少于这么多条就当门限失效，恢复不剔
}


Eigen::Vector3d RTK::estimateApproxPosition(const ObsData &obs, const EphemerisTable &eph,
                                            const std::set<char> &systems, const double cutoffElevDeg) {
    if (eph.gps.empty() && eph.bds.empty()) return Eigen::Vector3d::Zero();
    SPP spp;
    spp.ephTable = eph;
    spp.setEnabledSystems(systems);
    spp.setCutoffElevDeg(cutoffElevDeg);
    ObsData work = obs;
    work.antennaPosition = Eigen::Vector3d::Zero();
    try { if (spp.processEpoch(work)) return spp.result.xyz; } catch (...) {
    }
    return Eigen::Vector3d::Zero();
}

bool RTK::processEpoch(const ObsData &base, const ObsData &rover, EphemerisTable &eph) {
    result.reset();
    mWeightReady = false; // 随机模型按本历元卫星集合重算
    // 与 SPP::preprocess 同语义：LSQ 档每历元清掉跨历元状态，estimatePosition 随即落到引导分支。
    if (!mUseFilter && mFilterInit) resetFilterState();

    if (mBaseXyzUser.norm() > 1e3) {
        mBaseXyz = mBaseXyzUser;
    } else if (base.antennaPosition.norm() > 1e3) {
        mBaseXyz = base.antennaPosition;
    } else {
        if (const std::size_t es = eph.gps.size() + eph.bds.size() + eph.gal.size() + eph.glo.size();
            mBaseXyz.norm() < 1e3 || es > mBaseEphSize) {
            const Eigen::Vector3d p = estimateApproxPosition(base, eph, mConfig.enabledSystems,
                                                             mConfig.cutoffElevRad * RAD_TO_DEG);
            if (p.norm() > 1e3) {
                mBaseXyz = p;
                mBaseEphSize = es;
            }
        }
    }
    if (mBaseXyz.norm() < 1e3) {
        LOG_WARN << "[RTK] base position not available (no BESTPOS/header and SPP fallback failed)";
        return false;
    }

    if (rover.antennaPosition.norm() > 1e3) {
        mRoverApprox = rover.antennaPosition;
    } else if (mUseFilter && mFilterInit && mFilterXyz.norm() > 1e3) {
        // 滤波档：上一历元滤波解就是最准的概略坐标，不必每历元再跑一遍伪距粗解。
        mRoverApprox = mFilterXyz;
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
    mRtkRover = &rover;
    mEph = &eph;

    detectFreqCombinations();
    detectSlips();

    //构建双差几何 + 选参考星 + 记录参与卫星集合
    if (!buildGeometry()) return false;
    // 站间单差方差（mSdVar）
    computeSdVariance();
    buildDDCache();

    // 位置链路：双差浮点解 → 回填 DOP/残差 → 模糊度固定
    if (!estimatePosition()) return false;
    estimateVelocity();
    mPrevRefSat = mRtkRefSat; // 下一历元据此判断是否要搬基准
    return true;
}

bool RTK::estimatePosition() {
    // 与 SPP 同一骨架：只按引导标志分派；LSQ 档靠 processEpoch 里每历元的重置落到引导分支。
    if (mFilterInit) {
        updateFilter();
    } else {
        initFilter();
    }
    if (mFilterXyz.norm() < 1.0e6) return false;

    result.xyz = mFilterXyz;
    result.blh = Coord::XYZToBLH(mFilterXyz, Frame::WGS84);
    result.xyzFloat = mFilterXyz; // 浮点解单独留存：固定时 result.xyz 会被覆盖
    mBaselineFloat = mFilterXyz - mBaseXyz;

    const Eigen::Matrix3d Q = positionCov(mFloat);
    result.sigmaXYZ = Eigen::Vector3d(std::sqrt((std::max)(Q(0, 0), 0.0)),
                                      std::sqrt((std::max)(Q(1, 1), 0.0)),
                                      std::sqrt((std::max)(Q(2, 2), 0.0)));
    result.sigmaP = positionSigma(Q, mFloat.sigma0);
    result.sigmaPFloat = result.sigmaP; // 浮点 σ 单独留存，固定时不被覆盖
    result.ratio = 0.0;
    result.stat = 2; // Float

    readback(mFloat.eq, mFloat.v, mFilterXyz);

    if (mConfig.useAR) {
        Eigen::VectorXd b;
        Eigen::MatrixXd Qb;
        std::vector<Variable> aVars;
        extractAmbiguities(mFloat, b, Qb, aVars);
        if (tryFixAmbiguities(aVars, b, Qb)) {
            applyFixedSolution(mFloat.sigma0);
            return true;
        }
    }
    result.xyzFixed = result.xyz;
    result.blhFixed = result.blh;
    return true;
}

void RTK::resetFilterState() {
    ekf.reset();
    mFilterInit = false;
    mFloat = FloatState{};
    mPrevRefSat.clear();
}

void RTK::initFilter() {
    // 引导：LSQ 迭代出位置与模糊度浮点解；LSQ 档到此为止，滤波档再往下灌一次初值。
    EquSys eqFloat;
    SolverLSQ solverFloat;
    Eigen::Vector3d baseline;
    if (!solveFloat(eqFloat, solverFloat, baseline)) return;

    const int numUnk = static_cast<int>(solverFloat.currentUnkSet.size());
    mFloat.eq = eqFloat;
    mFloat.x = solverFloat.state;
    mFloat.P = solverFloat.covMatrix;
    mFloat.v = solverFloat.v;
    mFloat.sigma0 = mConfig.fullCovariance
        ? varianceFactor(solverFloat.v.dot(mWeightMatrix * solverFloat.v),
                         static_cast<int>(solverFloat.v.size()) - numUnk)
        : posteriorSigma0(solverFloat.v, eqFloat.weightVector(), numUnk);
    mFloat.index.clear();
    int k = 0;
    for (const auto &v: eqFloat.varSet) mFloat.index[v] = k++;
    mFilterXyz = mBaseXyz + baseline;

    if (!mUseFilter) {
        mFilterInit = true; // 下一历元 processEpoch 会清掉，等价逐历元独立解
        return;
    }

    ekf.reset();
    // 不接常速度模型：RTK 浮点位置本来就被相位直接约束到厘米级，动力学模型只在测速可靠时才有增益，
    // 而多普勒在遮挡/低仰角下常只剩几颗星（σ 到 cm/s），CV 会把垃圾速度当先验、把位置往错方向推。
    ekf.configure(mPosProcSigma, false, mConfig.ambProcSigma, 0.0);
    ekf.accProcSigma = 0.0;
    ekf.ambInitSigma = mConfig.ambInitSigma;

    // 变量集必须先于 timeUpdate 定型；这次几何同时把每条相位的模糊度初值反算出来，
    // 与 LSQ 的 state 语境无关——那是相对各自线性化点的改正量，直接当绝对周数会差几十周。
    AmbSeed seed;
    EquSys eq;
    buildEquSys(eq, mFilterXyz, false, &seed);
    ekf.timeUpdate(eq.varSet, nullptr);
    ekf.setBootstrapPosition(mFilterXyz, Eigen::Vector3d::Constant(kBootPosVar));
    ekf.setBootstrapStates(seed.init);

    buildEquSys(mFloat.eq, mFilterXyz, false);
    ekf.measUpdate(mFloat.eq, mConfig.fullCovariance ? &mWeightMatrix : nullptr);
    mFilterXyz = filteredPosition();
    captureFilteredState();
    mFilterInit = true;
}

void RTK::rebaseAmbiguities() {
    if (!mFilterInit || mPrevRefSat.empty() || ekf.currentUnkSet.empty()) return;

    const VariableSet &oldSet = ekf.currentUnkSet;
    std::map<Variable, int> oldIndex;
    std::map<char, std::set<SatID> > oldSats;
    for (const auto &v: oldSet) {
        oldIndex[v] = ekf.indexOf(v);
        if (v.getParaType() == Parameter::ambiguity) oldSats[v.getObsID().satSys[0]].insert(v.getSat());
    }

    // 逐系统判定：参考星没变就原样保留；新参考星上历元没状态就整段按新弧段重开
    std::set<char> changed, reopen;
    for (const auto &[sys, part]: mRtkPartSats) {
        const auto itNew = mRtkRefSat.find(sys);
        if (itNew == mRtkRefSat.end()) continue;
        const auto itOld = mPrevRefSat.find(sys);
        if (itOld == mPrevRefSat.end() || itOld->second == itNew->second) continue;
        if (oldSats[sys].count(itNew->second)) changed.insert(sys);
        else reopen.insert(sys);
    }
    if (changed.empty() && reopen.empty()) return;

    // 新参数化逐行给出：非模糊度状态取恒等；模糊度按 N'_{i,b} = N_{i,a} − N_{b,a} 组合，
    // 旧参考星 a 变成普通星（N'_{a,b} = −N_{b,a}），新参考星 b 退出状态。
    std::map<Variable, std::vector<std::pair<int, double> > > rowMap;
    for (const auto &v: oldSet) {
        const auto type = v.getParaType();
        if (type != Parameter::ambiguity) {
            rowMap[v] = {{oldIndex.at(v), 1.0}};
            continue;
        }
        const char sys = v.getObsID().satSys[0];
        if (changed.count(sys) || reopen.count(sys)) continue;
        rowMap[v] = {{oldIndex.at(v), 1.0}};
    }
    for (const char sys: changed) {
        const SatID b = mRtkRefSat.at(sys);
        const SatID a = mPrevRefSat.at(sys);
        for (int f = 1; f <= 2; ++f) {
            const auto itb = oldIndex.find(ambVar(b, f));
            if (itb == oldIndex.end()) continue;
            for (const SatID &sat: mRtkPartSats.at(sys)) {
                if (sat == b) continue;
                const Variable vi = ambVar(sat, f);
                if (sat == a) {
                    rowMap[vi] = {{itb->second, -1.0}};
                } else if (const auto iti = oldIndex.find(vi); iti != oldIndex.end()) {
                    rowMap[vi] = {{iti->second, 1.0}, {itb->second, -1.0}};
                }
                // 上历元没有的星不入 rowMap，留给 timeUpdate 按新弧段初始化
            }
        }
    }

    VariableSet newOrder;
    for (const auto &[v, row]: rowMap) newOrder.insert(v);
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(static_cast<int>(rowMap.size()), static_cast<int>(oldSet.size()));
    int r = 0;
    for (const auto &v: newOrder) {
        for (const auto &[col, coef]: rowMap.at(v)) T(r, col) = coef;
        ++r;
    }
    ekf.reparameterize(newOrder, T);
}

void RTK::updateFilter() {
    // 参考星换了就得先把状态搬到新基准，否则本历元的 λN 与状态里的 N 不是同一个基准
    rebaseAmbiguities();
    // 先用上历元后验走一遍几何：只为给新弧段反算模糊度初值（变量集与线性化点无关，这次够用）
    AmbSeed seed;
    EquSys eq;
    buildEquSys(eq, mFilterXyz, false, &seed);
    for (const auto &[v, n]: seed.init) ekf.forceValue(v, n);
    ekf.timeUpdate(eq.varSet, &seed.novar); // 周跳/新弧段标记在这里被消费
    // 再拿预测后的先验位置重算几何，让 prefit/H 与先验严格对齐（新息口径）。
    // 注意必须先经 timeUpdate：filteredPosition() 读的是上一历元后验，不是 x⁻。
    const Eigen::Vector3d xPred = predictedPosition();
    buildEquSys(mFloat.eq, xPred, false);
    // 粗差门限按星成组剔：门限 = 4σ（σ 含先验位置不确定度投影 + 观测噪声，逐行算）
    const Eigen::VectorXd sigPred = ekf.priorInnovationStd(mFloat.eq);
    std::set<SatID> bad;
    int irow = 0;
    for (const auto &[eid, ed]: mFloat.eq.obsEquData) {
        const double sigObs = std::sqrt(1.0 / ed.weight);
        const double sig = irow < sigPred.size() ? std::hypot(sigPred(irow), sigObs) : sigObs;
        if (std::fabs(ed.prefit) > kPrefitGateSigma * sig) bad.insert(eid.sat);
        ++irow;
    }
    if (!bad.empty()) {
        EquSys gated = mFloat.eq;
        for (auto it = gated.obsEquData.begin(); it != gated.obsEquData.end();)
            it = bad.count(it->first.sat) ? gated.obsEquData.erase(it) : std::next(it);
        if (gated.obsEquData.size() >= kGateMinObs) {
            mFloat.eq = std::move(gated);
            mWeightReady = false; // 行数变了，权阵必须按新行序重建
            buildWeightMatrix(mFloat.eq);
        }
    }
    ekf.measUpdate(mFloat.eq, mConfig.fullCovariance ? &mWeightMatrix : nullptr);
    mFilterXyz = filteredPosition();
    captureFilteredState();
}

void RTK::applyFixedSolution(const double s0) {
    result.xyz = mBaseXyz + mBaselineFixed;
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

Eigen::Matrix3d RTK::positionCov(const FloatState &fs) const {
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * 1.0e12;
    const Variable vars[3] = {Variable("rover", Parameter::dX), Variable("rover", Parameter::dY),
                              Variable("rover", Parameter::dZ)};
    int k[3] = {-1, -1, -1};
    for (int a = 0; a < 3; ++a) {
        const auto it = fs.index.find(vars[a]);
        if (it != fs.index.end()) k[a] = it->second;
    }
    if (k[0] < 0 || k[1] < 0 || k[2] < 0) return Q;
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b) Q(a, b) = fs.P(k[a], k[b]);
    return Q;
}

void RTK::captureFilteredState() {
    mFloat.x = ekf.solution;
    mFloat.P = ekf.covMatrix;
    mFloat.v = ekf.postfitResidual;
    mFloat.sigma0 = posteriorSigma0(ekf.postfitResidual, mFloat.eq.weightVector(),
                                    static_cast<int>(ekf.currentUnkSet.size()));
    mFloat.index.clear();
    for (const auto &v: ekf.currentUnkSet) mFloat.index[v] = ekf.indexOf(v);
}

void RTK::extractAmbiguities(const FloatState &fs, Eigen::VectorXd &b, Eigen::MatrixXd &Qb,
                             std::vector<Variable> &aVars) const {
    // 只取本历元确实有观测的星：门限剔除后留下的变量没有方程，取整会让 LAMBDA 选错
    std::set<SatID> used;
    for (const auto &[eid, ed]: fs.eq.obsEquData) used.insert(eid.sat);

    aVars.clear();
    std::vector<int> idx;
    for (const auto &v: fs.eq.varSet) {
        if (v.getParaType() != Parameter::ambiguity || !used.count(v.getSat())) continue;
        const auto it = fs.index.find(v);
        if (it == fs.index.end() || it->second < 0) continue;
        aVars.push_back(v);
        idx.push_back(it->second);
    }

    const int M = static_cast<int>(aVars.size());
    b.resize(M);
    Qb.resize(M, M);
    for (int i = 0; i < M; ++i) {
        b(i) = fs.x(idx[i]);
        for (int j = 0; j < M; ++j) Qb(i, j) = fs.P(idx[i], idx[j]);
    }
}

void RTK::estimateVelocity() {
    if (!mRtkRover || result.xyz.norm() < 1.0e6) return;
    // 权只看相对高低：σ0 由后验残差自估，绝对量级会被吸收掉。
    std::vector<Velocity::DopplerSat> list;
    list.reserve(satPVTRoverRecTime.size());
    for (const auto &[sat, pvt]: satPVTRoverRecTime) {
        constexpr Weight w;
        const auto it = mRtkRover->satTypeValueData.find(sat);
        if (it == mRtkRover->satTypeValueData.end()) continue;
        // 第一频点的多普勒；拿不到就不参与（不拿别的频点凑数，波长对不上）。
        double doppler = 0.0;
        bool found = false;
        for (const auto &[k, v]: it->second) {
            if (k.size() >= 2 && k[0] == 'D' && k[1] == '1') {
                doppler = v;
                found = true;
                break;
            }
        }
        if (!found) continue;

        Velocity::DopplerSat ds;
        ds.sat = sat;
        ds.pos = pvt.p;
        ds.vel = pvt.v;
        ds.freqHz = sat.system == 'R' ? getGlonassFreq('1', sat.chan) : getFreq(sat.system, "L1");
        ds.dopplerHz = doppler;
        if (const auto e = satRoverElevData.find(sat); e != satRoverElevData.end())
            ds.posWeight = w.weight(e->second);
        list.push_back(ds);
    }
    const Velocity::DopplerResult v = Velocity::byDoppler(result.xyz, list);
    if (!v.ok) return;
    result.vel = v.vel;
    result.sigmaVel = v.sigma;
    result.sigmaV = v.sigma.norm();
}

void RTK::detectFreqCombinations() {
    mRtkCombos.clear();
    for (char sys: mConfig.enabledSystems) {
        SysCombo sc;
        sc.base = FreqCombo::detect(sys, *mRtkBase, true);
        sc.rover = FreqCombo::detect(sys, *mRtkRover, true);
        if (!sc.base.code1.empty() && !sc.rover.code1.empty()) sc.ok = true;
        if (!sc.ok) continue;
        mRtkCombos[sys] = sc;
    }
}

//构建双差几何、选参考星、记录参与卫星
bool RTK::buildGeometry() {
    mUsedSats.clear();
    satElevData.clear();
    satAzimData.clear();
    satRoverElevData.clear();
    satPVTTransTime.clear();
    satPVTRecTime.clear();
    satPVTRoverRecTime.clear();
    mSdVar.clear();
    mRtkPartSats.clear();
    mRtkRefSat.clear();

    // 流动站概略坐标拿不到时按基准站处理，与 buildEquSys 的迭代起点一致
    const Eigen::Vector3d &roverRef = mRoverApprox.norm() > 1e3 ? mRoverApprox : mBaseXyz;

    for (char sys: mConfig.enabledSystems) {
        auto itc = mRtkCombos.find(sys);
        if (itc == mRtkCombos.end() || !itc->second.ok) continue;
        const SysCombo &sc = itc->second;

        std::vector<SatID> sysPart;
        for (const auto &[sat, tvb]: mRtkBase->satTypeValueData) {
            if (sat.system != sys) continue;
            auto itr = mRtkRover->satTypeValueData.find(sat);
            if (itr == mRtkRover->satTypeValueData.end()) continue;
            if (const TypeValueMap &tvr = itr->second; !sc.base.available(tvb) || !sc.rover.available(tvr)) continue;
            if (mRtkSlipExclude.count(sat)) continue; // 周跳剔除：不参与本历元双差

            Ephemeris *e = mEph->find(sat, mRtkBase->epoch);
            if (!e) continue;

            // 发射时刻（用基准站 L1 伪距做传播时延修正）
            const double Pb = tvb.at(sc.base.code1);

            CommonTime t_emit = mRtkBase->epoch;
            t_emit.m_sod -= Pb / C_MPS;
            const PVT pvt = e->getPVT(t_emit, &sc.base);
            const PVT pvtBase = Coord::earthRotationCorrected(pvt, mBaseXyz);
            const PVT pvtRover = Coord::earthRotationCorrected(pvt, roverRef);

            const double elev = Coord::elevation(mBaseXyz, pvtBase.p, Frame::WGS84);
            const double azim = Coord::azimuth(mBaseXyz, pvtBase.p, Frame::WGS84);
            if (elev < mConfig.cutoffElevRad) continue;

            // 缓存 PVT/仰角/方位
            satPVTTransTime[sat] = pvt;
            satPVTRecTime[sat] = pvtBase;
            satPVTRoverRecTime[sat] = pvtRover;
            satElevData[sat] = elev;
            satAzimData[sat] = azim;
            satRoverElevData[sat] = Coord::elevation(roverRef, pvtRover.p, Frame::WGS84);
            sysPart.push_back(sat);
        }
        if (sysPart.size() < 2) continue; // 双差至少需 2 颗
        // 参考星每历元按仰角重选：状态是站间单差量，换基准只改变双差的组合方式，状态本身连续
        const std::optional<SatID> ref = selectRtkRefSat(sysPart);
        if (!ref) continue;
        mRtkRefSat[sys] = *ref;
        mRtkPartSats[sys] = std::move(sysPart);
    }

    if (mRtkPartSats.empty()) return false;

    // 记录实际参与双差解算的卫星集合
    for (auto &[sys, vec]: mRtkPartSats)
        for (auto &sat: vec) mUsedSats.insert(sat);
    int nCov = 0;
    for (auto &[s, m]: mRtkPartSats) nCov += static_cast<int>(m.size());
    result.numSats = nCov;
    if (nCov < mConfig.minSats) return false;
    return true;
}

// 预取双差方程「迭代不变」数据到 mDDCache
void RTK::buildDDCache() {
    mDDCache.clear();
    for (auto &[sys, vec]: mRtkPartSats) {
        const SatID &ref = mRtkRefSat.at(sys);
        const SysCombo &sc = mRtkCombos.at(sys);
        const TypeValueMap &tvbRef = mRtkBase->satTypeValueData.at(ref);
        const TypeValueMap &tvrRef = mRtkRover->satTypeValueData.at(ref);
        const double refBase[4] = {
            tvbRef.at(sc.base.phase1), tvbRef.at(sc.base.phase2),
            tvbRef.at(sc.base.code1), tvbRef.at(sc.base.code2)
        };
        const double refRover[4] = {
            tvrRef.at(sc.rover.phase1), tvrRef.at(sc.rover.phase2),
            tvrRef.at(sc.rover.code1), tvrRef.at(sc.rover.code2)
        };
        const auto &[rph, rco] = mSdVar.at(ref);
        for (const SatID &sat: vec) {
            if (sat == ref) continue;
            const TypeValueMap &tvb = mRtkBase->satTypeValueData.at(sat);
            const TypeValueMap &tvr = mRtkRover->satTypeValueData.at(sat);
            DDCache c;
            c.sat = sat;
            c.ref = ref;
            c.sys = sys;
            c.pvtBase = satPVTRecTime.at(sat);
            c.pvtRover = satPVTRoverRecTime.at(sat);
            c.obsBase[0] = tvb.at(sc.base.phase1);
            c.obsBase[1] = tvb.at(sc.base.phase2);
            c.obsBase[2] = tvb.at(sc.base.code1);
            c.obsBase[3] = tvb.at(sc.base.code2);
            c.obsRover[0] = tvr.at(sc.rover.phase1);
            c.obsRover[1] = tvr.at(sc.rover.phase2);
            c.obsRover[2] = tvr.at(sc.rover.code1);
            c.obsRover[3] = tvr.at(sc.rover.code2);
            for (int i = 0; i < 4; ++i) {
                c.refObsBase[i] = refBase[i];
                c.refObsRover[i] = refRover[i];
            }
            const auto &[ph, co] = mSdVar.at(sat);
            c.phaseVar = ph;
            c.codeVar = co;
            c.refPhaseVar = rph;
            c.refCodeVar = rco;
            c.amb[0] = ambVar(sat, 1);
            c.amb[1] = ambVar(sat, 2);
            c.lambda[0] = sc.base.lambda1;
            c.lambda[1] = sc.base.lambda2;
            // 双差模糊度初值 = 两站（相位减伪距）再作星间差，单位周；伪距噪声折成几周，靠 ambInitSigma 收
            for (int f = 0; f < 2; ++f) {
                const double lam = c.lambda[f];
                if (lam <= 0.0) continue;
                const double sdSat = (c.obsRover[f] - c.obsBase[f]
                                      - (c.obsRover[f + 2] - c.obsBase[f + 2])) / lam;
                const double sdRef = (c.refObsRover[f] - c.refObsBase[f]
                                      - (c.refObsRover[f + 2] - c.refObsBase[f + 2])) / lam;
                c.ambInit[f] = sdSat - sdRef;
            }
            c.refPvtBase = satPVTRecTime.at(ref);
            c.refPvtRover = satPVTRoverRecTime.at(ref);
            mDDCache.push_back(c);
        }
    }
}

// 站间单差方差：相位按高度角法定权，伪距方差 = 相位方差 × (σ 比)²
void RTK::computeSdVariance() {
    for (const auto &[sys, vec]: mRtkPartSats) {
        const SatID &ref = mRtkRefSat.at(sys);
        for (const SatID &sat: vec) {
            const double phase = mConfig.phaseWeight.variance(satElevData.at(sat))
                                 + mConfig.phaseWeight.variance(satRoverElevData.at(sat));
            mSdVar[sat] = SdVar{phase, phase * mConfig.codeOverPhaseSigma * mConfig.codeOverPhaseSigma};
        }
        const double refPhase = mConfig.phaseWeight.variance(satElevData.at(ref))
                                + mConfig.phaseWeight.variance(satRoverElevData.at(ref));
        mSdVar[ref] = SdVar{refPhase, refPhase * mConfig.codeOverPhaseSigma * mConfig.codeOverPhaseSigma};
    }
}

std::optional<SatID> RTK::selectRtkRefSat(const std::vector<SatID> &candidates) const {
    // 候选 = 系统内满足截止角/可用观测的星；可选排除北斗 GEO
    std::vector<SatID> cands;
    for (const SatID &s: candidates) {
        if (mConfig.excludeGeoRef && mEph) {
            if (const Ephemeris *e = mEph->find(s, mRtkBase->epoch); e && e->isGEO()) continue;
        }
        cands.push_back(s);
    }
    if (cands.empty()) return std::nullopt;
    // 参考星 = 流动站高度角最大者
    return *std::max_element(cands.begin(), cands.end(),
                             [&](const SatID &a, const SatID &b) {
                                 return satRoverElevData.at(a) < satRoverElevData.at(b);
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
        const auto &[phase, code] = mSdVar.at(mRtkRefSat.at(eid.sat.system));
        return eid.obsType[0] == 'L' ? phase : code;
    };

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
    std::map<std::pair<char, std::string>, std::vector<int> > blocks;
    for (int i = 0; i < m; ++i)
        blocks[{rows[i].sat.system, rows[i].obsType}].push_back(i);

    for (auto &[_, idx]: blocks) {
        const double b = refVar(rows[idx[0]]);
        for (int a = 0; a < static_cast<int>(idx.size()); ++a) {
            const double va = selfVar(rows[idx[a]]);
            for (int c = 0; c < static_cast<int>(idx.size()); ++c)
                R(idx[a], idx[c]) = a == c ? va + b : b;
        }
    }
    mWeightMatrix = R.inverse();
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

// 后处理回填：残差按 obsEquData 顺序与残差向量逐行对应（LSQ 与滤波档同一约定）
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
            auto itr = mRtkRover->satTypeValueData.find(sat);
            const bool roverOk = itr != mRtkRover->satTypeValueData.end() && sc.rover.available(itr->second);
            if (!baseOk && !roverOk) continue;

            bool slip = false;
            if (baseOk) {
                if (feedSlot(mSlipDetBase, sc.base, sat, tvb, mRtkBase)) slip = true;
                fedBase.insert(sat);
            }
            if (roverOk) {
                if (feedSlot(mSlipDetRover, sc.rover, sat, itr->second, mRtkRover)) slip = true;
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


void RTK::addDoubleDiffEquation(EquSys &eq, const DDCache &c,const Eigen::Vector3d &ddLos, double ddGeom,
                                const int freq, const bool isPhase, const bool fixed, AmbSeed *seed) {
    const SatID &sat = c.sat;
    const char sys = c.sys;
    // 观测值顺序 [L1,L2,C1,C2] 对应 (isPhase,freq) → 下标 [0,1,2,3]
    const int ci = isPhase ? (freq == 1 ? 0 : 1) : freq == 1 ? 2 : 3;
    const double lam = isPhase ? c.lambda[freq - 1] : 0.0;
    const std::string label = freqLabel(freq, isPhase);
    const double dd = c.obsRover[ci] - c.obsBase[ci]
                      - (c.refObsRover[ci] - c.refObsBase[ci]);

    EquData ed;
    ed.prefit = dd - ddGeom;
    const double var = isPhase ? c.refPhaseVar + c.phaseVar : c.refCodeVar + c.codeVar;
    ed.weight = 1.0 / var;
    const Variable dx("rover", Parameter::dX);
    const Variable dy("rover", Parameter::dY);
    const Variable dz("rover", Parameter::dZ);
    ed.varCoeffData[dx] = ddLos.x();
    ed.varCoeffData[dy] = ddLos.y();
    ed.varCoeffData[dz] = ddLos.z();
    if (isPhase) {
        if (fixed) {
            // 整周已知：扣掉 λ(N_sat − N_ref)。fixedMap 存的是相对参考星的双差整周，参考星缺项按 0 计。
            const auto &fm = mRtkFixedMap.at(sys).at(label);
            const auto itRef = fm.find(c.ref);
            ed.prefit += lam * (fm.at(sat) - (itRef == fm.end() ? 0.0 : itRef->second));
        } else {
            // 参数 x = −N：方程 l = −λ·x + v，系数取 −λ；滤波档新息 ν = prefit + λx⁻
            const Variable &amb = c.amb[freq - 1];
            ed.varCoeffData[amb] = -lam;
            if (mUseFilter) {
                if (ekf.currentUnkSet.count(amb)) {
                    ed.prefit += lam * ekf.priorOf(amb);
                } else if (seed != nullptr) {
                    // 新弧段：初值用相位减伪距（课件第 15 页），与线性化点无关
                    seed->init[amb] = -c.ambInit[freq - 1];
                    seed->novar[amb] = 2.0;
                }
            }
        }
    }
    eq.obsEquData[EquID(sat, label)] = ed;
}

bool RTK::solveFloat(EquSys &eq, SolverLSQ &solver, Eigen::Vector3d &baseline) {
    Eigen::Vector3d roverPos = mRoverApprox.norm() > 1e3 ? mRoverApprox : mBaseXyz;
    for (int iter = 0; iter < mConfig.maxIterFloat; ++iter) {
        buildEquSys(eq, roverPos, false);
        if (eq.obsEquData.size() < 4) return false;
        // 中间迭代跳过协方差 O(n³)
        solveIteration(solver, eq);
        auto dx = Eigen::Vector3d(solver.getSolution(Parameter::dX),
                                  solver.getSolution(Parameter::dY),
                                  solver.getSolution(Parameter::dZ));
        roverPos += dx;
        if (dx.norm() < mConfig.convEps) break;
    }
    solver.computeCovariance(); // 用最终迭代的正规方程分解补算协方差
    baseline = roverPos - mBaseXyz;
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
    for (const auto &c: mDDCache) {
        eq.varSet.insert(c.amb[0]);
        eq.varSet.insert(c.amb[1]);
    }
}

void RTK::buildEquSys(EquSys &eq, const Eigen::Vector3d &roverPos, const bool fixed, AmbSeed *seed) {
    eq.obsEquData.clear();
    eq.station = "rtk";
    collectVars(eq, fixed);

    for (auto &c: mDDCache) {
        // 固定解只用被 AR 取到的星：整周缺项（秩不足被跳过）的星整组不进方程
        if (fixed) {
            const auto its = mRtkFixedMap.find(c.sys);
            bool covered = its != mRtkFixedMap.end();
            for (const char *lb: {"L1", "L2"})
                covered = covered && its->second.count(lb) && its->second.at(lb).count(c.sat);
            if (!covered) continue;
        }
        // 参考星几何：站端各用自己旋转后的星位，迭代内随 roverPos 重算
        double rhoBref = 0.0, rhoRref = 0.0;
        Eigen::Vector3d losRref;
        Coord::geometricRangeAndLos(c.refPvtBase.p, mBaseXyz, rhoBref, losRref);
        Coord::geometricRangeAndLos(c.refPvtRover.p, roverPos, rhoRref, losRref);

        double rhoB = 0.0, rhoR = 0.0;
        Eigen::Vector3d losR;
        Coord::geometricRangeAndLos(c.pvtBase.p, mBaseXyz, rhoB, losR);
        Coord::geometricRangeAndLos(c.pvtRover.p, roverPos, rhoR, losR);

        const double ddGeom = rhoR - rhoB - (rhoRref - rhoBref);
        const Eigen::Vector3d ddLos = losR - losRref;

        // 四条双差观测
        addDoubleDiffEquation(eq, c, ddLos, ddGeom, 1, true, fixed, seed);
        addDoubleDiffEquation(eq, c, ddLos, ddGeom, 2, true, fixed, seed);
        addDoubleDiffEquation(eq, c, ddLos, ddGeom, 1, false, fixed, seed);
        addDoubleDiffEquation(eq, c, ddLos, ddGeom, 2, false, fixed, seed);
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
        fixedMap[v.getObsID().satSys[0]][v.getObsID().obsType][v.getSat()] = z(i);
    }
    for (auto &[sys, byType]: fixedMap) {
        const auto itRef = mRtkRefSat.find(sys);
        if (itRef == mRtkRefSat.end()) continue;
        for (auto &[ot, bySat]: byType) bySat.emplace(itRef->second, 0.0);
    }
    mRtkFixedMap = std::move(fixedMap);

    EquSys eqFix;
    SolverLSQ solverFix;
    Eigen::Vector3d roverPos = mBaseXyz + mBaselineFloat; // 从浮点基线起步
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
    mBaselineFixed = roverPos - mBaseXyz;
    mBaselineSigmaFixed = Eigen::Vector3d(std::sqrt(solverFix.covMatrix(0, 0)),
                                          std::sqrt(solverFix.covMatrix(1, 1)),
                                          std::sqrt(solverFix.covMatrix(2, 2)));
    return true;
}

void RTK::applyOptions(const SolverOptions &o) {
    mConfig = o.rtk;
    mConfig.cutoffElevRad = o.cutoffDeg * DEG_TO_RAD;
    mConfig.enabledSystems = o.systems;
    // 滤波方向与 SPP 家族同一套词表：none = 逐历元 LSQ，其余档都走 EKF。
    // 后向/双向由 Task 层另起一个 RTK 倒序重放，这里只负责"这一趟要不要滤波"。
    mUseFilter = o.filter != SolverOptions::FilterMode::LSQ;
    // 位置过程 σ 与 SPP 共用同一个键（pos_proc_sigma）
    mPosProcSigma = o.posProcSigma;
    // 关掉滤波就把 EKF 状态清干净：留着旧状态会在下次开滤波时被当成上历元后验，跨档串味
    if (!mUseFilter) resetFilterState();
}
