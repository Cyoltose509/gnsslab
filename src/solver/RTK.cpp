#include "RTK.h"
#include "Weight.h"
#include "Const.h"
#include "rinex/RINEXNavStore.h"
#include "ARLambda.h"
#include "SPP.h"
#include "CoordStruct.h"
#include "Log.h"


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
    } else {
        if (const std::size_t es =eph.gps.size() + eph.bds.size() + eph.gal.size() + eph.glo.size();
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
    mWeightReady = false; // 随机模型按本历元卫星集合重算

    detectFreqCombinations();
    detectSlips();

    //构建双差几何 + 选参考星 + 记录参与卫星集合
    if (!buildGeometry()) return false;
    // 站间单差方差（mSdVar）→ 预取「迭代不变」数据到 mDDCache，供 buildEquSys 各次迭代复用
    computeSdVariance();
    buildDDCache();

    // 3) 浮点解（迭代最小二乘）
    EquSys eqFloat;
    SolverLSQ solverFloat;
    Eigen::Vector3d baseline;
    if (!solveFloat(eqFloat, solverFloat, baseline))
        return false;
    mBaselineFloat = baseline;
    result.xyz = mBaseXyz + baseline;
    result.blh = Coord::XYZToBLH(result.xyz, Frame::WGS84);
    result.xyzFloat = result.xyz; // 浮点解 ECEF 单独留存：固定时 result.xyz 会被覆盖
    result.sigmaXYZ = Eigen::Vector3d(std::sqrt(solverFloat.covMatrix(0, 0)),
                                      std::sqrt(solverFloat.covMatrix(1, 1)),
                                      std::sqrt(solverFloat.covMatrix(2, 2)));
    result.sigmaP = result.sigmaXYZ.norm() / std::sqrt(3.0);
    result.sigmaPFloat = result.sigmaP; // 浮点 σ 单独留存，固定时不被覆盖
    result.ratio = 0.0;
    result.stat = 2; // Float

    //后处理回填
    readback(eqFloat, solverFloat, result.xyz);

    // 模糊度固定
    bool fixed = false;
    if (mConfig.useAR) fixed = tryFixAmbiguities(eqFloat, solverFloat);
    if (fixed) {
        result.xyz = mBaseXyz + mBaselineFixed;
        result.blh = Coord::XYZToBLH(result.xyz, Frame::WGS84);
        result.xyzFixed = result.xyz;
        result.blhFixed = result.blh;
        result.sigmaXYZ = mBaselineSigmaFixed;
        result.sigmaP = mBaselineSigmaFixed.norm() / std::sqrt(3.0);
        result.sigmaPFixed = mBaselineSigmaFixed.norm() / std::sqrt(3.0); // 固定 σ 单独留存
        result.stat = 1; // Fixed
    } else {
        result.xyzFixed = result.xyz;
        result.blhFixed = result.blh;
    }
    return true;
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
    mSdVar.clear();
    mRtkPartSats.clear();
    mRtkRefSat.clear();

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
            PVT pvtRec = pvt;
            applyEarthRotation(pvtRec.p, pvtRec.v, mBaseXyz);

            const double elev = Coord::elevation(mBaseXyz, pvtRec.p, Frame::WGS84);
            const double azim = Coord::azimuth(mBaseXyz, pvtRec.p, Frame::WGS84);
            if (elev < mConfig.cutoffElevRad) continue;

            // 缓存 PVT/仰角/方位
            satPVTTransTime[sat] = pvt;
            satPVTRecTime[sat] = pvtRec;
            satElevData[sat] = elev;
            satAzimData[sat] = azim;
            const double elevRover = Coord::elevation(mRoverApprox, pvtRec.p, Frame::WGS84);
            //const double azimRover = Coord::azimuth(mRoverApprox, pvt.p, Frame::WGS84);
            satRoverElevData[sat] = elevRover;
            sysPart.push_back(sat);
        }
        if (sysPart.size() < 2) continue; // 双差至少需 2 颗
        // 参考星 = 流动站高度角最大者（默认排除北斗 GEO）
        const auto ref = selectRtkRefSat(sysPart, satRoverElevData, mConfig.excludeGeoRef);
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
    mCachedRefPvt.clear();
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
        mCachedRefPvt[sys] = satPVTRecTime.at(ref);
        for (const SatID &sat: vec) {
            if (sat == ref) continue;
            const TypeValueMap &tvb = mRtkBase->satTypeValueData.at(sat);
            const TypeValueMap &tvr = mRtkRover->satTypeValueData.at(sat);
            DDCache c;
            c.sat = sat;
            c.ref = ref;
            c.sys = sys;
            c.pvt = satPVTRecTime.at(sat);
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
            c.refPvt = mCachedRefPvt[sys];
            mDDCache.push_back(c);
        }
    }
}

// 站间单差方差：相位按高度角法定权，伪距方差 = 相位方差 / k
void RTK::computeSdVariance() {
    for (const auto &[sys, vec]: mRtkPartSats) {
        const SatID &ref = mRtkRefSat.at(sys);
        for (const SatID &sat: vec) {
            const double phase = mConfig.phaseWeight.variance(satElevData.at(sat))
                                 + mConfig.phaseWeight.variance(satRoverElevData.at(sat));
            mSdVar[sat] = SdVar{phase, phase * mConfig.codeOverPhaseVar};
        }
        const double refPhase = mConfig.phaseWeight.variance(satElevData.at(ref))
                                + mConfig.phaseWeight.variance(satRoverElevData.at(ref));
        mSdVar[ref] = SdVar{refPhase, refPhase * mConfig.codeOverPhaseVar};
    }
}

std::optional<SatID> RTK::selectRtkRefSat(const std::vector<SatID> &candidates,
                                          const std::map<SatID, double> &roverElev,
                                          const bool excludeGeo) {
    // 候选 = 系统内满足截止角/可用观测的星；可选排除北斗 GEO
    std::vector<SatID> cands;
    for (const SatID &s: candidates)
        if (!excludeGeo || getSatType(s.system, s.id) != SatType::GEO) cands.push_back(s);
    if (cands.empty()) return std::nullopt;
    // 参考星 = 流动站高度角最大者
    return *std::max_element(cands.begin(), cands.end(),
                             [&](const SatID &a, const SatID &b) {
                                 return roverElev.at(a) < roverElev.at(b);
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

void RTK::solveIteration(SolverLSQ &solver, EquSys &eq) {
    if (mConfig.fullCovariance) solver.solve(eq, mWeightMatrix, false);
    else solver.solve(eq, eq.weightVector().asDiagonal().toDenseMatrix(), false);
}

// 后处理回填
void RTK::readback(const EquSys &eqFloat, const SolverLSQ &solverFloat, const Eigen::Vector3d &posEcef) {
    const auto &P = solverFloat.covMatrix; // 状态协方差 Q_xx
    const Eigen::Vector3d blh = Coord::XYZToBLH(posEcef, Frame::WGS84);
    computePositionDops(result, P.topLeftCorner(3, 3), 0.0, blh);

    int i = 0;
    for (const auto &[eid, ed]: eqFloat.obsEquData) {
        // obsType 以 'C' 开头=码(伪距)双差；以 'L' 开头=相位双差(L1/L2)
        if (eid.obsType[0] != 'L')
            result.postRes[eid.sat] = solverFloat.v(i); // 码(伪距)双差残差
        else
            result.postResPhase[eid.sat] = solverFloat.v(i); // 相位双差残差
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

    // 配检测器、喂数、GF 簿记：基站与流动站走同一条。
    // sat 走形参而不是捕获：它是结构化绑定，C++17 的 lambda 捕获不了。
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

    // 弧段中断（本历元未喂有效观测）的卫星：喂入 valid=false 断开 GF 弧段，避免重现时误报
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


void RTK::addDoubleDiffEquation(EquSys &eq, const DDCache &c,
                                const Eigen::Vector3d &ddLos, double ddGeom,
                                int freq, bool isPhase, bool fixed) const {
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
        if (fixed) ed.prefit += lam * mRtkFixedMap.at(sys).at(label).at(sat);
        else ed.varCoeffData[c.amb[freq - 1]] = -lam;
    }
    eq.obsEquData[EquID(sat, label)] = ed;
}

bool RTK::solveFloat(EquSys &eq, SolverLSQ &solver, Eigen::Vector3d &baseline) {
    Eigen::Vector3d roverPos = mRoverApprox.norm() > 1e3 ? mRoverApprox : mBaseXyz;
    for (int iter = 0; iter < mConfig.maxIterFloat; ++iter) {
        buildEquSys(eq, roverPos, false);
        if (eq.obsEquData.size() < 4) return false;
        // 中间迭代跳过协方差 O(n³)，仅最终解后补算一次（等价、提速）
        solveIteration(solver, eq);
        auto dx = Eigen::Vector3d(solver.getSolution(Parameter::dX),
                                  solver.getSolution(Parameter::dY),
                                  solver.getSolution(Parameter::dZ));
        roverPos += dx;
        if (dx.norm() < mConfig.convEps) break;
    }
    solver.computeCovariance(); // 用最终迭代的正规方程分解补算协方差（结果逐位相同）
    baseline = roverPos - mBaseXyz;
    return true;
}

// 组装双差方程
void RTK::buildEquSys(EquSys &eq, const Eigen::Vector3d &roverPos, bool fixed) {
    eq.obsEquData.clear();
    eq.varSet.clear();
    eq.station = "rtk";

    const Variable dx("rover", Parameter::dX);
    const Variable dy("rover", Parameter::dY);
    const Variable dz("rover", Parameter::dZ);
    eq.varSet.insert(dx);
    eq.varSet.insert(dy);
    eq.varSet.insert(dz);

    // 浮点模式：每非参考星每频率一个模糊度状态（从 mDDCache 取缓存变量，迭代内复用，不再重建 map）
    if (!fixed) {
        for (auto &c: mDDCache) {
            eq.varSet.insert(c.amb[0]);
            eq.varSet.insert(c.amb[1]);
        }
    }

    for (auto &c: mDDCache) {
        // 参考星几何（基准/流动锚定，迭代内随 roverPos 重算）
        const PVT &pvtRef = c.refPvt;

        double rhoBref = 0.0, rhoRref = 0.0;
        Eigen::Vector3d losRref;
        Coord::geometricRangeAndLos(pvtRef.p, mBaseXyz, rhoBref, losRref);
        Coord::geometricRangeAndLos(pvtRef.p, roverPos, rhoRref, losRref);

        const PVT &pvt = c.pvt;
        double rhoB = 0.0, rhoR = 0.0;
        Eigen::Vector3d losR;
        Coord::geometricRangeAndLos(pvt.p, mBaseXyz, rhoB, losR);
        Coord::geometricRangeAndLos(pvt.p, roverPos, rhoR, losR);

        const double ddGeom = rhoR - rhoB - (rhoRref - rhoBref);
        const Eigen::Vector3d ddLos = losR - losRref;

        // 四条双差观测
        addDoubleDiffEquation(eq, c, ddLos, ddGeom, 1, true, fixed);
        addDoubleDiffEquation(eq, c, ddLos, ddGeom, 2, true, fixed);
        addDoubleDiffEquation(eq, c, ddLos, ddGeom, 1, false, fixed);
        addDoubleDiffEquation(eq, c, ddLos, ddGeom, 2, false, fixed);
    }
    buildWeightMatrix(eq); // 随机模型只依赖卫星集合与仰角，迭代内复用
}


bool RTK::tryFixAmbiguities(const EquSys &eqFloat, SolverLSQ &solverFloat) {
    //按 varSet 顺序提取所有双差模糊度变量的状态索引
    std::vector<Variable> aVars;
    std::vector<int> aIdx;
    {
        int k = 0;
        for (const auto &var: eqFloat.varSet) {
            if (var.getParaType() == Parameter::ambiguity) {
                aVars.push_back(var);
                aIdx.push_back(k);
            }
            ++k;
        }
    }
    const int M = static_cast<int>(aVars.size());
    if (M < 2) return false;

    //双差模糊度浮点向量 b 及其协方差 Q_b
    Eigen::VectorXd b(M);
    Eigen::MatrixXd Qb(M, M);
    for (int i = 0; i < M; ++i) {
        b(i) = solverFloat.state(aIdx[i]);
        for (int j = 0; j < M; ++j)
            Qb(i, j) = solverFloat.covMatrix(aIdx[i], aIdx[j]);
    }

    //LAMBDA 整数最小二乘搜索
    const auto [z, ratio, fixed] = resolveWithRatio(b, Qb, mConfig.arThreshold);
    result.ratio = ratio;
    if (!fixed) return false;

    //固定解
    std::map<char, std::map<std::string, std::map<SatID, double> > > fixedMap;
    for (int i = 0; i < M; ++i) {
        const Variable &v = aVars[i];
        const char sys = v.getObsID().satSys[0];
        const std::string &ot = v.getObsID().obsType;
        fixedMap[sys][ot][v.getSat()] = z(i);
    }
    mRtkFixedMap = fixedMap;

    EquSys eqFix;
    SolverLSQ solverFix;
    Eigen::Vector3d roverPos = mBaseXyz + mBaselineFloat; // 从浮点基线起步
    mWeightReady = false; // 固定解重新建一次，确保 P 与方程行序严格对齐
    for (int iter = 0; iter < mConfig.maxIterFix; ++iter) {
        buildEquSys(eqFix, roverPos, true);
        // 中间迭代跳过协方差 O(n³)，仅最终解后补算一次（等价、提速）
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
    RTKConfig rc = o.rtk;
    rc.cutoffElevRad = o.cutoffDeg * DEG_TO_RAD;
    rc.enabledSystems = o.systems;
    setConfig(rc);
}
