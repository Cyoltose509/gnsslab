#include "RTK.h"
#include "Weight.h"
#include "Const.h"
#include "RinexNavStore.h"
#include "ARLambda.h"
#include "Log.h"
#include <tuple>


bool RTK::processEpoch(const ObsData &base, const ObsData &rover, EphemerisTable &eph) {
    result.reset();
    if (mBaseXyz.norm() < 1e3) {
        LOG_WARN << "[RTK] base position not set";
        return false;
    }

    mRtkBase = &base;
    mRtkRover = &rover;
    mEph = &eph;
    mDDWeightReady = false; // 随机模型按本历元卫星集合重算

    detectFreqCombinations();
    detectSlips();

    //构建双差几何 + 选参考星 + 记录参与卫星集合
    if (!buildGeometry()) return false;

    // addDoubleDiffEquation 要按 mSdVar 取双差方差，故须先于建方程填充；setupFullWeight 只负责建 R 与回写权。
    computeSdVariance();

    // 3) 浮点解（迭代最小二乘）
    EquSys eqFloat;
    SolverLSQ solverFloat;
    Vector3d baseline;
    if (!solveFloat(eqFloat, solverFloat, baseline))
        return false;
    mBaselineFloat = baseline;
    result.xyz = mBaseXyz + baseline;
    result.blh = XYZtoBLH(result.xyz, Frame::WGS84);
    result.sigmaXYZ = Vector3d(std::sqrt(solverFloat.covMatrix(0, 0)),
                               std::sqrt(solverFloat.covMatrix(1, 1)),
                               std::sqrt(solverFloat.covMatrix(2, 2)));
    result.sigmaP = result.sigmaXYZ.norm() / std::sqrt(3.0);
    result.ratio = 0.0;
    result.stat = 2; // Float

    //后处理回填
    readback(eqFloat, solverFloat, result.xyz);

    // 模糊度固定
    bool fixed = false;
    if (mConfig.useAR) fixed = tryFixAmbiguities(eqFloat, solverFloat);
    if (fixed) {
        result.xyz = mBaseXyz + mBaselineFixed;
        result.blh = XYZtoBLH(result.xyz, Frame::WGS84);
        result.xyzFixed = result.xyz;
        result.blhFixed = result.blh;
        result.sigmaXYZ = mBaselineSigmaFixed;
        result.sigmaP = mBaselineSigmaFixed.norm() / std::sqrt(3.0);
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
    // 每历元清缓存（与 SPP/PPP 的 satPVTRecTime/satElevData 同语义：仅保留本历元参与星）
    mUsedSats.clear();
    satElevData.clear();
    satAzimData.clear();
    satRoverElevData.clear();
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
            PVT pvt = e->getPVT(t_emit);

            double elev, azim;
            satElevAzim(mBaseXyz, pvt.p, elev, azim, Frame::WGS84);
            if (elev < mConfig.cutoffElevRad) continue;

            // 缓存 PVT/仰角/方位（供 buildEquSys 与 GUI，避免重复算星历）
            satPVTRecTime[sat] = pvt;
            satElevData[sat] = elev;
            satAzimData[sat] = azim;
            double elevRover, azimRover;
            satElevAzim(mRoverApprox, pvt.p, elevRover, azimRover, Frame::WGS84);
            satRoverElevData[sat] = elevRover;
            sysPart.push_back(sat);
        }
        if (sysPart.size() < 2) continue; // 双差至少需 2 颗
        // 参考星 = 流动站高度角最大者（老师 SeekReferSat），一般不选北斗 GEO 作参考星。
        std::vector<SatID> refCand;
        for (const SatID &sat: sysPart)
            if (!mConfig.excludeGeoRef || getSatType(sat.system, sat.id) != SatType::GEO) refCand.push_back(sat);
        if (refCand.empty()) continue; // 只剩 GEO，本历元该系统不参与
        const auto it = std::max_element(refCand.begin(), refCand.end(),
                                         [this](const SatID &a, const SatID &b) {
                                             return satRoverElevData.at(a) < satRoverElevData.at(b);
                                         });
        mRtkRefSat[sys] = *it;
        mRtkPartSats[sys] = std::move(sysPart);
    }

    if (mRtkPartSats.empty()) return false;
    // 记录实际参与双差解算的卫星集合（扁平，供 GUI）
    for (auto &[sys, vec]: mRtkPartSats)
        for (auto &sat: vec) mUsedSats.insert(sat);
    int nCov = 0;
    for (auto &[s, m]: mRtkPartSats) nCov += static_cast<int>(m.size());
    result.numSats = nCov;
    if (nCov < mConfig.minSats) return false;
    return true;
}

// 站间单差方差：相位按高度角法定权，伪距方差 = 相位方差 / k
void RTK::computeSdVariance() {
    for (const auto &[sys, vec]: mRtkPartSats) {
        const SatID &ref = mRtkRefSat.at(sys);
        for (const SatID &sat: vec) {
            // 单差方差 = 两端各按高度角算一次方差再相加（Weight 类的单站模型）
            const double phase = mConfig.phaseWeight.variance(satElevData.at(sat))
                                 + mConfig.phaseWeight.variance(satRoverElevData.at(sat));
            // 伪距方差 = 相位方差 × codeOverPhaseVar（伪距噪声远大于载波相位，老师给 10000:1）
            mSdVar[sat] = SdVar{phase, phase * mConfig.codeOverPhaseVar};
        }
        const double refPhase = mConfig.phaseWeight.variance(satElevData.at(ref))
                                + mConfig.phaseWeight.variance(satRoverElevData.at(ref));
        mSdVar[ref] = SdVar{refPhase, refPhase * mConfig.codeOverPhaseVar};
    }
}

void RTK::buildStochasticModel(const EquSys &eq) {
    const int m = static_cast<int>(eq.obsEquData.size());
    if (m == 0) {
        mDDWeight = MatrixXd::Zero(0, 0);
        return;
    }

    std::vector<EquID> rows(m);
    int k = 0;
    for (const auto &[eid, ed]: eq.obsEquData) rows[k++] = eid;

    MatrixXd R = MatrixXd::Zero(m, m);
    auto refVar = [&](const EquID &eid) -> double {
        const auto &[phase, code] = mSdVar.at(mRtkRefSat.at(eid.sat.system));
        return eid.obsType[0] == 'L' ? phase : code;
    };
    auto selfVar = [&](const EquID &eid) -> double {
        const auto &[phase, code] = mSdVar.at(eid.sat);
        return eid.obsType[0] == 'L' ? phase : code;
    };
    for (int i = 0; i < m; ++i) {
        const double ri = refVar(rows[i]);
        R(i, i) = ri + selfVar(rows[i]);
        // 双差共享参考星只使「同系统同类型观测(L1-L1/L2-L2/C1-C1/C2-C2)」彼此相关
        // 跨系统用不同参考星，互不相关
        for (int j = 0; j < m; ++j)
            if (j != i && rows[i].sat.system == rows[j].sat.system && rows[i].obsType == rows[j].obsType)
                R(i, j) = ri * mConfig.offDiagScale;
    }
    mDDWeight = R.inverse();
}

void RTK::setupFullWeight(EquSys &eq) {
    if (!mConfig.fullCovariance) return;
    if (!mDDWeightReady) {
        computeSdVariance();
        buildStochasticModel(eq);
        mDDWeightReady = true;
    }
    // ed.weight 只用于后验 σ0 与诊断；N = Hᵀ·P·H 由 SolverLSQ 施加满阵 P。
    int i = 0;
    for (auto &[eid, ed]: eq.obsEquData) {
        ed.weight = mDDWeight(i, i);
        ++i;
    }
}

// 后处理回填
void RTK::readback(const EquSys &eqFloat, const SolverLSQ &solverFloat, const Vector3d &posEcef) {
    const auto &P = solverFloat.covMatrix; // 状态协方差 Q_xx
    const Vector3d blh = XYZtoBLH(posEcef, Frame::WGS84);
    computePositionDops(result, P.topLeftCorner(3, 3), 0.0, blh);

    int i = 0;
    for (const auto &[eid, ed]: eqFloat.obsEquData) {
        if (eid.obsType == "C1") // L1 伪距双差（带符号）
            result.postRes[eid.sat] = solverFloat.v(i);
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
                CycleSlip &dB = mSlipDetBase[sat];
                dB.configure(sc.base);
                dB.setGfThreshold(gfThr);
                const double L1 = tvb.at(sc.base.phase1), L2 = tvb.at(sc.base.phase2);

                const double C1 = tvb.at(sc.base.code1), C2 = tvb.at(sc.base.code2);

                dB.feed(L1, L2, C1, C2, true);
                if (dB.lastGfSlip())
                    gfEpoch.record(sat, dB.lastGfArtifact());
                // MW 周跳即时剔除(与 PPP 一致)；GF 跳留待 gfEpoch.decide 统一裁决(共模/伪影抑制)
                if (dB.lastMwSlip()) slip = true;
                fedBase.insert(sat);
            }
            if (roverOk) {
                CycleSlip &dR = mSlipDetRover[sat];
                dR.configure(sc.rover);
                dR.setGfThreshold(gfThr);
                const TypeValueMap &tvr = itr->second;
                const double L1 = tvr.at(sc.rover.phase1), L2 = tvr.at(sc.rover.phase2);

                const double C1 = tvr.at(sc.rover.code1), C2 = tvr.at(sc.rover.code2);

                dR.feed(L1, L2, C1, C2, true);
                if (dR.lastGfSlip())
                    gfEpoch.record(sat, dR.lastGfArtifact());
                // MW 周跳即时剔除(与 PPP 一致)；GF 跳留待 gfEpoch.decide 统一裁决(共模/伪影抑制)
                if (dR.lastMwSlip()) slip = true;
                fedRover.insert(sat);
            }
            if (baseOk && roverOk && slip) mRtkSlipExclude.insert(sat);
        }
    }

    // 弧段中断（本历元未喂有效观测）的卫星：喂入 valid=false 断开 GF 弧段，避免重现时误报
    auto feedGap = [&](std::map<SatID, CycleSlip> &m, const std::set<SatID> &fed) {
        for (auto &[sat, det]: m)
            if (!fed.count(sat)) det.feed(0, 0, 0, 0, false);
    };
    feedGap(mSlipDetBase, fedBase);
    feedGap(mSlipDetRover, fedRover);
    SatIDSet gfExclude;
    const SatIDSet gfApply = gfEpoch.decide(mGfCommonMode, gfExclude);
    for (const auto &s: gfApply) mRtkSlipExclude.insert(s);
    for (const auto &s: gfExclude) mRtkSlipExclude.insert(s);
}


void RTK::addDoubleDiffEquation(EquSys &eq, const SatID &sat,
                                const Vector3d &ddLos, double ddGeom,
                                int freq, bool isPhase, bool fixed) const {
    const SatID &ref = mRtkRefSat.at(sat.system);

    const TypeValueMap &tvb = mRtkBase->satTypeValueData.at(sat);
    const TypeValueMap &tvr = mRtkRover->satTypeValueData.at(sat);
    const TypeValueMap &tvbRef = mRtkBase->satTypeValueData.at(ref);
    const TypeValueMap &tvrRef = mRtkRover->satTypeValueData.at(ref);
    const char sys = sat.system;
    const SysCombo &sc = mRtkCombos.at(sys);

    const std::string &codeBase = isPhase
                                      ? (freq == 1 ? sc.base.phase1 : sc.base.phase2)
                                      : freq == 1
                                            ? sc.base.code1
                                            : sc.base.code2;
    const std::string &codeRover = isPhase
                                       ? (freq == 1 ? sc.rover.phase1 : sc.rover.phase2)
                                       : freq == 1
                                             ? sc.rover.code1
                                             : sc.rover.code2;
    const double lam = isPhase ? (freq == 1 ? sc.base.lambda1 : sc.base.lambda2) : 0.0;
    // 行标签即 freqLabel：模糊度参数名与固定解查表共用同一套别名(原 "P1"/"P2" 为同频冗余别名，无人读取)。
    const std::string label = freqLabel(freq, isPhase);
    const double dd = tvr.at(codeRover) - tvb.at(codeBase)

                      - (tvrRef.at(codeRover) - tvbRef.at(codeBase));

    EquData ed;
    ed.prefit = dd - ddGeom;
    const auto &[phaseS, codeS] = mSdVar.at(sat);
    const auto &[phaseR, codeR] = mSdVar.at(ref);
    const double var = isPhase ? phaseR + phaseS : codeR + codeS;
    ed.weight = mConfig.fullCovariance ? 0.0 : 1.0 / var; // 满阵模式下随后被 P 的对角元覆盖
    const Variable dx("rover", Parameter::dX);
    const Variable dy("rover", Parameter::dY);
    const Variable dz("rover", Parameter::dZ);
    ed.varCoeffData[dx] = ddLos.x();
    ed.varCoeffData[dy] = ddLos.y();
    ed.varCoeffData[dz] = ddLos.z();
    if (isPhase) {
        if (fixed) ed.prefit += lam * mRtkFixedMap.at(sys).at(freqLabel(freq, true)).at(sat);

        else ed.varCoeffData[ambVar(sat, freq)] = -lam;
    }
    eq.obsEquData[EquID(sat, label)] = ed;
}

bool RTK::solveFloat(EquSys &eq, SolverLSQ &solver, Vector3d &baseline) {
    Vector3d roverPos = mRoverApprox.norm() > 1e3 ? mRoverApprox : mBaseXyz;
    for (int iter = 0; iter < mConfig.maxIterFloat; ++iter) {
        buildEquSys(eq, roverPos, false);
        if (eq.obsEquData.size() < 4) return false;
        solver.solve(eq, mConfig.fullCovariance ? &mDDWeight : static_cast<MatrixXd *>(nullptr));
        auto dx = Vector3d(solver.getSolution(Parameter::dX),
                           solver.getSolution(Parameter::dY),
                           solver.getSolution(Parameter::dZ));
        roverPos += dx;
        if (dx.norm() < mConfig.convEps) break;
    }
    baseline = roverPos - mBaseXyz;
    return true;
}

// 组装双差方程
void RTK::buildEquSys(EquSys &eq, const Vector3d &roverPos, bool fixed) {
    eq.obsEquData.clear();
    eq.varSet.clear();
    eq.station = "rtk";

    const Variable dx("rover", Parameter::dX);
    const Variable dy("rover", Parameter::dY);
    const Variable dz("rover", Parameter::dZ);
    eq.varSet.insert(dx);
    eq.varSet.insert(dy);
    eq.varSet.insert(dz);

    // 浮点模式：每系统每非参考星每频率一个模糊度状态
    if (!fixed) {
        std::map<std::tuple<char, SatID, int>, Variable> ambVars;
        for (auto &[sys, vec]: mRtkPartSats) {
            const SatID &ref = mRtkRefSat.at(sys);

            for (auto &sat: vec) {
                if (sat == ref) continue;
                for (int f: {1, 2}) {
                    Variable v = ambVar(sat, f);
                    ambVars[{sys, sat, f}] = v;
                    eq.varSet.insert(v);
                }
            }
        }
    }

    for (auto &[sys, vec]: mRtkPartSats) {
        const SatID &ref = mRtkRefSat.at(sys);


        // 参考星几何（基准/流动锚定，迭代不变）
        const PVT &pvtRef = satPVTRecTime.at(ref);

        double rhoBref = 0.0, rhoRref = 0.0;
        Vector3d losRref;
        geometricRangeAndLos(pvtRef.p, mBaseXyz, rhoBref, losRref); // rhoBref 用于双差；los 取下方 rover
        geometricRangeAndLos(pvtRef.p, roverPos, rhoRref, losRref); // losRref = 流动站→卫星单位向量

        for (auto &sat: vec) {
            if (sat == ref) continue;
            const PVT &pvt = satPVTRecTime.at(sat);

            double rhoB = 0.0, rhoR = 0.0;
            Vector3d losR;
            geometricRangeAndLos(pvt.p, mBaseXyz, rhoB, losR); // rhoB 用于双差(基准 LOS 抵消)
            geometricRangeAndLos(pvt.p, roverPos, rhoR, losR); // losR = 流动站→卫星单位向量

            const double ddGeom = rhoR - rhoB - (rhoRref - rhoBref);
            const Vector3d ddLos = losR - losRref;

            // 四条双差观测：L1/L2 相位 + C1/C2 伪距（相位挂模糊度、伪距不挂）
            addDoubleDiffEquation(eq, sat, ddLos, ddGeom, 1, true, fixed);
            addDoubleDiffEquation(eq, sat, ddLos, ddGeom, 2, true, fixed);
            addDoubleDiffEquation(eq, sat, ddLos, ddGeom, 1, false, fixed);
            addDoubleDiffEquation(eq, sat, ddLos, ddGeom, 2, false, fixed);
        }
    }
    setupFullWeight(eq); // 随机模型只依赖卫星集合与仰角，迭代内复用
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
    VectorXd b(M);
    MatrixXd Qb(M, M);
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
    Vector3d roverPos = mBaseXyz + mBaselineFloat; // 从浮点基线起步
    mDDWeightReady = false; // 固定解重新建一次，确保 P 与方程行序严格对齐
    for (int iter = 0; iter < mConfig.maxIterFix; ++iter) {
        buildEquSys(eqFix, roverPos, true);
        solverFix.solve(eqFix, mConfig.fullCovariance ? &mDDWeight : static_cast<MatrixXd *>(nullptr));
        auto dx = Vector3d(solverFix.getSolution(Parameter::dX),
                           solverFix.getSolution(Parameter::dY),
                           solverFix.getSolution(Parameter::dZ));
        roverPos += dx;
        if (dx.norm() < mConfig.convEps) break;
    }
    mBaselineFixed = roverPos - mBaseXyz;
    mBaselineSigmaFixed = Vector3d(std::sqrt(solverFix.covMatrix(0, 0)),
                                   std::sqrt(solverFix.covMatrix(1, 1)),
                                   std::sqrt(solverFix.covMatrix(2, 2)));
    return true;
}

std::vector<std::pair<size_t, size_t> > RTK::matchEpochPairs(
    const std::vector<ObsData> &base, const std::vector<ObsData> &rover,
    const double tolSec) {
    std::vector<std::pair<size_t, size_t> > pairs;
    size_t ri = 0;
    for (size_t bi = 0; bi < base.size(); ++bi) {
        while (ri < rover.size() && rover[ri].epoch < base[bi].epoch) ++ri;
        if (ri >= rover.size()) break;
        if (rover[ri].epoch > base[bi].epoch + tolSec) continue;
        pairs.emplace_back(bi, ri);
        ++ri;
    }
    return pairs;
}
