#include "Static.h"
#include "core/Const.h"
#include "core/equ/Precision.h"
#include "Log.h"
#include "core/TimeStruct.h"
#include <cmath>
#include <utility>

bool PPPStatic::processEpoch(ObsData &obs) {
    if (mRcvAntenna.empty() && !obs.antType.empty()) mRcvAntenna = obs.antType;
    mFilterFrozen = false;
    const bool ok = SPP::processEpoch(obs);
    if (!ok) mGapThisEpoch = false; // 解算异常后不让中断标志串到下一历元
    return ok && !mFilterFrozen;
}

PPPStatic::~PPPStatic() = default;

void PPPStatic::applyOptions(const SolverOptions &o) {
    SPP::applyOptions(o);
    const SPPConfig &c = o.spp;
    mApplySolidTide = c.applySolidTide;
    mConfig.codeSigma = c.codeSigma;
    mConfig.phaseSigma = c.phaseSigma;
    mConfig.firstEpochPosSigma = c.firstEpochPosSigma;
    setTropo(true, c.ztdInitValue, c.ztdInitSigma, c.ztdProcSigma);
    mConfig.ambInitSigma = c.ambInitSigma;
    mConfig.ztdPriorSigma = c.ztdPriorSigma;
    mConfig.posFloorSigma = c.posFloorSigma;
    mConfig.ambProcSigma = c.ambProcSigma;
    mConfig.troZtdSigma = c.troZtdSigma;
    mUseAR = c.useAR;
    mUseCycleSlip = c.useCycleSlip;
    mUseGFCycleSlip = c.useGFCycleSlip;
    mGFSlipMult = c.GFSlipMult;
    mUseTroGrad = c.useTroGrad;
    mApplyWindup = c.applyWindup;
    mConfig.codeBiasProcSigma = c.codeBiasProcSigma;
    mConfig.codeBiasInitSigma = c.codeBiasInitSigma;
    mConfig.gloIfcbProcSigma = c.gloIfcbProcSigma;
    mConfig.gloIfcbInitSigma = c.gloIfcbInitSigma;
    mKeepAmbMaxGap = c.keepAmbMaxGap;
    mConfig.arRatioThreshold = c.arRatioThreshold;
    mConfig.arGateSigma = c.arGateSigma;
    mConfig.clockProcSigma = c.clockProcSigma;
    mConfig.minSatsForUpdate = c.minSatsForUpdate;
    mConfig.maxReanchorIter = c.maxReanchorIter;
    if (o.sampleInterval > 0.0) mNominalInterval = o.sampleInterval;
}

void PPPStatic::solve() {
    result.reset();
    curEq.reset();
    ++mProcEpoch;

    // 每历元就地探测 IF 组合（同时给出参与系统集），参数化随后按这批系统重定
    detectIFCombinations();
    selectClockModel(activeSystems);
    if (ifCodeTypes.empty() && mGloCombo.empty()) return;

    bindRefPosition();
    for (const auto &[sys, par]: mCdtParam) rClockBias[sys] = clockPrior(sys);
    prepareGeometry();
    // 几何与钟初值备齐后走基类骨架：历元簿记 → 位置 → 冻结收尾 → 测速
    GnssSolver::solve();
}

void PPPStatic::bindRefPosition() {
    if (!mFirstEpoch) return;
    for (const auto &[id, bias]: rClockBias) mClkBase[id] = bias;
    const Eigen::Vector3d ap = mCurObs->antennaPosition;
    mRefPos = ap.norm() > 1e6 ? ap : xyz;
    xyz = mRefPos;
    mFirstEpoch = false;
}

void PPPStatic::resetFilterState() {
    SPP::resetFilterState();
    mFirstEpoch = true;
    mForceAmbReset = true;
    mHasLastEpoch = false;
    mGapThisEpoch = false;
}

void PPPStatic::initFilter() {
    // 原来由 solve 在分派前挡掉的「不足 4 星」，现在收进引导里。
    if (satPVTTransTime.size() < 4) {
        mFilterFrozen = true;
        return;
    }
    if (!bootstrapSolution()) {
        mFilterFrozen = true;
        return;
    }
    mEpochXyz = xyz;
    buildEquSys();
    mFilterFrozen = static_cast<int>(curEq.obsEquData.size()) / 2 < mConfig.minSatsForUpdate;
    if (mFilterFrozen) return;

    ekf.configure(mConfig.posProcSigma, mHasTropo, mConfig.ambProcSigma, mConfig.clockProcSigma);
    ekf.isbProcSigma = mConfig.isbProcSigma;
    ekf.isbInitSigma = mConfig.isbInitSigma;
    ekf.posFloorSigma = mConfig.posFloorSigma;
    ekf.ztdInitSigma = mConfig.ztdInitSigma;
    ekf.ambInitSigma = mConfig.ambInitSigma;
    ekf.ztdProcSigma = mConfig.ztdProcSigma;
    const double fpvVar = mConfig.firstEpochPosSigma * mConfig.firstEpochPosSigma;
    ekf.posFloorSigmaVec = Eigen::Vector3d(mConfig.posFloorSigma, mConfig.posFloorSigma, mConfig.posFloorSigma);
    ekf.ztdInitValue = mConfig.ztdInitValue;
    ekf.codeBiasProcSigma = mConfig.codeBiasProcSigma;
    ekf.codeBiasInitSigma = mConfig.codeBiasInitSigma;
    ekf.timeUpdate(curEq.varSet, mAmbResets);
    ekf.setPosition(xyz, fpvVar);
    {
        std::map<Variable, double> bootVals;
        std::map<Variable, double> bootVars;
        for (const auto &[sys, par]: mCdtParam) {
            if (const auto it = mClkBase.find(sys); it != mClkBase.end()) {
                // mClkBase 里参考系统存绝对钟、其余系统已存相对参考的 ISB，与方程里
                // clockPrior(sys)+clockPrior(ref) 同口径，落状态时不能再减一次参考钟。
                bootVals[Variable(mCurObs->station, par)] = it->second;
                // 参考系统钟差用 clockInitSigma(白噪声)；ISB 载体(cdt2~4)用 isbInitSigma(常数/随机游走)。
                const bool isbOn = par != Parameter::cdt1 && ekf.isbInitSigma > 0.0;
                bootVars[Variable(mCurObs->station, par)] =
                        isbOn ? ekf.isbInitSigma * ekf.isbInitSigma : ekf.clockInitSigma * ekf.clockInitSigma;
            }
        }
        if (mHasTropo) {
            bootVals[Variable(mCurObs->station, Parameter::ztd)] =
                    mZtdBaseValid ? mZtdBase : mConfig.ztdInitValue;
            bootVars[Variable(mCurObs->station, Parameter::ztd)] =
                    mConfig.ztdInitSigma * mConfig.ztdInitSigma;
        }
        if (!bootVals.empty()) ekf.setStates(bootVals, &bootVars);
    }
    ekf.setAmbiguity(mCurObs->station, mAmbEstimate, nullptr);
    ekf.measUpdate(curEq, curEq.weightVector().asDiagonal().toDenseMatrix());
    mFilterInit = true;
}

bool PPPStatic::bootstrapSolution() {
    const auto [pos, ok] = estimateInitialPosition();
    mAmbEstimate.clear(); // 引导内部建方程会顺带填它，这里重置，留给 initFilter 的正式建方程重填
    if (!ok) return false;
    xyz = pos;
    return true;
}

void PPPStatic::updateFilter() {
    const double dt = mHasLastEpoch ? mCurObs->epoch - mLastEpochTime : 0.0;
    const Eigen::Vector3d xyzEst = predictPosition(dt, mCurObs->epoch);
    mEpochXyz = xyzEst;
    buildEquSys();
    mFilterFrozen = static_cast<int>(curEq.obsEquData.size()) / 2 < mConfig.minSatsForUpdate;
    if (!mFilterFrozen) {
        const VariableSet prevUnk = ekf.currentUnkSet;
        ekf.timeUpdate(curEq.varSet, mAmbResets);
        // 系统断档期间钟状态会被移出状态集，再出现时若从 0 起步，高权重相位方程
        // 会把钟拽到错误基准，污染观测时间标记。重新入集时沿用上次的钟估计。
        for (const auto &[sys, par]: mCdtParam) {
            const auto it = mClkEstimate.find(sys);
            if (it == mClkEstimate.end()) continue;
            const Variable vclk(mCurObs->station, par);
            if (ekf.currentUnkSet.count(vclk) && !prevUnk.count(vclk))
                ekf.setStates({{vclk, it->second}}, nullptr);
        }
        {
            std::vector<double> codePhaseDiffs;
            codePhaseDiffs.reserve(mCurCodePhase.size());
            for (const auto &[sat, cp]: mCurCodePhase)
                if (const auto it = mPrevCodePhase.find(sat); it != mPrevCodePhase.end())
                    codePhaseDiffs.push_back(std::fabs(cp - it->second));
            const bool isClockJump = SlipDetector::clockJump(codePhaseDiffs);
            std::map<SatSystem, double> sysDclk;
            double dclkRef = 0.0;
            bool haveRef = false;
            for (const auto &[sys, par]: mCdtParam) {
                double sw = 0.0, swp = 0.0;
                for (const auto &[eid, ed]: curEq.obsEquData) {
                    if (eid.sat.system != sys) continue;
                    if (eid.sat.id == 0) continue; // 伪观测(ZTD 先验等)
                    if (ed.isPhase) continue; // 仅码
                    sw += ed.weight;
                    swp += ed.weight * ed.prefit;
                }
                if (!(sw > 0.0)) continue;
                const double dclk = swp / sw;
                if (!std::isfinite(dclk) || std::fabs(dclk) > (isClockJump ? 1.0e8 : 1.0e6)) continue;
                if (std::fabs(dclk) < 1e-12) continue;
                sysDclk[sys] = dclk;
                if (par == Parameter::cdt1) {
                    dclkRef = dclk;
                    haveRef = true;
                }
            }
            if (!haveRef && !sysDclk.empty()) {
                double sw = 0.0, swp = 0.0;
                for (const auto &[s, d]: sysDclk) {
                    sw += 1.0;
                    swp += d;
                }
                dclkRef = swp / sw;
            }
            if (mCommonClockActive) {
                const Variable vcommon(mCurObs->station, Parameter::cdt1);
                for (auto &[eid, ed]: curEq.obsEquData) {
                    if (const auto it = ed.varCoefficient.find(vcommon); it != ed.varCoefficient.end())
                        ed.prefit -= dclkRef * it->second; // 所有星方程均含 cdt(系数1)，
                }
                ekf.shiftState(vcommon, dclkRef);
                for (SatSystem sys: mSysSeen) {
                    const Parameter par = mCdtParam.at(sys);
                    if (par == Parameter::cdt1) continue; // 参考系统无 ISB
                    const auto itd = sysDclk.find(sys);
                    if (itd == sysDclk.end()) continue;
                    const double disb = itd->second - dclkRef; // 相对参考系统的钟差偏差
                    const Variable visb(mCurObs->station, par);
                    for (auto &[eid, ed]: curEq.obsEquData) {
                        if (eid.sat.system != sys) continue;
                        if (const auto it = ed.varCoefficient.find(visb); it != ed.varCoefficient.end())
                            ed.prefit -= disb * it->second;
                    }
                    ekf.shiftState(visb, disb);
                }
            } else {
                // 各系统独立绝对钟：按本系统码 prefit 均值重锚，避免系统间偏差漏进位置。
                for (const auto &[sys, dclk]: sysDclk) {
                    const auto it = mCdtParam.find(sys);
                    if (it == mCdtParam.end()) continue;
                    const Variable vclk(mCurObs->station, it->second);
                    for (auto &[eid, ed]: curEq.obsEquData) {
                        if (eid.sat.system != sys) continue;
                        if (const auto itc = ed.varCoefficient.find(vclk); itc != ed.varCoefficient.end())
                            ed.prefit -= dclk * itc->second;
                    }
                    ekf.shiftState(vclk, dclk);
                }
            }
        }
        ekf.setPosition(xyzEst, 0.0);
        ekf.setAmbiguity(mCurObs->station, mAmbEstimate, &mAmbResets);
        ekf.savePrior();
        ekf.measUpdate(curEq, curEq.weightVector().asDiagonal().toDenseMatrix());
    }
    mPrevCodePhase = std::move(mCurCodePhase);
    mCurCodePhase.clear();
}

Eigen::Vector3d PPPStatic::predictPosition(const double, const CommonTime &) {
    return currentEstimatedPosition();
}

void PPPStatic::publishResult() {
    SPP::publishResult();
    if (mFilterFrozen) return;
    try { result.xyz = currentEstimatedPosition(); } catch (...) {}
    result.xyzFloat = result.xyz;
    result.blh = Coord::XYZToBLH(result.xyz, Frame::WGS84);
    result.numSats = static_cast<int>(curEq.obsEquData.size()) / 2;

    for (const auto &[sys, par]: mCdtParam)
        GnssEKF::tryGetSolution(par, ekf.currentUnkSet, ekf.solution, mClkEstimate[sys]);
    for (SatSystem sys: mSysSeen) {
        if (!osbCoversSystem(sys)) {
            const Variable vcb = Variable::makeCodeBias(mCurObs->station, SatID(sys, -1), "IF");
            mCbEstimate[sys] = ekf.getSolution(vcb);
        }
    }
    // 逐 GLONASS/BDS 星 IFCB 回读
    for (const auto &[sat, tv]: mCurObs->satTypeValueData) {
        if (sat.system != SatSystem::R && sat.system != SatSystem::C) continue;
        if (osbCovers(sat)) continue;
        if (const FreqCombo *defp = freqComboFor(sat); !defp || !defp->available(tv, true)) continue;
        const Variable vifb = Variable::makeIfb(mCurObs->station, sat, "IF");
        mIfcbEstimate[sat] = ekf.getSolution(vifb);
    }
    GnssEKF::tryGetSolution(Parameter::ztd, ekf.currentUnkSet, ekf.solution, mZtdEstimate);
    for (const auto &[sat, tv]: mCurObs->satTypeValueData) {
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp || !defp->available(tv, true)) continue;
        const Variable vamb = Variable::makeAmbiguity(mCurObs->station, sat, "IF");
        mAmbEstimate[sat] = ekf.getSolution(vamb); // 不在集中直接返回 0，不抛
    }
    readbackResidual(curEq, ekf.postfitResidual);
    geometryDOPs(result, curEq, result.blh);

    const Variable vdx(mCurObs->station, Parameter::dX);
    const Variable vdy(mCurObs->station, Parameter::dY);
    const Variable vdz(mCurObs->station, Parameter::dZ);
    const Variable vox(mCurObs->station, Parameter::dOX);
    const Variable voy(mCurObs->station, Parameter::dOY);
    const Variable voz(mCurObs->station, Parameter::dOZ);
    const Eigen::MatrixXd P = ekf.Q;
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
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    Q.diagonal() << axisVar(vdx, vox), axisVar(vdy, voy), axisVar(vdz, voz);
    sigma0 = posteriorSigma0(ekf.postfitResidual, curEq.weightVector(),
                             static_cast<int>(ekf.currentUnkSet.size()));
    result.sigmaXYZ = Q.diagonal().array().max(0.0).sqrt().matrix() * sigma0;
    result.sigmaP = std::max(positionSigma(Q, sigma0), 1e-3);
    result.sigmaPFloat = result.sigmaP;

    const bool arFixed = mUseAR && tryFixAmbiguities();
    result.stat = arFixed ? 1 : 2;

    if (arFixed) {
        result.xyz = result.xyzFixed;
        result.blh = Coord::XYZToBLH(result.xyzFixed, Frame::WGS84); // blh 跟着换成固定解，否则与 xyz 打架
        result.sigmaP = result.sigmaPFixed;
    }
    xyz = result.xyz;
    computeElevAzim(satPVTRecTime, xyz, satElevData, satAzimData);
}

void PPPStatic::computeSatPos() {
    satPVTTransTime.clear();
    for (auto const &[sat, codeList]: mCurObs->satTypeValueData) {
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;
        const FreqCombo &def = *defp;
        if (!codeList.count(def.code1) || !codeList.count(def.code2)) continue;

        Ephemeris *eph = prod->eph.find(sat, mCurObs->epoch);
        if (ephUnhealthy(sat, eph, mConfig.useHealth)) continue;

        const double obsVal = def.combineCodeFromObs(codeList);
        CommonTime t0 = mCurObs->epoch;
        t0.m_sod -= obsVal / C_MPS;

        const bool clkOk = prod->clk.contains(sat) && prod->clk.covers(sat, t0);
        const double dt_sv = clkOk ? prod->clk.getClockBias(sat, t0) : 0.0;
        CommonTime t_emit = t0;
        t_emit.m_sod -= dt_sv;

        PVT pvt;
        if (prod->sp3.contains(sat) && prod->sp3.covers(sat, t_emit)) {
            if (clkOk || prod->sp3.hasClock()) {
                pvt = prod->sp3.getPVT(sat, t_emit);
                if (clkOk) pvt.clockBias = prod->clk.getClockBias(sat, t_emit);
            } else {
                if (!eph) continue;
                if (mFallbackWarned.insert(sat).second)
                    LOG_WARN << "[FALLBACK] " << sat.toString()
                            << ": precise clock does not cover this epoch, using broadcast ephemeris"
                               " (orbit still from SP3)";
                CommonTime te = t_emit;
                te.m_sod -= eph->getPVT(t0, &def).clockBias;
                pvt = prod->sp3.getPVT(sat, te);
                pvt.clockBias = eph->getPVT(te, &def).clockBias;
                t_emit = te;
            }
        } else {
            if (!eph) continue;
            if (mFallbackWarned.insert(sat).second)
                LOG_WARN << "[FALLBACK] " << sat.toString()
                        << ": precise orbit does not cover this epoch, whole satellite falls back to"
                           " the broadcast ephemeris";
            if (!clkOk) t_emit.m_sod -= eph->getPVT(t0, &def).clockBias;
            pvt = eph->getPVT(t_emit, &def);
        }
        DCM attM;
        const DCM *att =
                prod->att.bodyDCM(sat, static_cast<double>(t_emit.toMJD().mjd), attM) ? &attM : nullptr;
        prod->atx.applySatPCO(sat, pvt.p, t_emit, def, att);
        satPVTTransTime[sat] = pvt;
    }
}
