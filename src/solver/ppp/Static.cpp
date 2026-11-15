#include "Static.h"
#include "core/Const.h"
#include "i18n/I18n.h"
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

void PPPStatic::resetFilterState() {
    SPP::resetFilterState();
    mFirstEpoch = true;
    mForceAmbReset = true;
    mHasLastEpoch = false;
    mGapThisEpoch = false;
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
                    LOG_WARN << "[FALLBACK] " << I18n::tr("log.ppp.clk_fallback", sat.toString());
                CommonTime te = t_emit;
                te.m_sod -= eph->getPVT(t0, &def).clockBias;
                pvt = prod->sp3.getPVT(sat, te);
                pvt.clockBias = eph->getPVT(te, &def).clockBias;
                t_emit = te;
            }
        } else {
            if (!eph) continue;
            if (mFallbackWarned.insert(sat).second)
                LOG_WARN << "[FALLBACK] " << I18n::tr("log.ppp.orbit_fallback", sat.toString());
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

void PPPStatic::bindRefPosition() {
    if (!mFirstEpoch) return;
    for (const auto &[id, bias]: rClockBias) mClkBase[id] = bias;
    const Eigen::Vector3d ap = mCurObs->antennaPosition;
    mRefPos = ap.norm() > 1e6 ? ap : xyz;
    xyz = mRefPos;
    mFirstEpoch = false;
}

Eigen::Vector3d PPPStatic::predictPosition(const double, const CommonTime &) const {
    return currentEstimatedPosition();
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

void PPPStatic::publishResult() {
    SPP::publishResult();
    if (mFilterFrozen) return;
    const bool arFixed = mUseAR && tryFixAmbiguities();
    result.stat = arFixed ? 1 : 2;
    readback();

    if (arFixed) {
        result.xyz = result.xyzFixed;
        result.blh = Coord::XYZToBLH(result.xyzFixed, Frame::WGS84); // blh 跟着换成固定解，否则与 xyz 打架
    }
    xyz = result.xyz;
    computeElevAzim(satPVTRecTime, xyz, satElevData, &satAzimData);
}

void PPPStatic::initFilter() {
    // 原来由 solve 在分派前挡掉的「不足 4 星」，现在收进引导里。
    if (satPVTTransTime.size() < 4) {
        mFilterFrozen = true;
        return;
    }
    Eigen::Vector3d xyzRef ;
    bool deferInit = false;
    {
        if (auto [pos, flag] = estimateInitialPosition(); !flag) deferInit = true;
        else xyzRef = pos;
        mAmbEstimate.clear();
    }
    if (deferInit) {
        mFilterFrozen = true;
    } else {
        mEpochXyz = xyzRef;
        mCsData = &mEpochStates;
        buildEquSys();
        mFilterFrozen = static_cast<int>(curEq.obsEquData.size()) / 2 < mConfig.minSatsForUpdate;
        if (!mFilterFrozen) {
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
            ekf.timeUpdate(curEq.varSet, &mEpochStates);
            ekf.setBootstrapPosition(xyzRef, Eigen::Vector3d(fpvVar, fpvVar, fpvVar));
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
                if (!bootVals.empty()) ekf.setBootstrapStates(bootVals, &bootVars);
            }
            ekf.setBootstrapAmbiguity(mCurObs->station, mAmbEstimate, nullptr);
            ekf.measUpdate(curEq);
            mFilterInit = true;
        }
    }
}

void PPPStatic::updateFilter() {
    const double dt = mHasLastEpoch ? mCurObs->epoch - mLastEpochTime : 0.0;
    const Eigen::Vector3d xyzEst = predictPosition(dt, mCurObs->epoch);
    mEpochXyz = xyzEst;
    mCsData = &mEpochStates;
    buildEquSys();
    mFilterFrozen = static_cast<int>(curEq.obsEquData.size()) / 2 < mConfig.minSatsForUpdate;
    if (!mFilterFrozen) {
        ekf.timeUpdate(curEq.varSet, &mEpochStates);
        {
            std::map<char, double> sysDclk;
            double dclkRef = 0.0;
            bool haveRef = false;
            for (const auto &[sys, par]: mCdtParam) {
                double sw = 0.0, swp = 0.0;
                for (const auto &[eid, ed]: curEq.obsEquData) {
                    if (eid.sat.system != sys) continue;
                    if (eid.sat.id == 0) continue; // 伪观测(ZTD 先验等)
                    if (eid.obsType.empty() || eid.obsType[0] == 'L') continue; // 仅码
                    sw += ed.weight;
                    swp += ed.weight * ed.prefit;
                }
                if (!(sw > 0.0)) continue;
                const double dclk = swp / sw;
                // 阈值须容得下低轨重捕获的真实钟跳（几十公里量级）。
                if (!std::isfinite(dclk) || std::fabs(dclk) > 1.0e6) continue;
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
                    if (const auto it = ed.varCoeffData.find(vcommon); it != ed.varCoeffData.end())
                        ed.prefit -= dclkRef * it->second; // 所有星方程均含 cdt(系数1)，
                }
                ekf.shiftState(vcommon, dclkRef);
                for (char sys: mSysSeen) {
                    const Parameter par = mCdtParam.at(sys);
                    if (par == Parameter::cdt1) continue; // 参考系统无 ISB
                    const auto itd = sysDclk.find(sys);
                    if (itd == sysDclk.end()) continue;
                    const double disb = itd->second - dclkRef; // 相对参考系统的钟差偏差
                    const Variable visb(mCurObs->station, par);
                    for (auto &[eid, ed]: curEq.obsEquData) {
                        if (eid.sat.system != sys) continue;
                        if (const auto it = ed.varCoeffData.find(visb); it != ed.varCoeffData.end())
                            ed.prefit -= disb * it->second;
                    }
                    ekf.shiftState(visb, disb);
                }
            } else {
                // 旧口径：各系统独立绝对钟
                for (const auto &[sys, par]: mCdtParam) {
                    const Variable vclk(mCurObs->station, par);
                    for (auto &[eid, ed]: curEq.obsEquData) {
                        if (const auto it = ed.varCoeffData.find(vclk); it != ed.varCoeffData.end())
                            ed.prefit -= dclkRef * it->second;
                    }
                    ekf.shiftState(vclk, dclkRef);
                }
            }
        }
        ekf.setBootstrapPosition(xyzEst, 0.0);
        ekf.setBootstrapAmbiguity(mCurObs->station, mAmbEstimate, &mEpochStates);
        ekf.savePrior();
        ekf.measUpdate(curEq);
    }
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
    mGfSlipMult = c.gfSlipMult;
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
