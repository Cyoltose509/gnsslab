#include "Core.h"
#include "core/Const.h"
#include "Log.h"
#include "TimeStruct.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace PPP {
    namespace {
        /// 解算中断超过这么久（星数不足导致滤波冻结）就认为相位弧段不可信，模糊度必须重来
        constexpr double kAmbSolveGapReset = 5.0;
    }

    bool Core::loadSP3(const std::string &path) {
        const bool ok = mSp3.read(path, /*append=*/true);
        // 解继承产品的参考框架。
        orbitFrame = mSp3.frame;
        hasOrbitFrame = mSp3.hasFrame;
        if (!hasOrbitFrame && ok && !path.empty())
            LOG_WARN << "[PPP] " << path << " 的 #cP 行没写框架（或不在 IGS14/IGS20/ITRF* 里），按 IGS14 记";
        return ok;
    }

    bool Core::loadCLK(const std::string &path, const std::set<char> &allowedSys) {
        return mClk.read(path, allowedSys, /*append=*/true);
    }

    bool Core::loadATX(const std::string &path) {
        return mAntx.read(path);
    }

    bool Core::loadOSB(const std::string &path) {
        return mOsb.read(path);
    }

    bool Core::loadERP(const std::string &path) {
        return mErp.read(path, /*append=*/true);
    }

    bool Core::loadTRO(const std::string &path) {
        return mTro.read(path);
    }

    bool Core::loadATT(const std::string &path) {
        return mAtt.read(path);
    }

    bool Core::loadIONEX(const std::string &path) {
        return mIonex.read(path);
    }

    void Core::adoptProducts(SPP &src) {
        auto *s = dynamic_cast<Core *>(&src);
        if (!s || s == this) return;
        SPP::adoptProducts(src);
        mSp3 = std::move(s->mSp3);
        mClk = std::move(s->mClk);
        mAntx = std::move(s->mAntx);
        mOsb = std::move(s->mOsb);
        mErp = std::move(s->mErp);
        mTro = std::move(s->mTro);
        mAtt = std::move(s->mAtt);
        mIonex = std::move(s->mIonex);
        orbitFrame = s->orbitFrame;
        hasOrbitFrame = s->hasOrbitFrame;
    }

    bool Core::processEpoch(ObsData &obs) {
        if (mRcvAntenna.empty() && !obs.antType.empty()) mRcvAntenna = obs.antType;
        mFilterFrozen = false;
        const bool ok = SPP::processEpoch(obs);
        if (!ok) mGapThisEpoch = false; // 解算异常后不让中断标志串到下一历元
        return ok && !mFilterFrozen;
    }

    void Core::resetFilterState() {
        SPP::resetFilterState();
        mFirstEpoch = true;
        mForceAmbReset = true;
        mHasLastEpoch = false;
        mGapThisEpoch = false;
    }

    void Core::computeSatPos(ObsData &obsData) {
        satPVTTransTime.clear();
        for (auto const &[sat, codeList]: obsData.satTypeValueData) {
            const FreqCombo *defp = freqComboFor(sat);
            if (!defp) continue;
            const FreqCombo &def = *defp;
            if (!codeList.count(def.code1) || !codeList.count(def.code2)) continue;

            Ephemeris *eph = ephTable.find(sat, obsData.epoch);
            if (ephUnhealthy(sat, eph)) continue;

            const double obsVal = def.combineCodeFromObs(codeList);
            CommonTime t0 = obsData.epoch;
            t0.m_sod -= obsVal / C_MPS;

            const bool clkOk = mClk.contains(sat) && mClk.covers(sat, t0);
            const double dt_sv = clkOk ? mClk.getClockBias(sat, t0) : 0.0;
            CommonTime t_emit = t0;
            t_emit.m_sod -= dt_sv;

            PVT pvt;
            if (mSp3.contains(sat) && mSp3.covers(sat, t_emit)) {
                if (clkOk || mSp3.hasClock()) {
                    pvt = mSp3.getPVT(sat, t_emit);
                    if (clkOk) pvt.clockBias = mClk.getClockBias(sat, t_emit);
                } else {
                    if (!eph) continue;
                    if (mFallbackWarned.insert(sat).second)
                        LOG_WARN << "[回退] " << sat << " 精密钟差未覆盖本历元，改用广播钟差（轨道仍取 SP3）";
                    CommonTime te = t_emit;
                    te.m_sod -= eph->getPVT(t0, &def).clockBias;
                    pvt = mSp3.getPVT(sat, te);
                    pvt.clockBias = eph->getPVT(te, &def).clockBias;
                    t_emit = te;
                }
            } else {
                if (!eph) continue;
                if (mFallbackWarned.insert(sat).second)
                    LOG_WARN << "[回退] " << sat << " 精密轨道未覆盖本历元，整颗回退广播星历";
                if (!clkOk) t_emit.m_sod -= eph->getPVT(t0, &def).clockBias;
                pvt = eph->getPVT(t_emit, &def);
            }
            DCM attM;
            const DCM *att =
                    mAtt.bodyDCM(sat, static_cast<double>(t_emit.toMJD().mjd), attM) ? &attM : nullptr;
            mAntx.applySatPCO(sat, pvt.p, t_emit, def, att);
            satPVTTransTime[sat] = pvt;
        }
    }

    void Core::bootstrapFirstEpoch(ObsData &obsData) {
        if (!mFirstEpoch) return;
        for (const auto &[id, bias]: rClockBias) mClkBase[id] = bias;
        const Eigen::Vector3d ap = obsData.antennaPosition;
        mRefPos = ap.norm() > 1e6 ? ap : xyz;
        xyz = mRefPos;
        mFirstEpoch = false;
    }

    void Core::applyPriorClocks() {
        for (const auto &[sys, par]: mCdtParam) {
            rClockBias[sys] = clockPrior(sys);
        }
    }

    void Core::selectClockReference() {
        mClockRefSys = clockRefSys(activeIfSystems());
        remapClockParams(mClockRefSys);
    }

    bool Core::wantCommonClock() const {
        if (mKinematic) return false; // 运动学/LEO 不走公共钟+ISB
        if (mConfig.isbInitSigma <= 0.0) return false; // 未开启 ISB 估计
        const std::set<char> sys = activeIfSystems();
        for (const char s: sys)
            if (sysCdtParam.find(s) == sysCdtParam.end()) return false;
        return sys.size() >= 2;
    }

    Eigen::Vector3d Core::predictPosition(const double, const CommonTime &) const {
        return currentEstimatedPosition();
    }

    void Core::solve(ObsData &obsData) {
        result.reset();
        mEq.reset();
        ++mProcEpoch;

        // 每历元就地探测 IF 组合
        detectIFCombinations(obsData);
        selectClockReference();

        if (ifCodeTypes.empty() && mGloCombo.empty()) return;


        // 观测中断与长时解算冻结都要重建模糊度弧段。
        if (mHasLastObs) {
            const double obsGap = mDirection * (obsData.epoch - mLastObsTime);
            const double solveGap = mHasLastEpoch ? mDirection * (obsData.epoch - mLastEpochTime) : 0.0;
            // 门限按标称采样间隔缩放。
            if (const double solveGapLimit = std::max(kAmbSolveGapReset, 1.5 * mNominalInterval);
                (mNominalInterval > 0.0 && obsGap > 1.5 * mNominalInterval) || obsGap > 60.0 ||
                (mNominalInterval > 0.0 && solveGap > solveGapLimit)) {
                if (!(mConfig.keepAmbMaxGap > 0.0 && obsGap <= mConfig.keepAmbMaxGap)) mForceAmbReset = true;
            }
        }
        mLastObsTime = obsData.epoch;
        mHasLastObs = true;
        if (mHasLastEpoch) {
            if (const double gap = std::fabs(obsData.epoch - mLastEpochTime);
                (mNominalInterval > 0.0 && gap > 1.5 * mNominalInterval) || gap > 60.0) {
                mGapThisEpoch = true;
            } else if (mNominalInterval < 0.0 && gap >= 1.0 && gap <= 60.0) mNominalInterval = gap;
        }

        bootstrapFirstEpoch(obsData);
        applyPriorClocks();
        computeSatPos(obsData);
        earthRotation();
        computeElevAzim();
        mCurObs = &obsData;

        estimatePosition(obsData);
        if (mFilterFrozen) {
            result.xyz = Eigen::Vector3d::Zero();
            result.numSats = 0;
            if (mFilterInit) {
                xyz = currentEstimatedPosition();
                computeElevAzim();
            }
            return;
        }
        mLastEpochTime = obsData.epoch;
        mHasLastEpoch = true;
        mGapThisEpoch = false;
        estimateVelocity(obsData);
    }

    void Core::estimatePosition(ObsData &obsData) {
        SPP::estimatePosition(obsData);
        if (mFilterFrozen) return;
        const bool arFixed = mUseAR &&  tryFixAmbiguities(obsData);
        result.stat = arFixed ? 1 : 2;
        readback(obsData);

        if (arFixed) {
            result.xyz = result.xyzFixed;
            result.blh = Coord::XYZToBLH(result.xyzFixed, Frame::WGS84); // blh 跟着换成固定解，否则与 xyz 打架
        }
        xyz = result.xyz;
        computeElevAzim();
    }

    void Core::initFilter(ObsData &obsData) {
        // 原来由 solve 在分派前挡掉的「不足 4 星」，现在收进引导里。
        if (satPVTTransTime.size() < 4) {
            mFilterFrozen = true;
            return;
        }
        Eigen::Vector3d xyzRef ;
        bool deferInit = false;
        {
            if (auto [pos, flag] = solveInitialPosition(obsData); !flag) deferInit = true;
            else xyzRef = pos;
            mAmbEstimate.clear();
        }
        if (deferInit) {
            mFilterFrozen = true;
        } else {
            buildEquSys(obsData, xyzRef, mEpochStates, true);
            mFilterFrozen = static_cast<int>(mEq.obsEquData.size()) / 2 < mConfig.minSatsForUpdate;
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
                ekf.timeUpdate(mVarSet, &mEpochStates);
                ekf.setBootstrapPosition(xyzRef, Eigen::Vector3d(fpvVar, fpvVar, fpvVar));
                {
                    std::map<Variable, double> bootVals;
                    std::map<Variable, double> bootVars;
                    for (const auto &[sys, par]: mCdtParam) {
                        if (const auto it = mClkBase.find(sys); it != mClkBase.end()) {
                            // mClkBase 里参考系统存绝对钟、其余系统已存相对参考的 ISB，与方程里
                            // clockPrior(sys)+clockPrior(ref) 同口径，落状态时不能再减一次参考钟。
                            bootVals[Variable(obsData.station, par)] = it->second;
                            // 参考系统钟差用 clockInitSigma(白噪声)；ISB 载体(cdt2~4)用 isbInitSigma(常数/随机游走)。
                            const bool isbOn = par != Parameter::cdt && ekf.isbInitSigma > 0.0;
                            bootVars[Variable(obsData.station, par)] =
                                    isbOn ? ekf.isbInitSigma * ekf.isbInitSigma : ekf.clockInitSigma * ekf.clockInitSigma;
                        }
                    }
                    if (mHasTropo) {
                        bootVals[Variable(obsData.station, Parameter::ztd)] =
                                mZtdBaseValid ? mZtdBase : mConfig.ztdInitValue;
                        bootVars[Variable(obsData.station, Parameter::ztd)] =
                                mConfig.ztdInitSigma * mConfig.ztdInitSigma;
                    }
                    if (!bootVals.empty()) ekf.setBootstrapStates(bootVals, &bootVars);
                }
                ekf.setBootstrapAmbiguity(obsData.station, mAmbEstimate, nullptr);
                ekf.measUpdate(mEq);
                mFilterInit = true;
            }
        }
    }

    void Core::updateFilter(ObsData &obsData) {
        const double dt = mHasLastEpoch ? obsData.epoch - mLastEpochTime : 0.0;
        const Eigen::Vector3d xyzEst = predictPosition(dt, obsData.epoch);
        buildEquSys(obsData, xyzEst, mEpochStates, true);
        mFilterFrozen = static_cast<int>(mEq.obsEquData.size()) / 2 < mConfig.minSatsForUpdate;
        if (!mFilterFrozen) {
            ekf.timeUpdate(mVarSet, &mEpochStates);
            {
                std::map<char, double> sysDclk;
                double dclkRef = 0.0;
                bool haveRef = false;
                for (const auto &[sys, par]: mCdtParam) {
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
                    // 阈值须容得下低轨重捕获的真实钟跳（几十公里量级）。
                    if (!std::isfinite(dclk) || std::fabs(dclk) > 1.0e6) continue;
                    if (std::fabs(dclk) < 1e-12) continue;
                    sysDclk[sys] = dclk;
                    if (par == Parameter::cdt) {
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
                    const Variable vcommon(obsData.station, Parameter::cdt);
                    for (auto &[eid, ed]: mEq.obsEquData) {
                        if (const auto it = ed.varCoeffData.find(vcommon); it != ed.varCoeffData.end())
                            ed.prefit -= dclkRef * it->second; // 所有星方程均含 cdt(系数1)，
                    }
                    ekf.shiftState(vcommon, dclkRef);
                    for (char sys: mSysSeen) {
                        const Parameter par = mCdtParam.at(sys);
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
                    for (const auto &[sys, par]: mCdtParam) {
                        const Variable vclk(obsData.station, par);
                        for (auto &[eid, ed]: mEq.obsEquData) {
                            if (const auto it = ed.varCoeffData.find(vclk); it != ed.varCoeffData.end())
                                ed.prefit -= dclkRef * it->second;
                        }
                        ekf.shiftState(vclk, dclkRef);
                    }
                }
            }
            ekf.setBootstrapPosition(xyzEst, 0.0);
            ekf.setBootstrapAmbiguity(obsData.station, mAmbEstimate, &mEpochStates);
            ekf.savePrior();
            ekf.measUpdate(mEq);
        }
    }

    Core::~Core() = default;

    void Core::applyOptions(const SolverOptions &o) {
        SPP::applyOptions(o);
        const SPPConfig &c = o.spp;
        mApplySolidTide = c.applySolidTide;
        setMeasSigma(c.codeSigma, c.phaseSigma);
        setFirstEpochPosSigma(c.firstEpochPosSigma);
        setTropo(true, c.ztdInitValue, c.ztdInitSigma, c.ztdProcSigma);
        setAmbInitSigma(c.ambInitSigma);
        setZTDProcSigma(c.ztdProcSigma);
        setZTDPriorSigma(c.ztdPriorSigma);
        setPosFloorSigma(c.posFloorSigma);
        setAmbProcSigma(c.ambProcSigma);
        setTROZTDSigma(c.troZtdSigma);
        mUseAR = c.useAR;
        mUseCycleSlip = c.useCycleSlip;
        mGfSlipMult = c.gfSlipMult;
        mUseTroGrad = c.useTroGrad;
        mApplyWindup = c.applyWindup;
        setCodeBiasProcSigma(c.codeBiasProcSigma, c.codeBiasInitSigma);
        setGloIfcbProcSigma(c.gloIfcbProcSigma, c.gloIfcbInitSigma);
        setKeepAmbMaxGap(c.keepAmbMaxGap);
        mConfig.arRatioThreshold = c.arRatioThreshold;
        mConfig.arGateSigma = c.arGateSigma;
        mConfig.clockProcSigma = c.clockProcSigma;
        mConfig.minSatsForUpdate = c.minSatsForUpdate;
        mConfig.maxReanchorIter = c.maxReanchorIter;
        mWeightInit = false;
        if (o.sampleInterval > 0.0) mNominalInterval = o.sampleInterval;
    }
}
