#include "SPP.h"
#include "SolverLSQ.h"
#include "core/equ/Precision.h"
#include "core/geom/CoordConvert.h"
#include "core/obs/FreqCombo.h"
#include <Eigen/Eigen>
#include <array>
#include <utility>
#include <vector>
#include <algorithm>
#include <set>

#include "Log.h"
#include "i18n/I18n.h"
#include "core/geom/CoordStruct.h"
#include "core/obs/Weight.h"

namespace {
    constexpr double kMinValidXyz = 6.0e6;
    /// 常速度预测的最长跨历元间隔：超过此时速度已不代表本历元运动状态
    constexpr double kMaxPredictGap = 60.0;
}

bool SPP::processEpoch(ObsData &obsData) {
    mCurObs = &obsData;
    try {
        // LSQ 档每历元都从零起滤波，上一历元的滤波状态不能带进来。
        if (!mUseFilter && mFilterInit) resetFilterState();
        satRejected.clear();
        solve();
    } catch (const std::exception &e) {
        LOG_ERROR << I18n::tr("log.spp.solve_error", e.what());
        return false;
    }
    return true;
}

void SPP::publishResult() {
    // 正常历元的 result 由 runFilter 末尾的 readback 写好，这里只收冻结历元的尾。
    if (!mFilterFrozen) return;
    // 冻结：本历元不发布位置，但有初值时按预测位置补一套几何，界面读数与下一历元判据都要用。
    result.xyz = Eigen::Vector3d::Zero();
    result.numSats = 0;
    if (mFilterInit) {
        xyz = currentEstimatedPosition();
        computeElevAzim(satPVTRecTime, xyz, satElevData, &satAzimData);
    }
}

void SPP::resetFilterState() {
    ekf.reset();
    mFilterInit = false;
    mHasLastEpoch = false;
}

void SPP::initFilter() {
    ekf.configure(mConfig.posProcSigma, false, 0.0, mConfig.clockProcSigma);
    ekf.accProcSigma = mConfig.accProcSigma;
    ekf.velInitSigma = mConfig.velInitSigma;
    ekf.motionDt = 0.0;
    ekf.isbProcSigma = mConfig.isbProcSigma;
    ekf.isbInitSigma = mConfig.isbInitSigma;
    runFilter();
}

void SPP::updateFilter() {
    const double gap = mHasLastEpoch ? std::fabs(mCurObs->epoch - mLastEpochTime) : 0.0;
    ekf.motionDt = gap > 0.0 && gap <= kMaxPredictGap ? gap : 0.0;
    runFilter();
}

void SPP::prepareGeometry() {
    computeSatPos();
    earthRotation(satPVTRecTime, xyz);
    computeElevAzim(satPVTRecTime, xyz, satElevData, &satAzimData);
}

void SPP::selectClockModel(const std::set<char> &active) {
    mCommonClockActive = mConfig.isbInitSigma > 0.0 && !mKinematic && active.size() >= 2;
    mClockRefSys = 'G';
    if (!active.count('G'))
        for (const char sys: {'C', 'R', 'E'})
            if (active.count(sys)) {
                mClockRefSys = sys;
                break;
            }
    mCdtParam = sysCdtParams;
    // 只有共钟参数化才需要把参考系统搬进 cdt1：独立钟时各系统槽位与参考无关。
    if (mCommonClockActive && mClockRefSys != 'G') {
        mCdtParam[mClockRefSys] = Parameter::cdt1;
        mCdtParam.erase('G');
    }
}

void SPP::addClockTerms(EquData &ed, const SatID &sat) const {
    const std::string st(curEq.station);
    if (mCommonClockActive) {
        ed.varCoeffData[Variable(st, Parameter::cdt1)] = 1.0;
        if (const Parameter isbPar = mCdtParam.at(sat.system); isbPar != Parameter::cdt1)
            ed.varCoeffData[Variable(st, isbPar)] = 1.0;
    } else {
        ed.varCoeffData[Variable(st, mCdtParam.at(sat.system))] = 1.0;
    }
}

void SPP::bootstrapSolution() {
    lastWStats.clear();
    SolverLSQ lsq;
    const bool isbAnyOn = mConfig.isbInitSigma > 0.0;
    if (isbAnyOn) {
        rClockBias = mPrevClockBias;
    } else {
        rClockBias.clear();
    }
    xyz = mCurObs->antennaPosition;

    detectIFCombinations();
    selectClockModel(activeSystems);

    // 观测源没给近似坐标
    // 先用上一历元解，首历元再按卫星方向估一个量级正确的初值
    if (xyz.norm() < kMinValidXyz) xyz = mLastXyz;
    if (xyz.norm() < kMinValidXyz) {
        computeSatPos();
        xyz = estimateRoughPosition();
    }

    int iter(0);
    while (iter < mConfig.maxIter) {
        prepareGeometry();
        if (mCurObs->satTypeValueData.size() < 4) {
            throw SVNumException("num of satellites with valid ephemeris is less than 4 ("
                                 + std::to_string(satPVTRecTime.size()) + ")");
        }

        // 迭代两轮后，按上一轮 LSQ 的 Baarda w 剔掉最差的一颗星（每历元最多一颗）
        if (iter >= 2) {
            SatID worstSat;
            double worstW = 0.0;
            for (const auto &[sat, codeList]: mCurObs->satTypeValueData) {
                const auto itW = lastWStats.find(sat);
                if (satRejected.count(sat) || itW == lastWStats.end() || itW->second <= worstW) continue;
                worstW = itW->second;
                worstSat = sat;
            }
            if (worstW > W_THRESHOLD) satRejected.insert(worstSat);
        }
        buildEquSys();

        if (curEq.obsEquData.size() < 4) {
            throw SVNumException("num of satellites after elevation cut is less than 4");
        }

        lsq.solve(curEq, curEq.weightVector().asDiagonal().toDenseMatrix());

        const Eigen::Vector3d dxyz = {
            lsq.getSolution(Parameter::dX),
            lsq.getSolution(Parameter::dY),
            lsq.getSolution(Parameter::dZ)
        };
        xyz += dxyz;
        // 钟改正按本历元参数化累加：共钟时系统钟 = 公共钟 + 自身 ISB。
        for (const char sys: activeSystems) {
            if (mCommonClockActive) {
                const double dref = lsq.getSolution(Parameter::cdt1);
                rClockBias[sys] += sys == mClockRefSys ? dref : dref + lsq.getSolution(mCdtParam.at(sys));
            } else {
                rClockBias[sys] += lsq.getSolution(mCdtParam.at(sys));
            }
        }

        if (iter >= mConfig.minCheckIter && dxyz.norm() < mConfig.convEps) {
            break;
        }

        // 按本次解的残差算 w-test 统计量，下一轮迭代（iter>=2）据此剔掉最差的一颗星。
        lastWStats.clear();
        if (lsq.sigma0 > 0.0) {
            int iobs = 0;
            for (const auto &[eid, data]: curEq.obsEquData) {
                if (iobs >= lsq.v.size()) break;
                const double invW = data.weight > 0.0 ? 1.0 / data.weight : 0.0;
                const auto hRow = lsq.H.row(iobs);
                if (const double qvv = invW - (hRow * lsq.Q * hRow.transpose())(0, 0); qvv > 0.0)
                    lastWStats[eid.sat] = std::abs(lsq.v[iobs]) / (lsq.sigma0 * std::sqrt(qvv));
                ++iobs;
            }
        }
        iter++;
    }

    if (iter >= 15) {
        throw InvalidSolver("too many iterations without convergence");
    }
    if (isbAnyOn) {
        mPrevClockBias = rClockBias;
    }
    if (xyz.norm() > kMinValidXyz) mLastXyz = xyz; // 供下一历元当初值
}

SPP::DopplerResult SPP::estimateVelocity() {
    if (xyz.norm() < kMinValidXyz) return {};
    const DopplerResult v = GnssSolver::estimateVelocity();
    if (!v.ok || !mUseFilter || !mFilterInit) return v;

    ekf.updateVelocity(v.vel, v.sigma);
    Eigen::Vector3d velPost = Eigen::Vector3d::Zero(), sigmaVelPost = Eigen::Vector3d::Zero();
    if (!ekf.posteriorVelocity(mCurObs->station, velPost, sigmaVelPost)) return v;
    result.vel = velPost;
    result.sigmaVel = sigmaVelPost;
    result.sigmaV = sigmaVelPost.norm();
    if (!ekf.velocityCoupled) return v;
    xyz = currentEstimatedPosition();
    if (xyz.norm() > kMinValidXyz) mLastXyz = xyz;
    result.xyz = xyz;
    result.xyzFloat = xyz;
    result.blh = Coord::XYZToBLH(xyz, frame);
    return v;
}

Weight SPP::dopplerWeight() const {
    const double a = mConfig.codeSigma / std::sqrt(2.0);
    return Weight(a, a, mConfig.sinElevFloor());
}

bool SPP::wantDopplerSat(const SatID &sat, const TypeValueMap &tv) const {
    if (satRejected.count(sat)) return false;
    const FreqCombo *defp = freqComboFor(sat);
    if (!defp) return false;
    if (!tv.count(defp->code1) || !tv.count(defp->code2)) return false;
    if (satElevData.at(sat) < mConfig.cutoffElevRad)
        return false;
    return true;
}

/// 观测源没给近似坐标且还没有上一历元解时，给一个「方位正确、量级正确」的起点。
/// ① 各卫星地心方向单位矢量求和、归一化后乘地球半径。参考：
///    [1] u-blox AG, US 8,125,381 B2, 2002-10-04（Doppler 方案第 3 步）
///    [2] 谢钢. GPS 原理与接收机设计. 第 5 章「首历元初值」法之二
///    与 [1][2] 的差别：此处先单位化再平均，避免北斗 GEO/MEO 混布时高轨星权重偏大。
/// ② 再用 IF 伪距做两轮等权 Gauss-Newton（钟差当未知、不估 ISB）。
Eigen::Vector3d SPP::estimateRoughPosition() const {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &[sat, pvt]: satPVTTransTime) {
        if (pvt.p.norm() > 1.0) mean += pvt.p.normalized();
    }
    if (mean.norm() < 1e-9) return {0, 0, 0};
    Eigen::Vector3d x = 6371000.0 * mean.normalized();

    for (int iter = 0; iter < 2; ++iter) {
        Eigen::MatrixXd A(0, 4);
        Eigen::VectorXd b(0);
        std::vector<std::array<double, 5> > rows;
        rows.reserve(satPVTTransTime.size());
        for (const auto &[sat, pvt]: satPVTTransTime) {
            const FreqCombo *def = freqComboFor(sat);
            if (!def) continue;
            const auto it = mCurObs->satTypeValueData.find(sat);
            if (it == mCurObs->satTypeValueData.end()) continue;
            const TypeValueMap &tv = it->second;
            if (!tv.count(def->code1) || !tv.count(def->code2)) continue;
            const double rho = (pvt.p - x).norm();
            if (rho < 1.0) continue;
            const Eigen::Vector3d u = (pvt.p - x) / rho;
            rows.push_back({-u[0], -u[1], -u[2], 1.0, def->combineCodeFromObs(tv) - rho});
        }
        if (rows.size() < 4) break;
        A.resize(static_cast<Eigen::Index>(rows.size()), 4);
        b.resize(static_cast<Eigen::Index>(rows.size()));
        for (Eigen::Index i = 0; i < A.rows(); ++i)
            for (int c = 0; c < 4; ++c) A(i, c) = rows[static_cast<std::size_t>(i)][static_cast<std::size_t>(c)];
        for (Eigen::Index i = 0; i < A.rows(); ++i) b(i) = rows[static_cast<std::size_t>(i)][4];
        const Eigen::Vector4d d = A.colPivHouseholderQr().solve(b);
        if (!d.allFinite()) break;
        x += d.head<3>();
        if (d.head<3>().norm() < 1.0) break;
    }
    return x;
}

void SPP::runFilter() {
    const bool boot = !mFilterInit;
    if (boot) {
        bootstrapSolution();
        if (xyz.norm() < kMinValidXyz) return;
        selectClockModel(activeSystems);
    } else {
        xyz = currentEstimatedPosition();
        lastWStats.clear();
        detectIFCombinations();
        // 按上一历元的参数化读回系统钟：公共钟 = 参考钟 + ISB，独立钟 = 各自槽位。
        if (mCommonClockActive) {
            const double refClk = ekf.getSolution(Variable(mCurObs->station, Parameter::cdt1));
            for (const auto &[sys, par]: mCdtParam)
                rClockBias[sys] = sys == mClockRefSys ? refClk : refClk + ekf.getSolution(Variable(mCurObs->station, par));
        } else {
            for (const auto &[sys, par]: mCdtParam)
                rClockBias[sys] = ekf.getSolution(Variable(mCurObs->station, par));
        }
        // 读回用的是上一历元参数化，本历元参数化在系统集确定后重算。
        selectClockModel(activeSystems);
    }
    mPrevClockBias = rClockBias;

    prepareGeometry();
    if (mCurObs->satTypeValueData.size() < 4) {
        throw SVNumException("num of satellites with valid ephemeris is less than 4 ("
                             + std::to_string(satPVTRecTime.size()) + ")");
    }
    buildEquSys();
    if (curEq.obsEquData.size() < 4) {
        throw SVNumException("num of satellites after elevation cut is less than 4");
    }

    if (mUseFilter && mConfig.accProcSigma > 0.0) {
        curEq.varSet.insert(Variable(mCurObs->station, Parameter::dVX));
        curEq.varSet.insert(Variable(mCurObs->station, Parameter::dVY));
        curEq.varSet.insert(Variable(mCurObs->station, Parameter::dVZ));
    }
    ekf.timeUpdate(curEq.varSet, nullptr);
    if (boot) {
        // 状态存绝对 ECEF 坐标与绝对钟差，与 PPP 同口径。
        ekf.setBootstrapPosition(xyz, Eigen::Vector3d(10.0 * 10.0, 10.0 * 10.0, 10.0 * 10.0)); // σ=10 m
        std::map<Variable, double> clk;
        if (mCommonClockActive) {
            // 公共钟参数化：参考系统存绝对钟差，其余槽位存相对参考系统的 ISB。
            const double refClk = rClockBias.count(mClockRefSys) ? rClockBias.at(mClockRefSys) : 0.0;
            for (const auto &[sys, par]: mCdtParam)
                if (activeSystems.count(sys))
                    clk[Variable(mCurObs->station, par)] =
                            sys == mClockRefSys ? refClk : rClockBias[sys] - refClk;
        } else {
            for (const char sys: activeSystems)
                if (const auto it = mCdtParam.find(sys); it != mCdtParam.end())
                    clk[Variable(mCurObs->station, it->second)] = rClockBias[sys];
        }
        ekf.setBootstrapStates(clk);
    }
    ekf.measUpdate(curEq);
    xyz = currentEstimatedPosition();
    if (xyz.norm() > kMinValidXyz) mLastXyz = xyz;
    mFilterInit = true;
    readback();
}

Eigen::Vector3d SPP::currentEstimatedPosition() const {
    return {
        GnssEKF::getSolution(Parameter::dX, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dY, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dZ, ekf.currentUnkSet, ekf.solution)
    };
}

void SPP::readback() {
    const Eigen::Matrix3d Q = posCovariance();
    // 速度三态不进位置观测方程，算自由度时要扣掉，否则 sigma0 被系统性低估。
    const int velStates = mUseFilter && mConfig.accProcSigma > 0.0 ? 3 : 0;
    sigma0 = posteriorSigma0(ekf.postfitResidual, curEq.weightVector(),
                             static_cast<int>(ekf.currentUnkSet.size()) - velStates);
    result.blh = Coord::XYZToBLH(xyz, frame);
    geometryDOPs(result, curEq, result.blh);
    result.sigmaP = positionSigma(Q, sigma0);
    result.sigmaXYZ = {
        std::sqrt(std::max(Q(0, 0), 0.0)),
        std::sqrt(std::max(Q(1, 1), 0.0)),
        std::sqrt(std::max(Q(2, 2), 0.0))
    };
    result.xyz = xyz;
    result.xyzFloat = xyz;
    result.numSats = static_cast<int>(satPVTRecTime.size());

    int iobs = 0;
    for (const auto &[id, data]: curEq.obsEquData) {
        if (iobs >= ekf.postfitResidual.size()) break;
        result.postRes.set(id.sat, ekf.postfitResidual(iobs++));
    }
}

double SPP::tropoDelay(const double elevRad) const {
    if (!mTropoSite.valid) return 0.0;
    double mW = 0.0;
    const double dry = mTropoSite.dryDelay(mConfig.tropoMap, elevRad, &mW);
    const double tropo = dry + TropoSite::wetDelay(mTropoSite.zwdResolved(), mW);
    if (!std::isfinite(tropo) || tropo < 0.0 || tropo > 100.0) return 0.0;
    return tropo;
}

void SPP::buildEquSys() {
    if (!mWeightInit) {
        const double a = mConfig.codeSigma / std::sqrt(2.0);
        mWeight = Weight(a, a, mConfig.sinElevFloor());
        mWeightInit = true;
    }

    curEq.reset();
    curEq.station = mCurObs->station;

    const Variable dx(mCurObs->station, Parameter::dX);
    const Variable dy(mCurObs->station, Parameter::dY);
    const Variable dz(mCurObs->station, Parameter::dZ);

    mTropoSite.update(prod->tropo, mConfig.tropoMap, xyz, mCurObs->epoch, frame);
    if (!(mTropoSite.hgt > 0.0)) mTropoSite = TropoSite{};

    activeSystems.clear();

    for (auto const &[sat, codeList]: mCurObs->satTypeValueData) {
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;

        if (satRejected.count(sat)) continue;

        if (!satPVTRecTime.count(sat)) {
            satRejected.insert(sat);
            continue;
        }

        // 近似坐标还没立住时不筛星，否则首历元的截止角判据会把星筛到 0 颗。
        if (const double elev = satElevData.at(sat);
            xyz.norm() > 1000.0 && elev < mConfig.cutoffElevRad) {
            satRejected.insert(sat);
            continue;
        }

        if (!codeList.count(defp->code1) || !codeList.count(defp->code2)) {
            satRejected.insert(sat);
            continue;
        }

        addSatEquation(sat, codeList, false);
        activeSystems.insert(sat.system);
    }
    curEq.varSet.insert(dx);
    curEq.varSet.insert(dy);
    curEq.varSet.insert(dz);
    collectClockStates(curEq);
}

void SPP::collectClockStates(EquSys &eq) {
    if (mCommonClockActive) {
        // 共钟：公共钟固定占 cdt1，非参考系统各留一个 ISB 槽（参考系统自己就是 cdt1）。
        eq.varSet.insert(Variable(mCurObs->station, Parameter::cdt1));
        for (const char sys: activeSystems)
            if (const Parameter isbPar = mCdtParam.at(sys); isbPar != Parameter::cdt1)
                eq.varSet.insert(Variable(mCurObs->station, isbPar));
        return;
    }
    for (const char sys: activeSystems)
        eq.varSet.insert(Variable(mCurObs->station, mCdtParam.at(sys)));
    // 独立钟：系统间差异在方程内没有约束，用「ISB 相对上一历元不变」的伪观测锚住随机游走。
    if (!(mConfig.isbInitSigma > 0.0) || activeSystems.size() < 2) return;
    for (const char sys: activeSystems) {
        if (sys == mClockRefSys) continue;
        if (!mPrevClockBias.count(sys) || !mPrevClockBias.count(mClockRefSys)) continue;
        const double prevIsb = mPrevClockBias.at(sys) - mPrevClockBias.at(mClockRefSys);
        EquData ed;
        ed.prefit = prevIsb - (rClockBias[sys] - rClockBias[mClockRefSys]);
        ed.varCoeffData[Variable(mCurObs->station, mCdtParam.at(sys))] = 1.0;
        ed.varCoeffData[Variable(mCurObs->station, mCdtParam.at(mClockRefSys))] = -1.0;
        const double var = mConfig.isbInitSigma * mConfig.isbInitSigma
                           + mConfig.isbProcSigma * mConfig.isbProcSigma;
        ed.weight = 1.0 / var;
        eq.obsEquData[EquID(SatID('I', 0), "ISB_" + std::string(1, sys))] = ed;
    }
}


void SPP::addSatEquation(const SatID &sat, const TypeValueMap &tv, bool) {
    const FreqCombo &def = *freqComboFor(sat);
    const PVT &pvt = satPVTRecTime.at(sat);
    const double elev = satElevData.at(sat);

    double rho = (pvt.p - xyz).norm();
    if (rho < 1.0) rho = 20000000.0;
    const Eigen::Vector3d los = {
        -(pvt.p[0] - xyz[0]) / rho,
        -(pvt.p[1] - xyz[1]) / rho,
        -(pvt.p[2] - xyz[2]) / rho
    };

    const double dts = pvt.clockBias * C_MPS;
    const double rel = pvt.relativityCorrection * C_MPS;

    EquData ed;
    ed.prefit = def.combineCodeFromObs(tv) - (rho + rClockBias[sat.system] - dts - rel + tropoDelay(elev));

    ed.varCoeffData[Variable(mCurObs->station, Parameter::dX)] = los[0];
    ed.varCoeffData[Variable(mCurObs->station, Parameter::dY)] = los[1];
    ed.varCoeffData[Variable(mCurObs->station, Parameter::dZ)] = los[2];
    addClockTerms(ed, sat);

    ed.weight = mWeight.weight(elev);
    curEq.obsEquData[EquID(sat, def.code1 + "+" + def.code2)] = ed;
}

void SPP::computeSatPos() {
    satPVTTransTime.clear();
    for (auto const &[sat, codeList]: mCurObs->satTypeValueData) {
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;

        Ephemeris *eph = prod->eph.find(sat, mCurObs->epoch);
        if (!eph) continue;
        if (ephUnhealthy(sat, eph, mConfig.useHealth)) continue; // 广播电文自报不可用

        const FreqCombo &def = *defp;
        if (!codeList.count(def.code1) || !codeList.count(def.code2)) continue;
        const double obsVal = def.combineCodeFromObs(codeList);

        const double tau_total = (obsVal - rClockBias[sat.system]) / C_MPS;
        CommonTime t_emit = mCurObs->epoch;
        t_emit.m_sod -= tau_total;

        satPVTTransTime[sat] = eph->getPVT(t_emit, &def);
    }
}

void SPP::detectIFCombinations(const IFCodeTypes &defaultTypes) {
    std::map<char, std::string> sig;
    for (const auto &[sat, tv]: mCurObs->satTypeValueData) {
        if (!mConfig.enabledSystems.empty() && !mConfig.enabledSystems.count(sat.system)) continue;
        std::string &s = sig[sat.system];
        for (const auto &[code, v]: tv) {
            if (v == 0.0) continue;
            s += code;
            s.push_back(',');
        }
    }
    if (sig == ifCodeSig && (!ifCodeTypes.empty() || !mGloCombo.empty())) return;
    ifCodeSig = std::move(sig);

    std::set<char> sysList;
    for (const auto &[sat, tv]: mCurObs->satTypeValueData) sysList.insert(sat.system);

    IFCodeTypes detected;
    mGloCombo.clear();
    for (char sys: sysList) {
        if (!mConfig.enabledSystems.empty() && !mConfig.enabledSystems.count(sys)) continue;
        if (sys == 'R') {
            for (const auto &[sat, tv]: mCurObs->satTypeValueData) {
                if (sat.system != 'R') continue;
                if (FreqCombo def = FreqCombo::detect('R', tv, sat.chan, mRequirePhaseForIF); !def.code1.empty() && !def.code2.empty())
                    mGloCombo[sat] = def;
            }
        } else if (FreqCombo def = FreqCombo::detect(sys, *mCurObs, mRequirePhaseForIF);
            !def.code1.empty() && !def.code2.empty()) {
            detected[sys] = def;
        }
    }
    if (detected.empty() && mGloCombo.empty()) {
        LOG_WARN << (defaultTypes.empty()
                         ? I18n::trc("log.spp.no_if")
                         : I18n::trc("log.spp.no_if_fallback"));
        ifCodeTypes = defaultTypes;
    } else {
        ifCodeTypes = std::move(detected);
    }

    activeSystems.clear();
    for (const auto &[sys, def]: ifCodeTypes) activeSystems.insert(sys);
    for (const auto &[sat, def]: mGloCombo) activeSystems.insert(sat.system);
}

void SPP::applyOptions(const SolverOptions &o) {
    setCutoffElevDeg(o.cutoffDeg);
    setEnabledSystems(o.systems);
    mConfig.codeSigma = o.spp.codeSigma;
    mConfig.minSinElev = o.spp.minSinElev;
    mConfig.tropoMap = o.spp.tropoMap;
    mConfig.convEps = o.spp.convEps;
    mConfig.maxIter = o.spp.maxIter;
    mConfig.minCheckIter = o.spp.minCheckIter;
    mConfig.useHealth = o.spp.useHealth;
    setISBProcSigma(o.spp.isbProcSigma, o.spp.isbInitSigma);
    mConfig.posProcSigma = o.posProcSigma;
    mConfig.accProcSigma = o.accProcSigma;
    mConfig.velInitSigma = o.velInitSigma;
    mConfig.clockProcSigma = o.spp.clockProcSigma;
    mWeightInit = false;
    mUseFilter = o.filter != SolverOptions::FilterMode::LSQ;
    mFilterInit = false;
    mHasLastEpoch = false;
    ekf.reset();
}
