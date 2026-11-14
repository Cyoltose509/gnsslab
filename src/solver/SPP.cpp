#include "SPP.h"
#include "core/equ/Precision.h"
#include "rinex/RINEXNavStore.h"
#include "core/geom/CoordConvert.h"
#include "core/obs/FreqCombo.h"
#include <Eigen/Eigen>
#include <array>
#include <utility>
#include <vector>
#include <algorithm>
#include <set>

#include "Log.h"
#include "core/geom/CoordStruct.h"
#include "core/obs/Weight.h"
#include "Velocity.h"

namespace {
    constexpr double kMinValidXyz = 6.0e6;
    /// 常速度预测的最长跨历元间隔：超过此时速度已不代表本历元运动状态
    constexpr double kMaxPredictGap = 60.0;
}

bool SPP::processEpoch(ObsData &obsData) {
    try {
        preprocess(obsData);
        solve(obsData);
    } catch (const std::exception &e) {
        LOG_ERROR << "[SPP] solve 异常: " << e.what();
        return false;
    }
    return true;
}

void SPP::preprocess(ObsData &obsData) {
    // 清跨历元状态必须在 solve() 之前：派生类的 gap 检测与首历元引导要读这些标志。
    if (!mUseFilter && mFilterInit) resetFilterState();
    satRejected.clear();
}

bool SPP::loadBRDC(const std::string &path) {
    RINEXNavStore store;
    store.loadFile(path, ephTable);
    return !ephTable.gps.empty() || !ephTable.bds.empty();
}

void SPP::adoptProducts(SPP &src) {
    if (&src == this) return;
    ephTable = std::move(src.ephTable);
    mTropoGrid = std::move(src.mTropoGrid);
}

char SPP::clockRefSys(const std::set<char> &active) {
    if (active.count('G')) return 'G';
    for (const char s: {'C', 'R', 'E'})
        if (active.count(s)) return s;
    return 'G';
}

void SPP::solve(ObsData &obsData) {
    result.reset();
    // 本历元两条独立链路，各写各的 result 字段：位置由伪距 IF 定，速度由多普勒定。
    estimatePosition(obsData);
    estimateVelocity(obsData);
}

void SPP::estimatePosition(ObsData &obsData) {
    // LSQ 档的 mFilterInit 每历元被 preprocess 清掉，于是与滤波档首历元走同一个引导入口。
    if (mFilterInit) updateFilter(obsData);
    else initFilter(obsData);
}

void SPP::resetFilterState() {
    ekf.reset();
    mFilterInit = false;
    mHasLastEpoch = false;
}

void SPP::initFilter(ObsData &obsData) {
    // LSQ 档每历元从引导重来，没有跨历元状态可平滑；钟差留随机常数，保持逐历元自由估钟。
    ekf.configure(mConfig.posProcSigma, false, 0.0, mUseFilter ? mConfig.clockProcSigma : 0.0);
    ekf.accProcSigma = mConfig.accProcSigma;
    ekf.velInitSigma = mConfig.velInitSigma;
    ekf.motionDt = 0.0; // 首历元无上一历元可比，不做预测
    // ISB 载体(cdt2~4)的先验/过程噪声口径与 PPP 一致，由 GnssEKF 按参数类型分派。
    ekf.isbProcSigma = mConfig.isbProcSigma;
    ekf.isbInitSigma = mConfig.isbInitSigma;
    runFilter(obsData);
}

void SPP::updateFilter(ObsData &obsData) {
    // 断档过久时速度预测已无意义，退回逐历元随机游走。
    const double gap = mHasLastEpoch ? std::fabs(obsData.epoch - mLastEpoch) : 0.0;
    ekf.motionDt = gap > 0.0 && gap <= kMaxPredictGap ? gap : 0.0;
    runFilter(obsData);
}

bool SPP::wantCommonClock() const {
    if (mConfig.isbInitSigma <= 0.0) return false;
    for (const char sys: activeSystems)
        if (!sysCdtParam.count(sys)) return false;
    return activeSystems.size() >= 2;
}

void SPP::remapClockParams(const char ref) {
    mCdtParam = sysCdtParam;
    if (ref != 'G') {
        mCdtParam[ref] = Parameter::cdt;
        mCdtParam.erase('G');
    }
}

/// 独立钟方程（每系统一个槽位系数 1）→ 公共钟参数化：每颗星挂参考钟 cdt，
/// 非参考系统再挂自己的 ISB 槽；ISB 伪观测整条删除，连续性交给 isbInit/isbProcSigma。
void SPP::toCommonClockEquations(const ObsData &obsData) {
    const Variable vref(obsData.station, Parameter::cdt);
    std::set<char> isbSys;
    for (auto &[eid, ed]: posEquations.obsEquData) {
        const auto it = sysCdtParam.find(eid.sat.system);
        if (it == sysCdtParam.end()) continue;
        const Variable vslot(obsData.station, it->second);
        if (!ed.varCoeffData.count(vslot)) continue;
        ed.varCoeffData.erase(vslot);
        ed.varCoeffData[vref] = 1.0;
        if (eid.sat.system != mClockRefSys) {
            ed.varCoeffData[vslot] = 1.0;
            isbSys.insert(eid.sat.system);
        }
    }
    for (auto it = posEquations.obsEquData.begin(); it != posEquations.obsEquData.end();)
        it = it->first.sat.system == 'I' ? posEquations.obsEquData.erase(it) : std::next(it);
    for (auto it = posEquations.varSet.begin(); it != posEquations.varSet.end();)
        it = it->getParaType().isClock() ? posEquations.varSet.erase(it) : std::next(it);
    posEquations.varSet.insert(vref);
    for (const char sys: isbSys) posEquations.varSet.insert(Variable(obsData.station, mCdtParam.at(sys)));
}

void SPP::bootstrapSolution(ObsData &obsData) {
    lastWStats.clear();
    const bool isbAnyOn = mConfig.isbInitSigma > 0.0;
    if (isbAnyOn) {
        rClockBias = mPrevClockBias;
    } else {
        rClockBias.clear();
    }
    xyz = obsData.antennaPosition;

    detectIFCombinations(obsData);

    // 观测源没给近似坐标
    // 先用上一历元解，首历元再按卫星方向估一个量级正确的初值
    if (xyz.norm() < kMinValidXyz) xyz = mLastXyz;
    if (xyz.norm() < kMinValidXyz) {
        computeSatPos(obsData);
        xyz = roughInitialXyz(obsData);
    }

    int iter(0);
    while (iter < mConfig.maxIter) {
        computeSatPos(obsData);
        earthRotation();

        if (obsData.satTypeValueData.size() < 4) {
            throw SVNumException("num of satellites with valid ephemeris is less than 4 ("
                                 + std::to_string(satPVTRecTime.size()) + ")");
        }
        satElevData.clear();
        satAzimData.clear();
        computeElevAzim();

        linearize(obsData, iter);

        if (posEquations.obsEquData.size() < 4) {
            throw SVNumException("num of satellites after elevation cut is less than 4");
        }
        if (!isRover) break;

        posSolver.solve(posEquations, posEquations.weightVector().asDiagonal().toDenseMatrix());

        const Eigen::Vector3d dxyz = {
            posSolver.getSolution(Parameter::dX),
            posSolver.getSolution(Parameter::dY),
            posSolver.getSolution(Parameter::dZ)
        };
        xyz += dxyz;
        for (char sys: activeSystems) {
            if (auto it = sysCdtParam.find(sys); it != sysCdtParam.end())
                rClockBias[sys] += posSolver.getSolution(it->second);
        }

        if (iter >= mConfig.minCheckIter && dxyz.norm() < mConfig.convEps) {
            break;
        }

        buildResidualMap();
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

void SPP::estimateVelocity(const ObsData &obsData) {
    if (xyz.norm() < kMinValidXyz) return;
    const double a = mConfig.codeSigma / std::sqrt(2.0);
    const Weight w(a, a, mConfig.sinElevFloor());

    std::vector<Velocity::DopplerSat> list;
    list.reserve(satPVTRecTime.size());
    for (const auto &[sat, pvt]: satPVTRecTime) {
        if (satRejected.count(sat)) continue;
        const auto it = obsData.satTypeValueData.find(sat);
        if (it == obsData.satTypeValueData.end()) continue;
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;
        if (!it->second.count(defp->code1) || !it->second.count(defp->code2)) continue;

        double elev = PI * 0.5;
        if (const auto itE = satElevData.find(sat); itE != satElevData.end()) {
            elev = itE->second;
            if (elev < mConfig.cutoffElevRad) continue;
        }

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
        // GLONASS 是 FDMA，频率逐星变
        ds.freqHz = sat.system == 'R' ? getGlonassFreq('1', sat.chan) : getFreq(sat.system, "L1");
        ds.dopplerHz = doppler;
        ds.posWeight = w.weight(elev);
        list.push_back(ds);
    }

    const Velocity::DopplerResult v = Velocity::byDoppler(xyz, list);
    if (!v.ok) return; // 少于 4 颗：速度留 0，交给导出侧的空值逻辑
    result.vel = v.vel;
    result.sigmaVel = v.sigma;
    result.sigmaV = v.sigma.norm();

    // 滤波档：多普勒解作为速度量测进 EKF。只有接了常速度模型（位置-速度耦合）才回写位置，
    // 否则位置解不受速度影响（PPP/LEO 无速度态，posteriorVelocity 返回 false 已提前退出）。
    if (!mUseFilter || !mFilterInit) return;
    ekf.updateVelocity(v.vel, v.sigma);
    Eigen::Vector3d velPost = Eigen::Vector3d::Zero(), sigmaVelPost = Eigen::Vector3d::Zero();
    if (!ekf.posteriorVelocity(obsData.station, velPost, sigmaVelPost)) return;
    result.vel = velPost;
    result.sigmaVel = sigmaVelPost;
    result.sigmaV = sigmaVelPost.norm();
    if (!ekf.velocityCoupled) return;
    xyz = currentEstimatedPosition();
    if (xyz.norm() > kMinValidXyz) mLastXyz = xyz;
    result.xyz = xyz;
    result.xyzFloat = xyz;
    result.blh = Coord::XYZToBLH(xyz, frame);
}

/// 观测源没给近似坐标且还没有上一历元解时，给一个「方位正确、量级正确」的起点。
/// ① 各卫星地心方向单位矢量求和、归一化后乘地球半径。参考：
///    [1] u-blox AG, US 8,125,381 B2, 2002-10-04（Doppler 方案第 3 步）
///    [2] 谢钢. GPS 原理与接收机设计. 第 5 章「首历元初值」法之二
///    与 [1][2] 的差别：此处先单位化再平均，避免北斗 GEO/MEO 混布时高轨星权重偏大。
/// ② 再用 IF 伪距做两轮等权 Gauss-Newton（钟差当未知、不估 ISB）。
Eigen::Vector3d SPP::roughInitialXyz(const ObsData &obsData) const {
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
            const auto it = obsData.satTypeValueData.find(sat);
            if (it == obsData.satTypeValueData.end()) continue;
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

void SPP::runFilter(ObsData &obsData) {
    // 公共钟判定取上一历元的系统集（与滤波器现有状态同一参数化）；boot 路径引导后会重判。
    mCommonClockActive = wantCommonClock();
    if (mCommonClockActive) mClockRefSys = clockRefSys(activeSystems);
    const bool boot = !mFilterInit;
    if (boot) {
        // 引导只要一个收敛的线性化点，不走档位钩子（LSQ 档本身就是这次引导）。
        bootstrapSolution(obsData);
        if (xyz.norm() < kMinValidXyz) return;
        mCommonClockActive = wantCommonClock();
        if (mCommonClockActive) {
            mClockRefSys = clockRefSys(activeSystems);
            remapClockParams(mClockRefSys);
        }
    } else {
        xyz = currentEstimatedPosition();
        lastWStats.clear();
        detectIFCombinations(obsData);
        // 按上一历元的参数化读回系统钟：公共钟 = 参考钟 + ISB，独立钟 = 各自槽位。
        if (mCommonClockActive) {
            const double refClk = ekf.getSolution(Variable(obsData.station, Parameter::cdt));
            for (const auto &[sys, par]: sysCdtParam)
                rClockBias[sys] = sys == mClockRefSys
                                      ? refClk
                                      : refClk + ekf.getSolution(Variable(obsData.station, par));
        } else {
            for (const auto &[sys, par]: sysCdtParam)
                rClockBias[sys] = ekf.getSolution(Variable(obsData.station, par));
        }
    }
    mPrevClockBias = rClockBias;

    computeSatPos(obsData);
    earthRotation();
    if (obsData.satTypeValueData.size() < 4) {
        throw SVNumException("num of satellites with valid ephemeris is less than 4 ("
                             + std::to_string(satPVTRecTime.size()) + ")");
    }
    satElevData.clear();
    satAzimData.clear();
    computeElevAzim();
    linearize(obsData, 0);
    if (posEquations.obsEquData.size() < 4) {
        throw SVNumException("num of satellites after elevation cut is less than 4");
    }

    if (mCommonClockActive) toCommonClockEquations(obsData);
    // 速度三态：常速度预测需要；观测方程里没有它的系数（H 对应列恒 0），只由多普勒量测更新。
    // LSQ 档每历元都从引导重来、不走多普勒量测，速度态只会白占状态维数，不插。
    if (mUseFilter && mConfig.accProcSigma > 0.0) {
        posEquations.varSet.insert(Variable(obsData.station, Parameter::dVX));
        posEquations.varSet.insert(Variable(obsData.station, Parameter::dVY));
        posEquations.varSet.insert(Variable(obsData.station, Parameter::dVZ));
    }
    ekf.timeUpdate(posEquations.varSet, nullptr);
    if (boot) {
        // 状态存绝对 ECEF 坐标与绝对钟差，与 PPP 同口径。
        ekf.setBootstrapPosition(xyz, Eigen::Vector3d(10.0 * 10.0, 10.0 * 10.0, 10.0 * 10.0)); // σ=10 m
        std::map<Variable, double> clk;
        if (mCommonClockActive) {
            // 公共钟参数化：参考系统存绝对钟差，其余槽位存相对参考系统的 ISB。
            const double refClk = rClockBias.count(mClockRefSys) ? rClockBias.at(mClockRefSys) : 0.0;
            for (const auto &[sys, par]: mCdtParam)
                if (activeSystems.count(sys))
                    clk[Variable(obsData.station, par)] =
                            sys == mClockRefSys ? refClk : rClockBias[sys] - refClk;
        } else {
            for (const char sys: activeSystems) {
                if (const auto it = sysCdtParam.find(sys); it != sysCdtParam.end())
                    clk[Variable(obsData.station, it->second)] = rClockBias[sys];
            }
        }
        ekf.setBootstrapStates(clk);
    }
    ekf.measUpdate(posEquations);
    xyz = currentEstimatedPosition();
    if (xyz.norm() > kMinValidXyz) mLastXyz = xyz;
    mFilterInit = true;
    readbackFiltered(obsData);
    mLastEpoch = obsData.epoch;
    mHasLastEpoch = true;
}

Eigen::Vector3d SPP::currentEstimatedPosition() const {
    return {
        GnssEKF::getSolution(Parameter::dX, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dY, ekf.currentUnkSet, ekf.solution),
        GnssEKF::getSolution(Parameter::dZ, ekf.currentUnkSet, ekf.solution)
    };
}

void SPP::readbackFiltered([[maybe_unused]] const ObsData &obsData) {
    const Eigen::Matrix3d Q = SPP::posCovariance();
    // 速度三态不进位置观测方程，算自由度时要扣掉，否则 sigma0 被系统性低估。
    const int velStates = mUseFilter && mConfig.accProcSigma > 0.0 ? 3 : 0;
    sigma0 = posteriorSigma0(ekf.postfitResidual, posEquations.weightVector(),
                             static_cast<int>(ekf.currentUnkSet.size()) - velStates);
    result.blh = Coord::XYZToBLH(xyz, frame);
    geometryDOPs(result, posEquations, result.blh);
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
    for (const auto &[id, data]: posEquations.obsEquData) {
        if (iobs >= ekf.postfitResidual.size()) break;
        result.postRes.set(id.sat, ekf.postfitResidual(iobs++));
    }
}

Eigen::Matrix3d SPP::positionCovBlock(const Eigen::MatrixXd &P, const int id[3]) {
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * 1.0e12;
    if (id[0] < 0 || id[1] < 0 || id[2] < 0 || P.rows() <= id[2]) return Q;
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b) Q(a, b) = P(id[a], id[b]);
    return Q;
}

Eigen::Matrix3d SPP::posCovariance() const {
    // 有 EKF 后验就用它（LSQ 档每历元引导后同样有），否则回落到 LSQ 协因数阵。
    if (ekf.covMatrix.rows() > 0 && !ekf.currentUnkSet.empty()) {
        int id[3] = {-1, -1, -1};
        int i = 0;
        for (const auto &v: ekf.currentUnkSet) {
            if (const auto pt = v.getParaType(); pt == Parameter::dX) id[0] = i;
            else if (pt == Parameter::dY) id[1] = i;
            else if (pt == Parameter::dZ) id[2] = i;
            ++i;
        }
        if (id[0] >= 0 && id[1] >= 0 && id[2] >= 0)
            return positionCovBlock(ekf.covMatrix, id);
    }
    if (posSolver.covMatrix.rows() < 3) return Eigen::Matrix3d::Identity() * 1.0e12;
    const double s2 = posSolver.sigma0 > 0.0 ? posSolver.sigma0 * posSolver.sigma0 : 1.0;
    return posSolver.covMatrix.topLeftCorner(3, 3) * s2;
}

void SPP::buildResidualMap() {
    lastWStats.clear();

    const auto &Ninv = posSolver.covMatrix; // N⁻¹ (order 5×5)
    const double s0 = posSolver.sigma0;
    if (s0 <= 0.0) return;
    int i = 0;
    for (const auto &[eid, data]: posEquations.obsEquData) {
        if (i >= posSolver.v.size()) break;
        const double v = posSolver.v[i];
        const double w = data.weight;
        const double invW = w > 0.0 ? 1.0 / w : 0.0;
        const auto hRow = posSolver.hMatrix.row(i);
        if (const double qvv = invW - (hRow * Ninv * hRow.transpose())(0, 0); qvv > 0.0) {
            const double wStat = std::abs(v) / (s0 * std::sqrt(qvv));
            lastWStats[eid.sat] = wStat;
        }

        ++i;
    }
}

void SPP::updateTropoSite(const Eigen::Vector3d &xyzEcef, const CommonTime &epoch) {
    mTropoSite.update(mTropoGrid, mConfig.tropoMap, xyzEcef, epoch, frame);
    if (!(mTropoSite.hgt > 0.0)) mTropoSite = TropoSite{};
}

double SPP::tropoDelay(const double elevRad) const {
    if (!mTropoSite.valid) return 0.0;
    double mW = 0.0;
    const double dry = mTropoSite.dryDelay(mConfig.tropoMap, elevRad, &mW);
    const double tropo = dry + TropoSite::wetDelay(mTropoSite.zwdResolved(), mW);
    if (!std::isfinite(tropo) || tropo < 0.0 || tropo > 100.0) return 0.0;
    return tropo;
}

void SPP::linearize(ObsData &obsData, const int iter) {
    SatID outlierSat;
    double worstW = 0.0;
    if (iter >= 2) {
        for (auto const &[sat, codeList]: obsData.satTypeValueData) {
            if (satRejected.count(sat)) continue;
            if (auto itW = lastWStats.find(sat);
                itW != lastWStats.end() && itW->second > worstW) {
                worstW = itW->second;
                outlierSat = sat;
            }
        }
    }
    const bool rejectOutlier = worstW > W_THRESHOLD;

    if (!mWeightInit) {
        const double a = mConfig.codeSigma / std::sqrt(2.0);
        mWeight = Weight(a, a, mConfig.sinElevFloor());
        mWeightInit = true;
    }

    posEquations.reset();
    posEquations.station = obsData.station;

    const Variable dx(obsData.station, Parameter::dX);
    const Variable dy(obsData.station, Parameter::dY);
    const Variable dz(obsData.station, Parameter::dZ);
    std::map<char, Variable> cdtVars;
    for (auto &[sys, param]: sysCdtParam)
        cdtVars.try_emplace(sys, obsData.station, param);

    updateTropoSite(xyz, obsData.epoch);

    activeSystems.clear();
    int nRejPVT = 0, nRejElev = 0, nRejIF = 0, nPass = 0;

    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;

        if (satRejected.count(sat)) {
            continue;
        }

        if (!satPVTRecTime.count(sat)) {
            satRejected.insert(sat);
            nRejPVT++;
            continue;
        }
        const auto &pvt = satPVTRecTime.at(sat);

        // Baarda w 检验
        if (rejectOutlier && sat == outlierSat) {
            satRejected.insert(sat);
            continue;
        }

        // 高度角
        double elev = PI * 0.5;
        if (auto itE = satElevData.find(sat); itE != satElevData.end())
            elev = itE->second;
        if (xyz.norm() > 1000.0 && elev < mConfig.cutoffElevRad) {
            satRejected.insert(sat);
            nRejElev++;
            continue;
        }

        const FreqCombo &def = *defp;
        if (!codeList.count(def.code1) || !codeList.count(def.code2)) {
            satRejected.insert(sat);
            nRejIF++;
            continue;
        }
        const double obsVal = def.combineCodeFromObs(codeList);

        // ---- 共享几何量 ----
        double rho = (pvt.p - xyz).norm();
        if (rho < 1.0) rho = 20000000.0;

        const Eigen::Vector3d los = {
            -(pvt.p[0] - xyz[0]) / rho,
            -(pvt.p[1] - xyz[1]) / rho,
            -(pvt.p[2] - xyz[2]) / rho
        };

        double trop = 0.0;
        if (mTropoSite.valid)
            trop = tropoDelay(elev);

        double weight = mWeight.weight(elev);

        // ---- 逐卫星构建方程 ----
        buildPosEquation(sat, pvt, los, rho, trop,
                         def.code1 + "+" + def.code2, obsVal, weight, dx, dy, dz, cdtVars[sat.system]);

        activeSystems.insert(sat.system);
        nPass++;
    }
    posEquations.varSet.insert(dx);
    posEquations.varSet.insert(dy);
    posEquations.varSet.insert(dz);
    for (char sys: activeSystems) {
        if (auto it = cdtVars.find(sys); it != cdtVars.end())
            posEquations.varSet.insert(it->second);
    }
    if (mConfig.isbInitSigma > 0.0 && activeSystems.size() >= 2) {
        const char ref = clockRefSys(activeSystems);
        for (char sys: activeSystems) {
            if (sys == ref) continue;
            if (!mPrevClockBias.count(sys) || !mPrevClockBias.count(ref)) continue;
            const double prevIsb = mPrevClockBias.at(sys) - mPrevClockBias.at(ref);
            const Variable &vclk = cdtVars.at(sys);
            const Variable &vref = cdtVars.at(ref);
            EquData ed;
            ed.prefit = prevIsb - (rClockBias[sys] - rClockBias[ref]);
            ed.varCoeffData[vclk] = 1.0;
            ed.varCoeffData[vref] = -1.0;
            const double var = mConfig.isbInitSigma * mConfig.isbInitSigma
                               + mConfig.isbProcSigma * mConfig.isbProcSigma;
            ed.weight = 1.0 / var;
            posEquations.obsEquData[EquID(SatID('I', 0), "ISB_" + std::string(1, sys))] = ed;
        }
    }
}


void SPP::buildPosEquation(const SatID &sat, const PVT &pvt, const Eigen::Vector3d &los,
                           const double rho, const double trop,
                           const std::string &obsType, const double obsVal, const double weight,
                           const Variable &dx, const Variable &dy, const Variable &dz,
                           const Variable &cdtVar) {
    const double dts = pvt.clockBias * C_MPS;
    const double rel = pvt.relativityCorrection * C_MPS;

    const EquID eid(sat, obsType);
    EquData ed;
    ed.prefit = obsVal - (rho + rClockBias[sat.system] - dts - rel + trop);

    ed.varCoeffData[dx] = los[0];
    ed.varCoeffData[dy] = los[1];
    ed.varCoeffData[dz] = los[2];
    ed.varCoeffData[cdtVar] = 1.0;

    ed.weight = weight;
    posEquations.obsEquData[eid] = ed;
}

bool SPP::ephUnhealthy(const SatID &sat, const Ephemeris *eph) {
    if (!mConfig.useHealth || !eph || eph->health == 0u) return false;
    if (mHealthWarned.insert(sat).second)
        LOG_WARN << "[健康] 剔除广播电文标记不可用的卫星 " << sat << " (health=" << eph->health << ")";
    return true;
}

void SPP::computeSatPos(ObsData &obsData) {
    satPVTTransTime.clear();
    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        const FreqCombo *defp = freqComboFor(sat);
        if (!defp) continue;

        Ephemeris *eph = ephTable.find(sat, obsData.epoch);
        if (!eph) continue;
        if (ephUnhealthy(sat, eph)) continue; // 广播电文自报不可用

        const FreqCombo &def = *defp;
        if (!codeList.count(def.code1) || !codeList.count(def.code2)) continue;
        const double obsVal = def.combineCodeFromObs(codeList);

        const double tau_total = (obsVal - rClockBias[sat.system]) / C_MPS;
        CommonTime t_emit = obsData.epoch;
        t_emit.m_sod -= tau_total;

        satPVTTransTime[sat] = eph->getPVT(t_emit, &def);
    }
}

void SPP::detectIFCombinations(const ObsData &obsData, const IFCodeTypes &defaultTypes) {
    std::map<char, std::string> sig;
    for (const auto &[sat, tv]: obsData.satTypeValueData) {
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
    for (const auto &[sat, tv]: obsData.satTypeValueData) sysList.insert(sat.system);

    IFCodeTypes detected;
    mGloCombo.clear();
    for (char sys: sysList) {
        if (!mConfig.enabledSystems.empty() && !mConfig.enabledSystems.count(sys)) continue;
        if (sys == 'R') {
            for (const auto &[sat, tv]: obsData.satTypeValueData) {
                if (sat.system != 'R') continue;
                if (FreqCombo def = FreqCombo::detect('R', tv, sat.chan, mRequirePhaseForIF); !def.code1.empty() && !def.code2.empty())
                    mGloCombo[sat] = def;
            }
        } else if (FreqCombo def = FreqCombo::detect(sys, obsData, mRequirePhaseForIF);
            !def.code1.empty() && !def.code2.empty()) {
            detected[sys] = def;
        }
    }
    if (detected.empty() && mGloCombo.empty()) {
        LOG_WARN << (defaultTypes.empty() ? "未检测到可用的 IF 组合" : "未检测到可用的 IF 组合，回退默认");
        ifCodeTypes = defaultTypes;
        return;
    }
    ifCodeTypes = std::move(detected);
}

void SPP::earthRotation() {
    satPVTRecTime.clear();
    for (auto const &[sat, pvt]: satPVTTransTime) {
        satPVTRecTime[sat] = Coord::earthRotationCorrected(pvt, xyz);
    }
}

void SPP::computeElevAzim() {
    for (auto const &[sat, pvt]: satPVTRecTime) {
        const double e = Coord::elevation(xyz, pvt.p, frame);
        const double a = Coord::azimuth(xyz, pvt.p, frame);
        satElevData[sat] = e;
        satAzimData[sat] = a;
    }
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
