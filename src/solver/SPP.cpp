#include "SPP.h"
#include "rinex/RINEXNavStore.h"
#include "CoordConvert.h"
#include "FreqCombo.h"
#include <Eigen/Eigen>
#include <array>
#include <vector>
#include <algorithm>
#include <set>

#include "Log.h"
#include "CoordStruct.h"
#include "Weight.h"

namespace {
    constexpr double kMinValidXyz = 6.0e6;
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
    satRejected.clear();
}

bool SPP::loadBRDC(const std::string &path) {
    RINEXNavStore store;
    store.loadFile(path, ephTable);
    return !ephTable.gps.empty() || !ephTable.bds.empty();
}

void SPP::solve(ObsData &obsData) {
    result.reset();
    lastWStats.clear();
    const bool isbAnyOn = mConfig.isbInitVar > 0.0;
    if (isbAnyOn) {
        rClockBias = mPrevClockBias;
    } else {
        rClockBias.clear();
    }
    vel = Eigen::Vector3d(0, 0, 0);
    rClockDrift = 0.0;
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

        posSolver.solve(posEquations);

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

        if (velEquations.obsEquData.size() >= 4) {
            velSolver.solve(velEquations);
            const Eigen::Vector3d dvel = {
                velSolver.getSolution(Parameter::dVX),
                velSolver.getSolution(Parameter::dVY),
                velSolver.getSolution(Parameter::dVZ)
            };
            const double dcdt_dot = velSolver.getSolution(Parameter::cdtr_dot);
            vel += dvel;
            rClockDrift += dcdt_dot;
        } else {
            LOG_WARN << "速度解算失败";
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
    readback(obsData);
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

void SPP::readback([[maybe_unused]] const ObsData &obsData) {
    auto &pD = posSolver.covMatrix;
    auto &vD = velSolver.covMatrix;
    double clockVar = 0.0;
    int idx = 0;
    for (const auto &v: posSolver.currentUnkSet) {
        if (const auto p = v.getParaType(); p == Parameter::cdt || p == Parameter::cdt2) {
            clockVar += pD(idx, idx);
        }
        idx++;
    }
    result.blh = Coord::XYZToBLH(xyz, frame);
    computePositionDops(result, pD.topLeftCorner(3, 3), clockVar, result.blh);

    result.sigmaP = result.pdop * posSolver.sigma0;
    result.sigmaXYZ = {
        std::sqrt(pD(0, 0)) * posSolver.sigma0,
        std::sqrt(pD(1, 1)) * posSolver.sigma0,
        std::sqrt(pD(2, 2)) * posSolver.sigma0
    };
    if (vD.rows() >= 3) {
        result.sigmaV = std::sqrt(vD(0, 0) + vD(1, 1) + vD(2, 2)) * velSolver.sigma0;
        result.sigmaVel = {
            std::sqrt(vD(0, 0)) * velSolver.sigma0,
            std::sqrt(vD(1, 1)) * velSolver.sigma0,
            std::sqrt(vD(2, 2)) * velSolver.sigma0
        };
    } else {
        result.sigmaV = 0.0;
        result.sigmaVel = {0.0, 0.0, 0.0};
    }
    result.xyz = xyz;
    result.vel = vel;

    result.numSats = static_cast<int>(satPVTRecTime.size());

    int iobs = 0;
    for (const auto &[id, data]: posEquations.obsEquData) {
        result.postRes[id.sat] = posSolver.v[iobs++];
    }
}

void SPP::buildResidualMap() {
    lastWStats.clear();

    const auto &Ninv = posSolver.covMatrix; // N⁻¹ (order 5×5)
    const double sigma0 = posSolver.sigma0;
    if (sigma0 <= 0.0) return;
    int i = 0;
    for (const auto &[eid, data]: posEquations.obsEquData) {
        if (i >= posSolver.v.size()) break;
        const double v = posSolver.v[i];
        const double w = data.weight;
        const double invW = w > 0.0 ? 1.0 / w : 0.0;
        const auto hRow = posSolver.hMatrix.row(i);
        if (const double qvv = invW - (hRow * Ninv * hRow.transpose())(0, 0); qvv > 0.0) {
            const double wStat = std::abs(v) / (sigma0 * std::sqrt(qvv));
            lastWStats[eid.sat] = wStat;
        }

        ++i;
    }
}

void SPP::updateTropoSite(const Eigen::Vector3d &xyzEcef, const CommonTime &epoch) {
    mTropoSite.update(mTropoGrid, mConfig.tropoMap, xyzEcef, epoch, nullptr, frame);
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
        const double a = mConfig.sigIFCode / std::sqrt(2.0);
        mWeight = Weight(a, a, mConfig.sinElevFloor());
        mWeightInit = true;
    }

    posEquations.reset();
    velEquations.reset();
    posEquations.station = obsData.station;
    velEquations.station = obsData.station;

    const Variable dx(obsData.station, Parameter::dX);
    const Variable dy(obsData.station, Parameter::dY);
    const Variable dz(obsData.station, Parameter::dZ);
    std::map<char, Variable> cdtVars;
    for (auto &[sys, param]: sysCdtParam)
        cdtVars.try_emplace(sys, obsData.station, param);
    const Variable dvx(obsData.station, Parameter::dVX);
    const Variable dvy(obsData.station, Parameter::dVY);
    const Variable dvz(obsData.station, Parameter::dVZ);
    const Variable dcdt(obsData.station, Parameter::cdtr_dot);

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
        // 速度方程与位置方程共用同一个 LOS（接收机→卫星方向）
        buildVelEquation(sat, codeList, pvt, los, weight, dvx, dvy, dvz, dcdt);

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
    if (mConfig.isbInitVar > 0.0 && activeSystems.count('G')) {
        for (char sys: activeSystems) {
            if (sysCdtParam.at(sys) == Parameter::cdt) continue;
            if (!mPrevClockBias.count(sys) || !mPrevClockBias.count('G')) continue;
            const double prevIsb = mPrevClockBias.at(sys) - mPrevClockBias.at('G');
            const Variable &vclk = cdtVars.at(sys);
            const Variable &vref = cdtVars.at('G');
            EquData ed;
            ed.prefit = prevIsb - (rClockBias[sys] - rClockBias['G']);
            ed.varCoeffData[vclk] = 1.0;
            ed.varCoeffData[vref] = -1.0;
            const double var = mConfig.isbInitVar + mConfig.isbProcNoise;
            ed.weight = 1.0 / var;
            posEquations.obsEquData[EquID(SatID('I', 0), "ISB_" + std::string(1, sys))] = ed;
        }
    }
    velEquations.varSet.insert(dvx);
    velEquations.varSet.insert(dvy);
    velEquations.varSet.insert(dvz);
    velEquations.varSet.insert(dcdt);
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

void SPP::buildVelEquation(const SatID &sat, const TypeValueMap &codeList,
                           const PVT &pvt, const Eigen::Vector3d &los,
                           const double posWeight,
                           const Variable &dvx, const Variable &dvy,
                           const Variable &dvz, const Variable &dcdt) {
    double dopplerVal = 0.0;
    bool found = false;
    for (const auto &[k, v]: codeList) {
        if (k.size() >= 2 && k[0] == 'D' && k[1] == '1') {
            dopplerVal = v;
            found = true;
            break;
        }
    }
    if (!found) {
        // LOG_WARN << "No doppler code found for " << sat;
        return;
    }

    // GLONASS 频率随通道号变化（FDMA），必须用逐星频率
    const double f = sat.system == 'R' ? getGlonassFreq('1', sat.chan) : getFreq(sat.system, "L1");
    if (f <= 0.0) return;
    const double lambda = C_MPS / f;
    const double rho_dot_obs = -lambda * dopplerVal;

    const double rho_dot_model = (vel - pvt.v).dot(los) + rClockDrift;

    EquData ed;
    ed.prefit = rho_dot_obs - rho_dot_model;

    ed.varCoeffData[dvx] = los[0];
    ed.varCoeffData[dvy] = los[1];
    ed.varCoeffData[dvz] = los[2];
    ed.varCoeffData[dcdt] = 1.0;

    const double velWeight = posWeight / 40.0;
    ed.weight = velWeight;

    const EquID eidVel(sat, "D1");
    velEquations.obsEquData[eidVel] = ed;
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

        satPVTTransTime[sat] = eph->getPVT(t_emit);
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
        PVT rotPvt = pvt;
        applyEarthRotation(rotPvt.p, rotPvt.v, xyz);
        satPVTRecTime[sat] = rotPvt;
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
    mConfig.sigIFCode = o.spp.sigIFCode;
    mConfig.minSinElev = o.spp.minSinElev;
    mConfig.tropoMap = o.spp.tropoMap;
    mConfig.convEps = o.spp.convEps;
    mConfig.maxIter = o.spp.maxIter;
    mConfig.minCheckIter = o.spp.minCheckIter;
    mConfig.useHealth = o.spp.useHealth; // 广播电文健康标志剔除
    setISBProcNoise(o.spp.isbProcNoise, o.spp.isbInitVar);
    mWeightInit = false;
}
