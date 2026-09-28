#include "SPP.h"
#include "RinexNavStore.h"
#include "CoordConvert.h"
#include "FreqCombo.h"
#include <Eigen/Eigen>
#include <vector>
#include <algorithm>
#include <set>

#include "Log.h"
#include "MathUtils.h"
#include "CoordStruct.h"
#include "Troposphere.h"
#include "Weight.h"
#include <cmath>

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
    setEphemeris(obsData.satEphemerisData);
}

bool SPP::loadBrdc(const std::string &path) {
    RinexNavStore store;
    store.loadFile(path, ephTable);
    return !ephTable.gps.empty() || !ephTable.bds.empty();
}

void SPP::solve(ObsData &obsData) {
    result.reset();
    lastWStats.clear();
    rClockBias.clear();
    vel = Vector3d(0, 0, 0);
    rClockDrift = 0.0;
    xyz = obsData.antennaPosition;

    // 每历元就地探测 IF 组合（SPP/PPP/LEO 统一入口：收集各系统码 → 统一探测）
    detectIFCombinations(obsData);

    int iter(0);
    while (iter < mConfig.maxIter) {
        computeSatPos(obsData);
        earthRotation();

        if (obsData.satTypeValueData.size() < 4) {
            throw SVNumException("num of satellites with valid ephemeris is less than 4 ("
                                 + to_string(satPVTRecTime.size()) + ")");
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

        const Vector3d dxyz = {
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
            const Vector3d dvel = {
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

        // 收敛判据：位置修正量 < 0.1mm 即停
        if (iter >= mConfig.minCheckIter && dxyz.norm() < mConfig.convEps) {
            break;
        }

        buildResidualMap();
        iter++;
    }

    if (iter >= 15) {
        throw InvalidSolver("too many iterations without convergence");
    }
    readback(obsData);
}

void SPP::readback(const ObsData &/*obsData*/) {
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
    result.blh = XYZtoBLH(xyz, frame);
    computePositionDops(result, pD.topLeftCorner(3, 3), clockVar, result.blh);

    result.sigmaP = result.pdop * posSolver.sigma0;
    result.sigmaXYZ = {
        sqrt(pD(0, 0)) * posSolver.sigma0,
        sqrt(pD(1, 1)) * posSolver.sigma0,
        sqrt(pD(2, 2)) * posSolver.sigma0
    };
    if (vD.rows() >= 3) {
        result.sigmaV = sqrt(vD(0, 0) + vD(1, 1) + vD(2, 2)) * velSolver.sigma0;
        result.sigmaVel = {
            sqrt(vD(0, 0)) * velSolver.sigma0,
            sqrt(vD(1, 1)) * velSolver.sigma0,
            sqrt(vD(2, 2)) * velSolver.sigma0
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

    // 高度角随机模型：a²+b²/sin²E，与 RTK/PPP 共用 Weight 类。
    // a=b=sigIFCode/√2 使高仰角权 ≈ 1/sigIFCode²；首轮解算惰性初始化一次，迭代内复用。
    if (!mWeightInit) {
        mWeight = Weight(mConfig.sigIFCode / std::sqrt(2.0), mConfig.sigIFCode / std::sqrt(2.0),
                         std::sin(mConfig.cutoffElevRad));
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

    double refHgt = 0.0;
    if (xyz.norm() > 1000.0) {
        const auto BLH = XYZtoBLH(xyz, frame);
        refHgt = BLH[2];
    }

    activeSystems.clear();
    int nRejPVT = 0, nRejElev = 0, nRejIF = 0, nPass = 0;

    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        if (!ifCodeTypes.count(sat.system)) continue;

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

        const FreqCombo &def = ifCodeTypes.at(sat.system);
        if (!codeList.count(def.code1) || !codeList.count(def.code2)) {
            satRejected.insert(sat);
            nRejIF++;
            continue;
        }
        const double obsVal = def.combineCodeFromObs(codeList);

        // ---- 共享几何量 ----
        double rho = (pvt.p - xyz).norm();
        if (rho < 1.0) rho = 20000000.0;

        const Vector3d los = {
            -(pvt.p[0] - xyz[0]) / rho,
            -(pvt.p[1] - xyz[1]) / rho,
            -(pvt.p[2] - xyz[2]) / rho
        };

        double trop = 0.0;
        if (refHgt > 0.0)
            trop = Troposphere::tropoHopfield(refHgt, elev);

        // 位置方程权：高度角随机模型（Weight 类）
        double weight = mWeight.weight(elev);

        // ---- 逐卫星构建方程 ----
        buildPosEquation(sat, pvt, los, rho, trop,
                         def.code1 + "+" + def.code2, obsVal, weight, dx, dy, dz, cdtVars[sat.system]);
        // 速度方程与位置方程共用同一个 LOS（接收机→卫星方向）
        buildVelEquation(sat, codeList, pvt, los, elev, weight,
                         dvx, dvy, dvz, dcdt);

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
    velEquations.varSet.insert(dvx);
    velEquations.varSet.insert(dvy);
    velEquations.varSet.insert(dvz);
    velEquations.varSet.insert(dcdt);
}


void SPP::buildPosEquation(const SatID &sat, const PVT &pvt, const Vector3d &los,
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
                           const PVT &pvt, const Vector3d &los,
                           const double elev, const double posWeight,
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

    const double f = getFreq(sat.system, "L1");
    const double lambda = C_MPS / f;
    const double rho_dot_obs = -lambda * dopplerVal;

    const double rho_dot_model = (vel - pvt.v).dot(los) + rClockDrift;

    EquData ed;
    ed.prefit = rho_dot_obs - rho_dot_model;

    ed.varCoeffData[dvx] = los[0];
    ed.varCoeffData[dvy] = los[1];
    ed.varCoeffData[dvz] = los[2];
    ed.varCoeffData[dcdt] = 1.0;

    double velWeight = mWeight.weight(elev) / 40.0; // 速度观测噪声基准为位置观测的 √40 倍
    ed.weight = velWeight;

    const EquID eidVel(sat, "D1");
    velEquations.obsEquData[eidVel] = ed;
}

void SPP::computeSatPos(ObsData &obsData) {
    satPVTTransTime.clear();
    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        if (!ifCodeTypes.count(sat.system)) continue;

        Ephemeris *eph = ephTable.find(sat, obsData.epoch);
        if (!eph) {
            auto itEph = ephMap.find(sat);
            if (itEph == ephMap.end()) continue;
            eph = itEph->second;
        }

        const FreqCombo &def = ifCodeTypes.at(sat.system);
        if (!codeList.count(def.code1) || !codeList.count(def.code2)) continue;
        const double obsVal = def.combineCodeFromObs(codeList);

        const double tau_total = (obsVal - rClockBias[sat.system]) / C_MPS;
        CommonTime t_emit = obsData.epoch;
        t_emit.m_sod -= tau_total;

        satPVTTransTime[sat] = eph->getPVT(std::move(t_emit));
    }
}

void SPP::detectIFCombinations(const ObsData &obsData, const IFCodeTypes &defaultTypes) {
    // 逐历元重扫全部观测类型既浪费、又会刷屏日志，故用"各系统可用且非零的码类型"做签名缓存：
    // 签名不变 ⇒ 码类型集合没变，检测结果必然相同，直接沿用。
    // （原先每历元无条件打 "Auto-detected IF types"，整日运行上万条，属纯噪声，已不再记录。）
    std::map<char, std::string> sig;
    for (const auto &[sat, tv]: obsData.satTypeValueData) {
        if (!mConfig.enabledSystems.empty() && !mConfig.enabledSystems.count(sat.system)) continue;
        std::string &s = sig[sat.system];
        for (const auto &[code, v]: tv) {
            if (v == 0.0) continue; // 缺测/占位零值不算可用类型，否则签名会因单历元缺测抖动
            s += code;
            s.push_back(',');
        }
    }
    if (!ifCodeTypes.empty() && sig == ifCodeSig) return;
    ifCodeSig = std::move(sig);

    std::set<char> sysList;
    for (const auto &[sat, tv]: obsData.satTypeValueData) sysList.insert(sat.system);

    IFCodeTypes detected;
    for (char sys: sysList) {
        if (!mConfig.enabledSystems.empty() && !mConfig.enabledSystems.count(sys)) continue;
        if (FreqCombo def = FreqCombo::detect(sys, obsData, mRequirePhaseForIF);
            !def.code1.empty() && !def.code2.empty())
            detected[sys] = def;
    }
    if (detected.empty()) {
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
        double e, a;
        satElevAzim(xyz, pvt.p, e, a, frame);
        satElevData[sat] = e;
        satAzimData[sat] = a;
    }
}

bool SPP::estimateApproxPosition(const std::vector<ObsData> &epochs,
                                 const std::vector<EphemerisTable> &ephs,
                                 const std::set<char> &systems,
                                 const double cutoffElevDeg, Vector3d &out) {
    const size_t n = std::min(epochs.size(), ephs.size()); // 防止 ephs 较短时越界
    for (size_t i = 0; i < n; ++i) {
        if (ephs[i].gps.empty() && ephs[i].bds.empty()) continue; // 星历快照仍空
        try {
            SPP spp;
            spp.ephTable = ephs[i];
            spp.setEnabledSystems(systems);
            spp.setCutoffElevDeg(cutoffElevDeg);
            ObsData work = epochs[i];
            work.antennaPosition = Vector3d::Zero(); // 线性化初值：地心冷启动
            if (spp.processEpoch(work)) out = spp.result.xyz;
            if (out.norm() > 1e6) return true;
        } catch (...) { /* 该历元不可用，试下一个 */ }
    }
    return false;
}
