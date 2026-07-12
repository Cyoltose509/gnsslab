#include "SPPIFCode.h"
#include "CoordConvert.h"
#include <Eigen/Eigen>
#include <vector>
#include <algorithm>

#include "Log.h"

void SPPIFCode::preprocess(ObsData &obsData) {
    satRejected.clear();
    setEphemeris(obsData.satEphemerisData);
    computeIF(obsData);
}

void SPPIFCode::solve(ObsData &obsData) {
    result.reset();
    lastWStats_.clear();
    rClockBias.clear();
    vel = Vector3d(0, 0, 0);
    rClockDrift = 0.0;
    xyz = obsData.antennaPosition;

    int iter(0);
    while (iter < 10) {
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
        for (char sys : activeSystems) {
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
        if (iter >= 2 && dxyz.norm() < 1e-4) {
            break;
        }

        buildResidualMap();
        iter++;
    }

    if (iter >= 15) {
        throw InvalidSolver("too many iterations without convergence");
    }
    getResult();
}

void SPPIFCode::getResult() {
    auto &pD = posSolver.covMatrix;
    auto &vD = velSolver.covMatrix;
    result.pdop = sqrt(pD(0, 0) + pD(1, 1) + pD(2, 2));
    {
        double gdop_sq = result.pdop * result.pdop;  // PDOP²
        double tdop_sq = 0.0;
        int idx = 0;
        for (const auto &v : posSolver.currentUnkSet) {
            if (const auto p = v.getParaType(); p == Parameter::cdt || p == Parameter::cdt2) {
                tdop_sq += pD(idx, idx);
                gdop_sq += pD(idx, idx);
            }
            idx++;
        }
        result.gdop = sqrt(gdop_sq);
        result.tdop = sqrt(tdop_sq);
    }

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
    }
    else {
        result.sigmaV = 0.0;
        result.sigmaVel = {0.0, 0.0, 0.0};
    }
    result.xyz = xyz;
    result.vel = vel;
    result.blh = XYZtoBLH(xyz, frame);
    auto mt = getBLMatrix(result.blh[0], result.blh[1]);
    Matrix3d C_enu = mt * pD.topLeftCorner(3, 3) * mt.transpose();
    result.hdop = sqrt(C_enu(0, 0) + C_enu(1, 1));
    result.vdop = sqrt(C_enu(2, 2));

    result.numSats = static_cast<int>(satPVTRecTime.size());

    int iobs = 0;
    for (const auto &[id, data]: posEquations.obsEquData) {
        result.postRes[id.sat] = posSolver.v[iobs++];
    }
}

// ============================================================================
// buildResidualMap — 解算完成后计算 Baarda w 统计量
//   w_i = |v_i| / (σ₀ · √q_vv_i)
//   q_vv_i = 1/w_i - h_i · N⁻¹ · h_iᵀ
// ============================================================================
void SPPIFCode::buildResidualMap() {
    lastWStats_.clear();

    const auto &Ninv = posSolver.covMatrix; // N⁻¹ (order 5×5)
    const double sigma0 = posSolver.sigma0;
    if (sigma0 <= 0.0) return;

    int i = 0;
    for (const auto &[eid, data]: posEquations.obsEquData) {
        if (i >= posSolver.v.size()) break;

        const double v = posSolver.v[i];
        const double w = data.weight;
        const double invW = w > 0.0 ? 1.0 / w : 0.0;

        // 设计矩阵第 i 行 (1 × numUnk)
        const auto hRow = posSolver.hMatrix.row(i);

        // q_vv = P⁻¹_ii - h_i · N⁻¹ · h_iᵀ

        if (const double qvv = invW - (hRow * Ninv * hRow.transpose())(0, 0); qvv > 0.0) {
            const double wStat = std::abs(v) / (sigma0 * std::sqrt(qvv));
            lastWStats_[eid.sat] = wStat;
        }

        ++i;
    }
}

void SPPIFCode::linearize(ObsData &obsData, const int iter) {

    // ---- 先找本轮最可疑的卫星（w 统计量最大），只剔除一颗 ----
    SatID outlierSat;
    double worstW = 0.0;
    if (iter >= 2) {
        for (auto const &[sat, codeList]: obsData.satTypeValueData) {
            if (satRejected.count(sat)) continue;
            if (auto itW = lastWStats_.find(sat);
                itW != lastWStats_.end() && itW->second > worstW) {
                worstW = itW->second;
                outlierSat = sat;
            }
        }
    }
    const bool rejectOutlier = worstW > W_THRESHOLD;

    posEquations.reset();
    velEquations.reset();
    posEquations.station = obsData.station;
    velEquations.station = obsData.station;

    // ---- 未知参数（只构造一次，所有卫星共用） ----
    const Variable dx(obsData.station, Parameter::dX);
    const Variable dy(obsData.station, Parameter::dY);
    const Variable dz(obsData.station, Parameter::dZ);
    // 各系统钟差 Variable，按需构造
    std::map<char, Variable> cdtVars;
    for (auto &[sys, param] : sysCdtParam)
        cdtVars.try_emplace(sys, obsData.station, param);
    const Variable dvx(obsData.station, Parameter::dVX);
    const Variable dvy(obsData.station, Parameter::dVY);
    const Variable dvz(obsData.station, Parameter::dVZ);
    const Variable dcdt(obsData.station, Parameter::cdtr_dot);

    // ---- BLH 提到循环外 ----
    double refHgt = 0.0;
    if (xyz.norm() > 1000.0) {
        const auto BLH = XYZtoBLH(xyz, frame);
        refHgt = BLH[2];
    }

    activeSystems.clear();
    int nRejPVT = 0, nRejElev = 0, nRejIF = 0, nPass = 0;
    static int dbgCnt = 0;  // 仅前几个历元输出拒绝原因

    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        // 只处理配置了 IF 类型的系统（GPS / BDS）
        if (!ifTypeNames.count(sat.system)) continue;

        // ---- 拒绝检查 ----
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
        if (xyz.norm() > 1000.0 && elev < cutOffElev) {
            satRejected.insert(sat);
            nRejElev++;
            continue;
        }

        // IF 观测值有效性
        const string ifType = ifTypeNames.at(sat.system);
        double obsVal = 0.0;
        if (auto itObs = codeList.find(ifType); itObs != codeList.end() && itObs->second > 0.0) {
            obsVal = itObs->second;
        } else if (auto itRaw = codeList.find(ifCodeTypes.at(sat.system).first);
            itRaw != codeList.end() && itRaw->second > 0.0) {
            obsVal = itRaw->second;
        } else {
            satRejected.insert(sat);
            nRejIF++;
            continue;
        }

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
            trop = tropoHopfield(refHgt, elev);

        // 位置方程权
        double weight = 1.0 / (sigIFCode * sigIFCode);
        if (elev < PI / 6) {
            const double s = std::sin(elev);
            weight *= s * s;
        }

        // ---- 逐卫星构建方程 ----
        buildPosEquation(sat, pvt, los, rho, trop,
                         ifType, obsVal, weight, dx, dy, dz, cdtVars[sat.system]);
        // 速度方程与位置方程共用同一个 LOS（接收机→卫星方向）
        buildVelEquation(sat, codeList, pvt, los, elev, weight,
                         dvx, dvy, dvz, dcdt);

        activeSystems.insert(sat.system);
        nPass++;
        dbgCnt++;
    }
    posEquations.varSet.insert(dx);
    posEquations.varSet.insert(dy);
    posEquations.varSet.insert(dz);
    for (char sys : activeSystems) {
        if (auto it = cdtVars.find(sys); it != cdtVars.end())
            posEquations.varSet.insert(it->second);
    }
    velEquations.varSet.insert(dvx);
    velEquations.varSet.insert(dvy);
    velEquations.varSet.insert(dvz);
    velEquations.varSet.insert(dcdt);
}


void SPPIFCode::buildPosEquation(
    const SatID &sat, const PVT &pvt, const Vector3d &los,
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

void SPPIFCode::buildVelEquation(
    const SatID &sat, const TypeValueMap &codeList,
    const PVT &pvt, const Vector3d &los,
    const double elev, const double posWeight,
    const Variable &dvx, const Variable &dvy,
    const Variable &dvz, const Variable &dcdt) {
    // 多普勒代码：L1 频段，可能是 D1C (GPS C/A)、D1I (BDS B1I) 等。
    // 简单做法：找任何以 "D1" 开头的键。
    double dopplerVal = 0.0;
    bool found = false;
    for (const auto &[k, v]: codeList) {
        if (k.size() >= 2 && k[0] == 'D' && k[1] == '1') {
            dopplerVal = v;
            found = true;
            break;
        }
    }
    if (!found) return;

    const double f = getFreq(sat.system, "L1");
    const double lambda = C_MPS / f;
    const double rho_dot_obs = -lambda * dopplerVal;

    const double rho_dot_model = (vel - pvt.v).dot(los) + rClockDrift;

    const EquID eidVel(sat, "D1");
    EquData ed;
    ed.prefit = rho_dot_obs - rho_dot_model;

    ed.varCoeffData[dvx] = los[0];
    ed.varCoeffData[dvy] = los[1];
    ed.varCoeffData[dvz] = los[2];
    ed.varCoeffData[dcdt] = 1.0;

    double velWeight = posWeight / 40.0;
    if (elev < PI / 6) {
        const double s = std::sin(elev);
        velWeight *= s * s;
    }
    ed.weight = velWeight;

    velEquations.obsEquData[eidVel] = ed;
}

void SPPIFCode::computeSatPos(ObsData &obsData) {
    satPVTTransTime.clear();
    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        if (!ifTypeNames.count(sat.system)) continue;

        Ephemeris *eph = ephTable ? ephTable->find(sat, obsData.epoch) : nullptr;
        if (!eph) {
            auto itEph = ephMap.find(sat);
            if (itEph == ephMap.end()) continue;
            eph = itEph->second;
        }

        const string ifType = ifTypeNames[sat.system];
        double obsVal = 0.0;
        if (auto itObs = codeList.find(ifType); itObs != codeList.end() && itObs->second > 0.0) {
            obsVal = itObs->second;
        } else if (auto itRaw = codeList.find(ifCodeTypes.at(sat.system).first);
            itRaw != codeList.end() && itRaw->second > 0.0) {
            obsVal = itRaw->second;
        } else {
            continue;
        }

        const double tau_total = (obsVal - rClockBias[sat.system]) / C_MPS;
        CommonTime t_emit = obsData.epoch;
        t_emit.m_sod -= tau_total;

        satPVTTransTime[sat] = eph->svPVT(std::move(t_emit));
    }
}

IFCodeTypes SPPIFCode::autoDetectIFTypes(const std::map<char, std::vector<string> > &availableTypes) {
    IFCodeTypes result;

    for (const auto &[sys, types]: availableTypes) {
        if (sys == 'G') {
            string c1, c2;
            for (const auto &t: types) {
                if (t.size() >= 2 && t[0] == 'C') {
                    if (t[1] == '1' && c1.empty()) c1 = t;
                    if (t[1] == '2' && c2.empty()) c2 = t;
                }
            }
            if (!c1.empty() && !c2.empty()) result['G'] = {c1, c2};
            LOG_INFO << "Auto-detected IF types for G: " << c1 << "/" << c2;
        } else if (sys == 'C') {
            string cl, c6;
            for (const auto &t: types) {
                if (t.size() >= 2 && t[0] == 'C') {
                    if ((t[1] == '2' || t[1] == '1') && cl.empty()) cl = t;
                    if (t[1] == '6' && c6.empty()) c6 = t;
                }
            }
            if (cl.empty() || c6.empty()) {
                for (const auto &t: types) {
                    if (t.size() >= 2 && t[0] == 'C') {
                        if (t[1] == '7' && cl.empty()) cl = t;
                        if (t[1] == '6' && c6.empty()) c6 = t;
                    }
                }
            }
            if (!cl.empty() && !c6.empty()) result['C'] = {cl, c6};
            LOG_INFO << "Auto-detected IF types for C: " << cl << "/" << c6;
        }
    }
    return result;
}

void SPPIFCode::computeIF(ObsData &obsData) {
    for (auto &[sat, codeList]: obsData.satTypeValueData) {
        if (const char sys = sat.system; ifCodeTypes.count(sys)) {
            const auto [code1, code2] = ifCodeTypes.at(sys);
            if (codeList.count(code1) && codeList.count(code2)) {
                const double f1 = getFreq(sys, code1);
                const double f2 = getFreq(sys, code2);
                const double v1 = codeList.at(code1);
                const double v2 = codeList.at(code2);
                const double ifVal = (f1 * f1 * v1 - f2 * f2 * v2) / (f1 * f1 - f2 * f2);
                const string ifCode = "CC" + code1.substr(1, 1) + code2.substr(1, 1);
                codeList[ifCode] = ifVal;
            } else {
                satRejected.insert(sat);
            }
        }
    }
}


void SPPIFCode::earthRotation() {
    satPVTRecTime.clear();
    for (auto const &[sat, pvt]: satPVTTransTime) {
        const double tau = (pvt.p - xyz).norm() / C_MPS;

        const double wt = OMEGA_EARTH * tau;
        const double cos_wt = cos(wt);
        const double sin_wt = sin(wt);
        Matrix3d rot;
        rot << cos_wt, sin_wt, 0.0,
                -sin_wt, cos_wt, 0.0,
                0.0, 0.0, 1.0;
        PVT rotPvt = pvt;
        rotPvt.p = rot * pvt.p;
        rotPvt.v = rot * pvt.v;
        satPVTRecTime[sat] = rotPvt;
    }
}

void SPPIFCode::computeElevAzim() {
    for (auto const &[sat, pvt]: satPVTRecTime) {
        satElevData[sat] = elevation(xyz, pvt.p, frame);
        satAzimData[sat] = azimuth(xyz, pvt.p, frame);
    }
}
