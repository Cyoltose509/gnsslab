#include "SPPIFCode.h"
#include "CoordConvert.h"
#include <Eigen/Eigen>

void SPPIFCode::preprocess(ObsData &obsData) {
    satRejected.clear();
    setEphemeris(obsData.satEphemerisData);
    computeIF(obsData);
}

// ============================================================================
// solve — 主解算流程
//   每轮迭代：卫星位置 → 地球自转 → 高度角 → 位置方程 → 速度方程 → LSQ
//   粗差探测通过 Baarda w 检验自动在 buildPosEquations 中处理
// ============================================================================
void SPPIFCode::solve(ObsData &obsData) {
    xyz = obsData.antennaPosition;
    result.reset();
    lastWStats_.clear(); // 新历元：清空上一历元的 w 统计量

    int iter(0);
    double last_rms = 1e9;
    while (iter < 10) {
        // 卫星位置（Kepler 轨道）只在第 1 轮计算，后续迭代复用：
        // 迭代间接收机位置变化 < 100m → 卫星位置变化 < 0.1m，对 SPP 精度无影响
        if (iter == 0) {
            computeSatPos(obsData);
            earthRotation();
        }

        if (obsData.satTypeValueData.size() < 4) {
            throw SVNumException("num of satellites with valid ephemeris is less than 4 ("
                                 + to_string(satPVTRecTime.size()) + ")");
        }
        if (abs(xyz.norm() - RADIUS_EARTH) < 100000.0) {
            satElevData.clear();
            satAzimData.clear();
            computeElevAzim();
        }

        // ---- 构建观测方程（前 2 轮跳过 w 检验，避免收敛前误伤） ----
        linearize(obsData, iter);

        if (posEquations.obsEquData.size() < 4 || velEquations.obsEquData.size() < 4) {
            throw SVNumException("num of satellites after elevation cut is less than 4");
        }
        if (!isRover) break;

        // ---- 位置解算 ----
        posSolver.solve(posEquations);

        // ---- 速度解算 ----
        velSolver.solve(velEquations);

        // ---- 更新状态 ----
        const Vector3d dxyz = {
            posSolver.getSolution(Parameter::dX),
            posSolver.getSolution(Parameter::dY),
            posSolver.getSolution(Parameter::dZ)
        };
        const double d_cdt = posSolver.getSolution(Parameter::cdt);
        const double d_cdt2 = posSolver.getSolution(Parameter::cdt2);
        const Vector3d dvel = {
            velSolver.getSolution(Parameter::dVX),
            velSolver.getSolution(Parameter::dVY),
            velSolver.getSolution(Parameter::dVZ)
        };
        const double dcdt_dot = velSolver.getSolution(Parameter::cdtr_dot);

        xyz += dxyz;
        rClockBias += d_cdt;
        rClockBiasBDS += d_cdt2;
        vel += dvel;
        rClockDrift += dcdt_dot;

        const double rms = posSolver.v.norm() / sqrt(posSolver.v.size());
        if (fabs(rms - last_rms) < 1e-4) {
            break;
        }
        last_rms = rms;

        // 还没收敛 → 计算 w 统计量供下一轮粗差探测
        buildResidualMap();

        iter++;
    }

    if (iter >= 10) {
        throw InvalidSolver("too many iterations without convergence");
    }
    getResult();
}

void SPPIFCode::getResult() {
    auto &pD = posSolver.covMatrix;
    auto &vD = velSolver.covMatrix;
    result.pdop = sqrt(pD(0, 0) + pD(1, 1) + pD(2, 2));
    result.gdop = sqrt(pD(0, 0) + pD(1, 1) + pD(2, 2) + pD(3, 3) + pD(4, 4));

    result.tdop = sqrt(pD(3, 3) + pD(4, 4));

    result.sigmaP = result.pdop * posSolver.sigma0;
    result.sigmaV = sqrt(vD(0, 0) + vD(1, 1) + vD(2, 2)) * velSolver.sigma0;
    result.sigmaXYZ = {
        sqrt(pD(0, 0)) * posSolver.sigma0,
        sqrt(pD(1, 1)) * posSolver.sigma0,
        sqrt(pD(2, 2)) * posSolver.sigma0
    };
    result.sigmaVel = {
        sqrt(vD(0, 0)) * velSolver.sigma0,
        sqrt(vD(1, 1)) * velSolver.sigma0,
        sqrt(vD(2, 2)) * velSolver.sigma0
    };
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
        const double invW = (w > 0.0) ? 1.0 / w : 0.0;

        // 设计矩阵第 i 行 (1 × numUnk)
        const auto hRow = posSolver.hMatrix.row(i);

        // q_vv = P⁻¹_ii - h_i · N⁻¹ · h_iᵀ
        const double qvv = invW - (hRow * Ninv * hRow.transpose())(0, 0);

        if (qvv > 0.0) {
            const double wStat = std::abs(v) / (sigma0 * std::sqrt(qvv));
            lastWStats_[eid.sat] = wStat;
        }

        ++i;
    }
}

// ============================================================================
// linearize — 单次遍历所有卫星，完成拒绝检查 + 共享几何计算 + 组装观测方程
// ============================================================================
void SPPIFCode::linearize(ObsData &obsData, const int iter) {
    static constexpr double W_THRESHOLD = 3.29; // Baarda w 检验阈值 (α=0.001)

    // 前 2 轮跳过 w 检验：此时位置可能远未收敛
    const bool applyWTest = (iter >= 2);

    // ---- 先找本轮最可疑的卫星（w 统计量最大），只剔除一颗 ----
    SatID outlierSat;
    double worstW = 0.0;
    if (applyWTest) {
        for (auto const &[sat, codeList]: obsData.satTypeValueData) {
            if (satRejected.count(sat)) continue;
            if (auto itW = lastWStats_.find(sat);
                itW != lastWStats_.end() && itW->second > worstW) {
                worstW = itW->second;
                outlierSat = sat;
            }
        }
    }
    const bool rejectOutlier = (worstW > W_THRESHOLD);

    posEquations.reset();
    velEquations.reset();
    posEquations.station = obsData.station;
    velEquations.station = obsData.station;

    // ---- 未知参数（只构造一次，所有卫星共用） ----
    const Variable dx(obsData.station, Parameter::dX);
    const Variable dy(obsData.station, Parameter::dY);
    const Variable dz(obsData.station, Parameter::dZ);
    const Variable cdt(obsData.station, Parameter::cdt);
    const Variable cdt2(obsData.station, Parameter::cdt2);
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

    bool hasGps = false, hasBds = false;

    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        // ---- 拒绝检查 ----
        if (satRejected.count(sat)) continue;

        if (!satPVTRecTime.count(sat)) {
            satRejected.insert(sat);
            continue;
        }
        const auto &pvt = satPVTRecTime.at(sat);

        // Baarda w 检验：只剔除本轮最可疑的一颗卫星，避免异常星污染整个解
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
            continue;
        }

        // IF 观测值有效性
        const bool isGps = (sat.system == 'G');
        const string ifType = isGps ? "CC12" : "CC26";
        auto itObs = codeList.find(ifType);
        if (itObs == codeList.end() || itObs->second <= 0.0) {
            satRejected.insert(sat);
            continue;
        }
        const double obsVal = itObs->second;

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
                         ifType, obsVal, weight, dx, dy, dz, cdt, cdt2);
        // 速度方程需要卫星→接收机方向的 LOS（与位置方程的 sign 相反）
        buildVelEquation(sat, codeList, pvt, los, elev, weight,
                         dvx, dvy, dvz, dcdt);

        if (isGps) hasGps = true;
        else hasBds = true;
    }

    // ---- varSet 统一插入（只做一次，不在循环内重复） ----
    posEquations.varSet.insert(dx);
    posEquations.varSet.insert(dy);
    posEquations.varSet.insert(dz);
    if (hasGps) posEquations.varSet.insert(cdt);
    if (hasBds) posEquations.varSet.insert(cdt2);
    velEquations.varSet.insert(dvx);
    velEquations.varSet.insert(dvy);
    velEquations.varSet.insert(dvz);
    velEquations.varSet.insert(dcdt);
}

// ============================================================================
// buildPosEquation — 单颗卫星的位置观测方程
// ============================================================================
void SPPIFCode::buildPosEquation(
    const SatID &sat, const PVT &pvt, const Vector3d &los,
    const double rho, const double trop,
    const std::string &obsType, const double obsVal, const double weight,
    const Variable &dx, const Variable &dy, const Variable &dz,
    const Variable &cdt, const Variable &cdt2) {
    const bool isGps = (sat.system == 'G');
    const double dts = pvt.clockBias * C_MPS;
    const double rel = pvt.relativityCorrection * C_MPS;

    const EquID eid(sat, obsType);
    EquData ed;
    ed.prefit = obsVal - (rho + (isGps ? rClockBias : rClockBiasBDS) - dts - rel + trop);

    ed.varCoeffData[dx] = los[0];
    ed.varCoeffData[dy] = los[1];
    ed.varCoeffData[dz] = los[2];
    if (isGps)
        ed.varCoeffData[cdt] = 1.0;
    else
        ed.varCoeffData[cdt2] = 1.0;

    ed.weight = weight;
    posEquations.obsEquData[eid] = ed;
}

// ============================================================================
// buildVelEquation — 单颗卫星的速度观测方程（多普勒 D1）
// ============================================================================
void SPPIFCode::buildVelEquation(
    const SatID &sat, const TypeValueMap &codeList,
    const PVT &pvt, const Vector3d &los,
    const double elev, const double posWeight,
    const Variable &dvx, const Variable &dvy,
    const Variable &dvz, const Variable &dcdt) {
    const auto itD = codeList.find("D1");
    if (itD == codeList.end()) return;

    const double f = getFreq(sat.system, "L1");
    const double lambda = C_MPS / f;
    const double rho_dot_obs = -lambda * itD->second;

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

// ============================================================================
// computeSatPos — 卫星位置计算（发射时刻）
// ============================================================================
void SPPIFCode::computeSatPos(ObsData &obsData) {
    satPVTTransTime.clear();
    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        auto itEph = ephMap.find(sat);
        if (itEph == ephMap.end()) continue;
        const auto &eph = itEph->second;

        const bool isGps = (sat.system == 'G');
        const string ifType = isGps ? "CC12" : "CC26";
        auto itObs = codeList.find(ifType);
        if (itObs == codeList.end() || itObs->second <= 0.0) continue;
        const double obsVal = itObs->second;

        const double tau_total = (obsVal - (isGps ? rClockBias : rClockBiasBDS)) / C_MPS;
        CommonTime t_emit = obsData.epoch;
        t_emit.m_sod -= tau_total;

        satPVTTransTime[sat] = eph->svPVT(std::move(t_emit), oldVersion);
    }
}

void SPPIFCode::computeIF(ObsData &obsData) {
    for (auto &[sat, codeList]: obsData.satTypeValueData) {
        if (const char sys = sat.system; ifCodeTypes.count(sys)) {
            const auto [code1, code2] = ifCodeTypes.at(sys);
            const double f1 = getFreq(sys, code1);
            const double f2 = getFreq(sys, code2);
            if (codeList.count(code1) && codeList.count(code2)) {
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
