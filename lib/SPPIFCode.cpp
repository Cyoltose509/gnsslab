#include "SPPIFCode.h"
#include "CoordConvert.h"
#include <Eigen/Eigen>

void SPPIFCode::solve(ObsData &obsData) {
    // 1. Convert observation types (e.g., C1C -> C1)
    convertObsType(obsData);

    // 2. Compute IF combination if possible (don't reject if fails)
    computeIF(obsData);

    // 3. Initialize state
    // 计算发射时刻卫星位置（参考框架为时刻的）
    satPVTTransTime = computeSatPos(obsData);

    //----------------------
    // 得到卫星发射时刻位置和钟差、相对论和TGD后，改正观测值延迟，并更新C1/C2等观测值
    //----------------------
    // todo:
    // correctTGD(obsData);

    xyz = obsData.antennaPosition;
    result.reset();


    int iter(0);
    while (iter < 20) {
        satPVTTransTime = computeSatPos(obsData);
        satPVTRecTime = earthRotation();

        if (obsData.satTypeValueData.size() < 4) {
            throw SVNumException("num of satellites with valid ephemeris is less than 4 (" + to_string(satPVTRecTime.size()) + ")");
        }
        if (abs(xyz.norm() - RADIUS_EARTH) < 100000.0) {
            satElevData.clear();
            satAzimData.clear();
            computeElevAzim();
        }

        linearize(obsData);

        if (posEquations.obsEquData.size() < 4 || velEquations.obsEquData.size() < 4) {
            throw SVNumException("num of satellites after elevation cut is less than 4");
        }
        if (!isRover) break;
        // 解算位置
        posSolver.solve(posEquations);
        const Vector3d dxyz = {
            posSolver.getSolution(Parameter::dX),
            posSolver.getSolution(Parameter::dY),
            posSolver.getSolution(Parameter::dZ)
        };
        const double d_cdt = posSolver.getSolution(Parameter::cdt);
        result.pdop = sqrt(posSolver.covMatrix(0, 0) +
                           posSolver.covMatrix(1, 1) +
                           posSolver.covMatrix(2, 2));
        result.sigmaP = result.pdop * posSolver.sigma0;

        // 解算速度
        velSolver.solve(velEquations);
        const Vector3d dvel = {
            velSolver.getSolution(Parameter::dVX),
            velSolver.getSolution(Parameter::dVY),
            velSolver.getSolution(Parameter::dVZ)
        };
        const double dcdt_dot = velSolver.getSolution(Parameter::cdtr_dot);
        result.sigmaV = sqrt(velSolver.covMatrix(0, 0) +
                             velSolver.covMatrix(1, 1) +
                             velSolver.covMatrix(2, 2)) * velSolver.sigma0;
        xyz += dxyz;
        rClockBias += d_cdt;
        vel += dvel;
        rClockDrift += dcdt_dot;

        if (dxyz.norm() < 0.0001 && abs(d_cdt) < 0.0001) {
            break;
        }
        iter++;
    }

    if (iter >= 10) {
        throw InvalidSolver("too many iterations without convergence");
    }

    result.xyz = xyz;
    result.vel = vel;
    result.blh = XYZtoBLH(xyz, frame);
    result.numSats = static_cast<int>(satPVTRecTime.size());
}

map<SatID, PVT> SPPIFCode::computeSatPos(ObsData &obsData) {
    map<SatID, PVT> satPVTData;
    satTropData.clear(); // Re-purpose this to store obs values for Sagnac if needed, 
    // but we'll use a better way.

    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        auto it = ephMap.find(sat);
        if (it == ephMap.end()) continue;
        Ephemeris *eph = it->second;

        // Choose observation to estimate travel time
        double obsVal = 0.0;
        if (string ifType = sat.system == 'G' ? "CC12" : "CC26"; codeList.count(ifType)) obsVal = codeList.at(ifType);
        else if (sat.system == 'G') {
            if (codeList.count("C1")) obsVal = codeList.at("C1");
            else if (codeList.count("C2")) obsVal = codeList.at("C2");
        } else if (sat.system == 'C') {
            if (codeList.count("C2")) obsVal = codeList.at("C2");
            else if (codeList.count("C6")) obsVal = codeList.at("C6");
        }

        if (obsVal <= 0.0) continue;

        // Emission time estimate (following lsq.cpp logic)
        // t_emit = t_receive_rcvr - (P + clk_rcvr)/c
        const double tau_total = (obsVal + rClockBias) / C_MPS;
        CommonTime t_emit = obsData.epoch;
        t_emit.m_sod -= tau_total;

        PVT pvt = eph->svPVT(t_emit);

        // BDS TGD correction
        if (sat.system == 'C') {
            if (const auto *beph = dynamic_cast<const BDSEphem *>(eph)) {
                const double f1 = getFreq('C', 2);
                const double f2 = getFreq('C', 6);
                const double alpha = f1 * f1 / (f1 * f1 - f2 * f2);
                pvt.clockBias -= alpha * beph->tgd1;
            }
        }

        // Store the raw observation value in a temporary map for Sagnac correction
        // We'll use satTropData for this to avoid header changes
        satTropData[sat] = obsVal;

        satPVTData[sat] = pvt;
    }

    return satPVTData;
}

void SPPIFCode::convertObsType(ObsData &obsData) {
    SatTypeValueMap stvData;
    for (const auto &[sat, codeList]: obsData.satTypeValueData) {
        TypeValueMap tvData;
        for (const auto &[lCode, v]: codeList) {
            if (lCode.length() >= 2)
                tvData[lCode.substr(0, 2)] = v;
            else
                tvData[lCode] = v;
        }
        stvData[sat] = tvData;
    }
    obsData.satTypeValueData = stvData;
}

void SPPIFCode::computeIF(ObsData &obsData) const {
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
            }
        }
    }
}

map<SatID, PVT> SPPIFCode::earthRotation() {
    map<SatID, PVT> satPVTRecTimeMap;
    for (auto const &[sat, pvt]: satPVTTransTime) {
        // Use the pseudorange to estimate signal travel time for Sagnac (following lsq.cpp)
        double tau = (pvt.p - xyz).norm() / C_MPS;
        if (satTropData.count(sat)) {
            tau = satTropData.at(sat) / C_MPS;
        }
        const double wt = OMEGA_EARTH * tau;
        const double cos_wt = cos(wt);
        const double sin_wt = sin(wt);
        Matrix3d rot;
        rot << cos_wt, -sin_wt, 0.0,
                sin_wt, cos_wt, 0.0,
                0.0, 0.0, 1.0;
        PVT rotPvt = pvt;
        rotPvt.p = rot * pvt.p;
        rotPvt.v = rot * pvt.v;
        satPVTRecTimeMap[sat] = rotPvt;
    }
    return satPVTRecTimeMap;
}

void SPPIFCode::computeElevAzim() {
    for (auto const &[sat, pvt]: satPVTRecTime) {
        satElevData[sat] = elevation(xyz, pvt.p);
        satAzimData[sat] = azimuth(xyz, pvt.p);
    }
}

double tropoCorrection(const double B_rad, const double H, const double elev_deg)
{
    // 1. 高度角（转弧度 + 限制）
    double E = elev_deg * PI / 180.0;
    if (E < 5.0 * PI / 180.0) E = 5.0 * PI / 180.0;
    if (E > 85.0 * PI / 180.0) E = 85.0 * PI / 180.0;

    // 2. 标准大气模型
    constexpr double T0 = 288.15;     // K
   constexpr  double P0 = 1013.25;    // hPa
    constexpr double RH0 = 0.5;       // 相对湿度

    const double P = P0 * pow(1 - 2.2557e-5 * H, 5.2568);
    const double T = T0 - 0.0065 * H;

    if (!isfinite(P) || !isfinite(T) || T < 200.0)
        return 0.0;

    // 饱和水汽压（hPa）
    const double es = 6.112 * exp(17.67 * (T - 273.15) / (T - 29.65));
    const double e = RH0 * es;

    // 3. Saastamoinen 天顶延迟
    const double denom = 1.0 - 0.00266 * cos(2.0 * B_rad) - 0.00028 * H / 1000.0;
    if (fabs(denom) < 1e-6) return 0.0;

    const double ZHD = 0.0022768 * P / denom;
    const double ZWD = 0.002277 * (1255.0 / T + 0.05) * e / denom;


    // 4. Niell Mapping Function（简化版）
    const double sinE = sin(E);

    // 干分量 mapping
    const double md = 1.0 / (sinE + 0.00143 / (tan(E) + 0.0455));

    // 湿分量 mapping（稍微不同）
     const double mw = 1.0 / (sinE + 0.00035 / (tan(E) + 0.017));

    // 5. 总对流层延迟
     const double tropo = ZHD * md + ZWD * mw;

    if (!isfinite(tropo) || tropo < 0.0 || tropo > 50.0)
        return 0.0;

    return tropo;
}


void SPPIFCode::linearize(ObsData &obsData) {
    EquSys equSysTemp;
    posEquations.reset();
    velEquations.reset();
    posEquations.station = obsData.station;
    velEquations.station = obsData.station;
    const Variable dx(obsData.station, Parameter::dX);
    const Variable dy(obsData.station, Parameter::dY);
    const Variable dz(obsData.station, Parameter::dZ);
    const Variable cdt(obsData.station, Parameter::cdt);

    const Variable dvx(obsData.station, Parameter::dVX);
    const Variable dvy(obsData.station, Parameter::dVY);
    const Variable dvz(obsData.station, Parameter::dVZ);
    const Variable dcdt(obsData.station, Parameter::cdtr_dot);

    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        if (!satPVTRecTime.count(sat)) continue;

        const PVT &pvt = satPVTRecTime.at(sat);

        double elev = 90.0;
        if (satElevData.count(sat)) elev = satElevData.at(sat);

        if (xyz.norm() > 1000.0 && elev < cutOffElev) continue;

        // Choose observation (following same logic as computeSatPos)
        double obsVal = 0.0;
        string usedType;

        if (string ifType = sat.system == 'G' ? "CC12" : "CC26"; codeList.count(ifType)) {
            obsVal = codeList.at(ifType);
            usedType = ifType;
        } else if (sat.system == 'G' && codeList.count("C1")) {
            obsVal = codeList.at("C1");
            usedType = "C1";
        } else if (codeList.count("C2")) {
            obsVal = codeList.at("C2");
            usedType = "C2";
        }

        if (obsVal <= 0.0) continue;

        double rho = (pvt.p - xyz).norm();
        if (rho < 1.0) rho = 20000000.0;

        const double dts = pvt.clockBias * C_MPS;
        const double rel = pvt.relativityCorrection * C_MPS;
        double trop = 0.0;
        if (xyz.norm() > 1000.0) {
            auto BLH = XYZtoBLH(xyz, frame);
            trop = tropoCorrection(BLH[0],BLH[2], elev);
           //trop = 2.3 / sin(max(elev, 5.0) * DEG_TO_RAD);
        }

        const EquID eid(sat, usedType);
        EquData ed;
        ed.prefit = obsVal - (rho + rClockBias - dts - rel + trop);

        ed.varCoeffData[dx] = -(pvt.p[0] - xyz[0]) / rho;
        ed.varCoeffData[dy] = -(pvt.p[1] - xyz[1]) / rho;
        ed.varCoeffData[dz] = -(pvt.p[2] - xyz[2]) / rho;
        ed.varCoeffData[cdt] = 1.0;

        // double weight = 1.0;
        // if (xyz.norm() > 1000.0) {
        //     double sinE = sin(max(elev, 5.0) * DEG_TO_RAD);
        //     weight = sinE * sinE;
        // }
        double sinE = sin(max(elev, 5.0) * DEG_TO_RAD);
        double sigma = 0.5 + 1.5 / sinE;
        double weight = 1.0 / (sigma * sigma);
        ed.weight = weight; //1.0 / (pow(sin(elev),2) + 0.01);// weight / (sigIFCode * sigIFCode);

        posEquations.obsEquData[eid] = ed;
        posEquations.varSet.insert(dx);
        posEquations.varSet.insert(dy);
        posEquations.varSet.insert(dz);
        posEquations.varSet.insert(cdt);

        if (codeList.count("D1")) {
            double D = codeList.at("D1");

            double f = getFreq(sat.system, "L1");
            double lambda = C_MPS / f;

            double rho_dot_obs = -lambda * D;

            Vector3d los = (pvt.p - xyz).normalized();

            Vector3d vs = pvt.v;


            // 模型值
            double rho_dot_model = (vs - vel).dot(los) + rClockDrift;

            // 构造观测方程
            EquID eid_vel(sat, "D1");

            EquData ed_vel;
            ed_vel.prefit = rho_dot_obs - rho_dot_model;
            ed_vel.varCoeffData[dvx] = -los[0];
            ed_vel.varCoeffData[dvy] = -los[1];
            ed_vel.varCoeffData[dvz] = -los[2];
            ed_vel.varCoeffData[dcdt] = 1.0;

            ed_vel.weight = weight / 4.0;

            velEquations.obsEquData[eid_vel] = ed_vel;
            velEquations.varSet.insert(dvx);
            velEquations.varSet.insert(dvy);
            velEquations.varSet.insert(dvz);
            velEquations.varSet.insert(dcdt);
        }
    }
}
