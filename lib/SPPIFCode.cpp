#include "SPPIFCode.h"
#include "CoordConvert.h"
#include <Eigen/Eigen>

void SPPIFCode::preprocess(ObsData &obsData) {
    satRejected.clear();
    setEphemeris(obsData.satEphemerisData);
    computeIF(obsData);
}

void SPPIFCode::solve(ObsData &obsData) {
    xyz = obsData.antennaPosition;
    result.reset();
    int iter(0);
    while (iter < 10) {
        computeSatPos(obsData);
        earthRotation();

        if (obsData.satTypeValueData.size() < 4) {
            throw SVNumException("num of satellites with valid ephemeris is less than 4 (" + to_string(satPVTRecTime.size()) + ")");
        }
        if (abs(xyz.norm() - RADIUS_EARTH) < 100000.0) {
            satElevData.clear();
            satAzimData.clear();
            computeElevAzim();
        }

        linearize(obsData, iter);

        if (posEquations.obsEquData.size() < 4 || velEquations.obsEquData.size() < 4) {
            throw SVNumException("num of satellites after elevation cut is less than 4");
        }
        if (!isRover) break;
        // 解算位置
        posSolver.solve(posEquations);
        // 解算速度
        velSolver.solve(velEquations);
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
        if (dxyz.norm() < 0.0001 && abs(d_cdt) < 0.0001 && abs(d_cdt2) < 0.0001 && iter >= 3) {
            break;
        }
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

void SPPIFCode::computeSatPos(ObsData &obsData) {
    satPVTTransTime.clear();
    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        auto it = ephMap.find(sat);
        if (it == ephMap.end()) continue;
        const auto &eph = it->second;

        // Choose observation to estimate travel time
        double obsVal = 0.0;
        if (string ifType = sat.system == 'G' ? "CC12" : "CC26"; codeList.count(ifType)) obsVal = codeList.at(ifType);

        if (obsVal <= 0.0) continue;

        const double tau_total = (obsVal - (sat.system == 'G' ? rClockBias : rClockBiasBDS)) / C_MPS;
        CommonTime t_emit = obsData.epoch;
        t_emit.m_sod -= tau_total;

        satPVTTransTime[sat] = eph->svPVT(std::move(t_emit), oldVersion);
    }
}

void SPPIFCode::computeIF(ObsData &obsData) {
    for (auto &[sat, codeList]: obsData.satTypeValueData) {
        // cout << sat<< ' '<< codeList << endl;
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

        const double omega = sat.getFrame().omega;
        const double wt = omega * tau;
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

double tropoHopfield(const double H, const double E) {
    constexpr double T0 = 288.16; // K
    constexpr double P0 = 1013.25; // hPa
    constexpr double RH0 = 0.5; // 相对湿度
    constexpr double H0 = 0.0; // 参考高度
    const double T = T0 - 0.0065 * (H - H0);
    if (T < 200.0) return 0.0;
    const double P = P0 * pow(1 - 0.0000226 * (H - H0), 5.225);
    const double RH = RH0 * exp(-0.0006396 * (H - H0));
    const double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
    constexpr double hd = 40136.0 + 148.72 * (T0 - 273.16); // 干层高度
    constexpr double hw = 11000.0; // 湿层高度
    const double Kd = 155.2e-7 * (P / T) * (hd - H);
    const double Kw = 155.2e-7 * (4810.0 * e / (T * T)) * (hw - H);
    const double md = 1.0 / sin(sqrt(E * E + 1.90386e-3));
    const double mw = 1.0 / sin(sqrt(E * E + 6.85389e-4));
    const double tropo = Kd * md + Kw * mw;
    if (!isfinite(tropo) || tropo < 0.0 || tropo > 100.0)
        return 0.0;

    return tropo;
}


void SPPIFCode::linearize(ObsData &obsData, int iterCount) {
    EquSys equSysTemp;
    posEquations.reset();
    velEquations.reset();
    posEquations.station = obsData.station;
    velEquations.station = obsData.station;
    const Variable dx(obsData.station, Parameter::dX);
    const Variable dy(obsData.station, Parameter::dY);
    const Variable dz(obsData.station, Parameter::dZ);
    const Variable cdt(obsData.station, Parameter::cdt);
    const Variable cdt2(obsData.station, Parameter::cdt2);


    const Variable dvx(obsData.station, Parameter::dVX);
    const Variable dvy(obsData.station, Parameter::dVY);
    const Variable dvz(obsData.station, Parameter::dVZ);
    const Variable dcdt(obsData.station, Parameter::cdtr_dot);
    int iobs = 0;
    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        if (satRejected.count(sat)) continue;
        if (!satPVTRecTime.count(sat)) {
            satRejected.insert(sat);
            continue;
        }
        if (iterCount >= 1 && abs(posSolver.v[iobs++]) > posSolver.sigma0) {
            satRejected.insert(sat);
            continue;
        }

        const auto &pvt = satPVTRecTime.at(sat);

        double elev = PI * 0.5;
        if (satElevData.count(sat)) elev = satElevData.at(sat);

        if (xyz.norm() > 1000.0 && elev < cutOffElev) {
            satRejected.insert(sat);
            continue;
        }

        // Choose observation (following same logic as computeSatPos)
        double obsVal = 0.0;
        string usedType;

        if (string ifType = sat.system == 'G' ? "CC12" : "CC26"; codeList.count(ifType)) {
            obsVal = codeList.at(ifType);
            usedType = ifType;
        }

        if (usedType.empty() || obsVal <= 0.0) {
            satRejected.insert(sat);
            continue;
        }

        double rho = (pvt.p - xyz).norm();
        if (rho < 1.0) rho = 20000000.0;

        const double dts = pvt.clockBias * C_MPS;
        const double rel = pvt.relativityCorrection * C_MPS;
        double trop = 0.0;
        if (xyz.norm() > 1000.0) {
            auto BLH = XYZtoBLH(xyz, frame);
            trop = tropoHopfield(BLH[2], elev);
        }

        const EquID eid(sat, usedType);
        EquData ed;
        ed.prefit = obsVal - (rho + (sat.system == 'G' ? rClockBias : rClockBiasBDS) - dts - rel + trop);

        ed.varCoeffData[dx] = -(pvt.p[0] - xyz[0]) / rho;
        ed.varCoeffData[dy] = -(pvt.p[1] - xyz[1]) / rho;
        ed.varCoeffData[dz] = -(pvt.p[2] - xyz[2]) / rho;
        if (sat.system == 'G') {
            ed.varCoeffData[cdt] = 1.0;
            posEquations.varSet.insert(cdt);
        } else {
            ed.varCoeffData[cdt2] = 1.0;
            posEquations.varSet.insert(cdt2);
        }

        // double weight = 1.0;
        // if (xyz.norm() > 1000.0) {
        //     double sinE = sin(max(elev, 5.0) * DEG_TO_RAD);
        //     weight = sinE * sinE;
        // }
        double weight = 1.0 / (sigIFCode * sigIFCode);
        if (elev < PI / 6) {
            weight *= std::pow(std::sin(elev), 2);
        }
        ed.weight = weight; //1.0 / (pow(sin(elev),2) + 0.01);// weight / (sigIFCode * sigIFCode);

        posEquations.obsEquData[eid] = ed;
        posEquations.varSet.insert(dx);
        posEquations.varSet.insert(dy);
        posEquations.varSet.insert(dz);


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

            double velWeight = weight / 40.0;
            if (elev < PI / 6) {
                velWeight *= std::pow(std::sin(elev), 2);
            }
            ed_vel.weight = velWeight;

            velEquations.obsEquData[eid_vel] = ed_vel;
            velEquations.varSet.insert(dvx);
            velEquations.varSet.insert(dvy);
            velEquations.varSet.insert(dvz);
            velEquations.varSet.insert(dcdt);
        }
    }
}
