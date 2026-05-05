#include "SPPIFCode.h"
#include "CoordConvert.h"
#include <Eigen/Eigen>

#define debug 0

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
    dxyz = {100, 100, 100};

    int iter(0);
    while (iter < 10) {
        // 4. Compute satellite positions at transmit time
        satPVTTransTime = computeSatPos(obsData);

        // 5. Apply Earth rotation (Sagnac effect)
        satPVTRecTime = earthRotation();

        // 6. Check satellite count
        if (obsData.satTypeValueData.size() < 4) {
            throw SVNumException("num of satellites with valid ephemeris is less than 4 (" + to_string(satPVTRecTime.size()) + ")");
        }

        // 7. Compute elevation and azimuth (only near earth surface)
        if (std::abs(xyz.norm() - RADIUS_EARTH) < 100000.0) {
            satElevData.clear();
            satAzimData.clear();
            computeElevAzim();
        }

        // 8. Linearize and form equations
        equSys = linearize(obsData);

        // If we only have < 4 equations after elevation cut, we fail
        if (equSys.obsEquData.size() < 4) {
            throw SVNumException("num of satellites after elevation cut is less than 4");
        }

        // 9. Solve LSQ
        if (!isRover) break; // Base station location is fixed

        solver.solve(equSys);
        dxyz = solver.getDXYZ();
        const double d_cdt = solver.getSolution(Parameter::cdt);

        // 10. Update state
        xyz += dxyz;
        rcvrClk += d_cdt;

        // cout << "Iteration " << iter << ": dxyz=" << dxyz.norm()
        //         << " d_cdt=" << d_cdt << " xyz=" << xyz.transpose() << endl;

        // 11. Check convergence
        if (dxyz.norm() < 0.0001 && std::abs(d_cdt) < 0.0001) {
            break;
        }
        iter++;
    }

    if (iter >= 10) {
        throw InvalidSolver("too many iterations without convergence");
    }

    result.xyz = xyz;
}

std::map<SatID, PVT> SPPIFCode::computeSatPos(ObsData &obsData) {
    std::map<SatID, PVT> satPVTData;
    satTropData.clear(); // Re-purpose this to store obs values for Sagnac if needed, 
    // but we'll use a better way.

    for (auto const &[sat, codeList]: obsData.satTypeValueData) {
        auto it = ephMap.find(sat);
        if (it == ephMap.end()) continue;
        Ephemeris *eph = it->second;

        // Choose observation to estimate travel time
        double obsVal = 0.0;
        if (string ifType = (sat.system == 'G') ? "CC12" : "CC26"; codeList.count(ifType)) obsVal = codeList.at(ifType);
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
        const double tau_total = (obsVal + rcvrClk) / C_MPS;
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

std::map<SatID, PVT> SPPIFCode::earthRotation() {
    std::map<SatID, PVT> satPVTRecTimeMap;
    for (auto const &[sat, pvt]: satPVTTransTime) {
        // Use the pseudorange to estimate signal travel time for Sagnac (following lsq.cpp)
        double tau = (pvt.p - xyz).norm() / C_MPS;
        if (satTropData.count(sat)) {
            tau = satTropData.at(sat) / C_MPS;
        }

        const double wt = OMEGA_EARTH * tau;
        const double coswt = cos(wt);
        const double sinwt = sin(wt);

        PVT rotPvt = pvt;
        rotPvt.p[0] = coswt * pvt.p[0] + sinwt * pvt.p[1];
        rotPvt.p[1] = -sinwt * pvt.p[0] + coswt * pvt.p[1];
        // z stays same

        rotPvt.v[0] = coswt * pvt.v[0] + sinwt * pvt.v[1];
        rotPvt.v[1] = -sinwt * pvt.v[0] + coswt * pvt.v[1];

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

EquSys SPPIFCode::linearize(ObsData &obsData) {
    EquSys equSysTemp;
    equSysTemp.station = obsData.station;

    const Variable dx(obsData.station, Parameter::dX);
    const Variable dy(obsData.station, Parameter::dY);
    const Variable dz(obsData.station, Parameter::dZ);
    const Variable cdt(obsData.station, Parameter::cdt);

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
        } else if (sat.system == 'C' && codeList.count("C2")) {
            obsVal = codeList.at("C2");
            usedType = "C2";
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
            trop = 2.3 / sin(max(elev, 5.0) * DEG_TO_RAD);
        }

        const EquID eid(sat, usedType);
        EquData ed;
        ed.prefit = obsVal - (rho + rcvrClk - dts - rel + trop);

        ed.varCoeffData[dx] = -(pvt.p[0] - xyz[0]) / rho;
        ed.varCoeffData[dy] = -(pvt.p[1] - xyz[1]) / rho;
        ed.varCoeffData[dz] = -(pvt.p[2] - xyz[2]) / rho;
        ed.varCoeffData[cdt] = 1.0;

        double weight = 1.0;
        // lsq.cpp is unweighted, but sin^2 E is better for stability
        if (xyz.norm() > 1000.0) {
            double sinE = sin(max(elev, 5.0) * DEG_TO_RAD);
            weight = sinE * sinE;
        }
        ed.weight = weight / (sigIFCode * sigIFCode);

        equSysTemp.obsEquData[eid] = ed;
        equSysTemp.varSet.insert(dx);
        equSysTemp.varSet.insert(dy);
        equSysTemp.varSet.insert(dz);
        equSysTemp.varSet.insert(cdt);
    }

    return equSysTemp;
}
