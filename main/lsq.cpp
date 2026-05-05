#include <iostream>
#include <map>
#include <vector>
#include <iomanip>
#include <fstream>
#include "OEM7Reader.h"
#include "SolverLSQ.h"
#include "CoordConvert.h"
#include "Const.h"

using namespace std;

inline double getTropDelay(const double elev_deg) {
    return 2.3 / sin(max(elev_deg,5.0) * DEG_TO_RAD);
}

double getObsValue(const SatID& sat, const TypeValueMap& obs) {
    const auto [code1, code2, valid] = getDualCode(sat.system);
    if (!valid) return 0.0;

    const auto it1 = obs.find(code1);
    const auto it2 = obs.find(code2);
    //双频无电离层组合
    if (it1 != obs.end() && it2 != obs.end()) {
        const double f1 = getFreq(sat.system, code1);
        const double f2 = getFreq(sat.system, code2);

        if (const double denom = f1 * f1 - f2 * f2; f1 > 0.0 && f2 > 0.0 && denom != 0.0) {
            const double P1 = it1->second;
            const double P2 = it2->second;

            return (f1 * f1 * P1 - f2 * f2 * P2) / denom;
        }
    }
    if (it1 != obs.end()) return it1->second;
    if (it2 != obs.end()) return it2->second;
    return 0.0;
}

int main() {
    const string filename = "NovatelOEM20211114-01.log";
    OEM7Reader oem7;
    if (!oem7.open(filename)) {
        cout << "Failed to open file: " << filename << endl;
        return -1;
    }

    ofstream outfile("spp_results.csv");
    if (!outfile.is_open()) {
        cout << "Failed to create output file." << endl;
        return -1;
    }

    outfile << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m" << endl;

    ObsData obs;
    int epochProcessed = 0;
    int successCount = 0;

    Variable varX("base", Parameter::dX);
    Variable varY("base", Parameter::dY);
    Variable varZ("base", Parameter::dZ);
    Variable varCdt("base", Parameter::cdt);

    while (oem7.getNextEpoch(obs)) {
        epochProcessed++;

        Vector4d x_state = Vector4d::Zero();
        bool converged = false;
        int iter = 0;

        while (iter < 10 && !converged) {
            EquSys equSys;
            equSys.station = "base";
            equSys.varSet.insert(varX);
            equSys.varSet.insert(varY);
            equSys.varSet.insert(varZ);
            equSys.varSet.insert(varCdt);

            for (auto &[sat, snd]: obs.satTypeValueData) {
                const Vector3d &pos = x_state.head<3>();
                double clk = x_state[3];
                double pr = getObsValue(sat, snd);
                if (pr <= 0) continue;

                Ephemeris *eph = nullptr;
                if (sat.system == 'G' && oem7.latestGps.count(sat.id)) {
                    eph = &oem7.latestGps.at(sat.id);
                } else if (sat.system == 'C' && oem7.latestBds.count(sat.id)) {
                    eph = &oem7.latestBds.at(sat.id);
                }

                if (!eph) continue;

                double tau = pr / C_MPS;
                auto t_emit = obs.epoch;
                t_emit.m_sod -= tau + clk / C_MPS;

                auto pvt = eph->svPVT(t_emit);
                double dts = pvt.clockBias;
                double rel = pvt.relativityCorrection;

                if (sat.system == 'C') {
                    if (auto *beph = dynamic_cast<BDSEphem *>(eph)) {
                        double f1 = getFreq('C', 2);
                        double f2 = getFreq('C', 6);
                        double alpha = f1 * f1 / (f1 * f1 - f2 * f2);
                        dts -= alpha * beph->tgd1;
                    }
                }


                double d_omega = Frame::WGS84.omega * tau;
                auto &satPos = pvt.p;
                Vector3d correctedSatPos;
                correctedSatPos[0] = satPos[0] * cos(d_omega) + satPos[1] * sin(d_omega);
                correctedSatPos[1] = -satPos[0] * sin(d_omega) + satPos[1] * cos(d_omega);
                correctedSatPos[2] = satPos[2];

                double rho = (correctedSatPos - pos).norm();
                if (rho < 1.0) rho = 20000000.0;

                double elev = 90.0;
                if (pos.norm() > 1000.0) {
                    try { elev = elevation(XYZ(pos), XYZ(correctedSatPos)); } catch (...) { elev = 0.0; }
                }

                if (elev < 10.0 && pos.norm() > 1000.0) continue;

                double tropo = pos.norm() > 1000.0 ? getTropDelay(elev) : 0.0;

                EquID eid(sat, "Obs");
                EquData ed;
                ed.prefit = pr - (rho + clk - C_MPS * dts - C_MPS * rel + tropo);
                ed.varCoeffData[varX] = -(correctedSatPos[0] - x_state[0]) / rho;
                ed.varCoeffData[varY] = -(correctedSatPos[1] - x_state[1]) / rho;
                ed.varCoeffData[varZ] = -(correctedSatPos[2] - x_state[2]) / rho;
                ed.varCoeffData[varCdt] = 1.0;
                ed.weight = 1.0;
                equSys.obsEquData[eid] = ed;
            }

            if (equSys.obsEquData.size() < 4) break;

            try {
                SolverLSQ solverLSQ;
                solverLSQ.solve(equSys);
                Vector3d dx = solverLSQ.getDXYZ();
                double d_cdt = solverLSQ.getSolution(Parameter::cdt);

                x_state[0] += dx[0];
                x_state[1] += dx[1];
                x_state[2] += dx[2];
                x_state[3] += d_cdt;

                if (dx.norm() < 1e-4) {
                    converged = true;
                    successCount++;
                    XYZ xyz(x_state.head<3>());
                    BLH blh = xyz2blh(xyz, Frame::WGS84);

                    // 控制台输出
                    // cout << fixed << setprecision(4)
                    //      << "Epoch " << setw(4) << successCount << ": " << CommonTime2CivilTime(obs.epoch)
                    //      << " B=" << setw(10) << blh.B() * RAD_TO_DEG
                    //      << " L=" << setw(11) << blh.L() * RAD_TO_DEG
                    //      << " H=" << setw(10) << blh.H()
                    //      << " sats=" << setw(2) << equSys.obsEquData.size()
                    // <<"X="<<setw(10)<<xyz.X()
                    // <<"Y="<<setw(10)<<xyz.Y()
                    // <<"Z="<<setw(10)<<xyz.Z()
                    // << endl;

                    // 文件输出

                    // outfile << successCount << ", " << CommonTime2CivilTime(obs.epoch) << ", "
                    //         << fixed << setprecision(8) << blh.B() * RAD_TO_DEG << ", "
                    //         << blh.L() * RAD_TO_DEG << ", "
                    //         << setprecision(4) << blh.H() << ", "
                    //         << equSys.obsEquData.size() << endl;
                    outfile << obs.weekSecond.week << ","
                            << fixed << setprecision(3) << obs.weekSecond.sow << ","
                            << fixed << setprecision(4)
                            << xyz[0] << ","
                            << xyz[1] << ","
                            << xyz[2]
                            << endl;
                }
            } catch (...) { break; }
            iter++;
        }
        //   if (epochProcessed >= 2000) break;
    }

    outfile.close();
    return 0;
}
