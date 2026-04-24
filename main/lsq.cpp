#include <iostream>
#include <map>
#include <vector>
#include <cmath>
#include <iomanip>
#include <fstream>
#include "OEM7Reader.h"
#include "SolverLSQ.h"
#include "CoordConvert.h"
#include "TimeConvert.h"
#include "Const.h"

using namespace std;

// 无电离层组合计算 (P_IF = (f1^2*P1 - f2^2*P2) / (f1^2 - f2^2))
double calcIF(double p1, double p2, double f1, double f2) {
    double f1_sq = f1 * f1;
    double f2_sq = f2 * f2;
    return (f1_sq * p1 - f2_sq * p2) / (f1_sq - f2_sq);
}

// 简单的对流层延迟模型
double tropDelay(double elev_deg) {
    if (elev_deg < 5.0) elev_deg = 5.0;
    double sinE = sin(elev_deg * DEG_TO_RAD);
    // 经验值：干延迟约 2.31m，湿延迟约 0.1m
    return 2.41 / sinE; 
}

int main() {
    const std::string filename = "NovatelOEM20211114-01.log";
    OEM7Reader oem7;
    if (oem7.open(filename)) {
        cerr << "Failed to open file: " << filename << endl;
        return -1;
    }

    // 初始化解算输出文件
    ofstream solFile("solution.txt");
    if (!solFile) {
        cerr << "Failed to create solution.txt" << endl;
        return -1;
    }
    solFile << "Epoch,X,Y,Z,VX,VY,VZ,Clk_m,ClkDrift_ms" << endl;

    map<SatID, std::unique_ptr<Ephemeris>> lastEph;
    Vector3d recPos = Vector3d::Zero(); 
    double recClk = 0.0;
    Vector3d recVel = Vector3d::Zero();
    double recClkDrift = 0.0;
    SolverLSQ solver;
    bool initialPosFound = false;

    cout << fixed << setprecision(3);
    cout << "Starting SPP/SPV processing (Single Pass Mode)..." << endl;

    while (true) {
        try {
            // 单次读取
            auto eph_ptr = oem7.readOne();
            
            if (eph_ptr) {
                SatID sat;
                sat.system = string(1, eph_ptr->type);
                sat.id = eph_ptr->PRN;
                lastEph[sat] = std::move(eph_ptr);
                continue;
            }

            if (!oem7.hasObs) continue;
            
            ObsData obs = oem7.lastObs;
            oem7.hasObs = false;

            if (lastEph.empty()) continue;

            // SPP 迭代解算
            if (!initialPosFound) {
                recPos.setZero();
                recClk = 0.0;
            }

            for (int iter = 0; iter < 10; ++iter) {
                EquSys equSys;
                equSys.station = "Rover";
                
                Variable vDX("Rover", Parameter::dX);
                Variable vDY("Rover", Parameter::dY);
                Variable vDZ("Rover", Parameter::dZ);
                Variable vCDT("Rover", Parameter::cdt);
                
                equSys.varSet.insert({vDX, vDY, vDZ, vCDT});

                for (auto const& [sat, typeVal] : obs.satTypeValueData) {
                    if (lastEph.find(sat) == lastEph.end()) continue;
                    
                    double psr = 0;
                    if (sat.system == "G") {
                        double f1 = getFreq("G", 1);
                        double f2 = getFreq("G", 2);
                        if (typeVal.count("C1") && typeVal.count("C2")) {
                            psr = calcIF(typeVal.at("C1"), typeVal.at("C2"), f1, f2);
                        } else if (typeVal.count("C1")) {
                            psr = typeVal.at("C1");
                        }
                    } else if (sat.system == "C") {
                        double f1 = getFreq("C", 2); // B1I
                        double f2 = getFreq("C", 6); // B3I
                        if (typeVal.count("C2") && typeVal.count("C6")) {
                            psr = calcIF(typeVal.at("C2"), typeVal.at("C6"), f1, f2);
                        } else if (typeVal.count("C2")) {
                            psr = typeVal.at("C2");
                        }
                    }
                    
                    if (psr < 1e6) continue;

                    double tau = psr / C_MPS;
                    PVT pvt = lastEph[sat]->svPVT(obs.epoch - tau);
                    
                    Vector3d satPos = pvt.p;
                    // Sagnac 效应修正
                    double ang = OMEGA_EARTH * tau;
                    satPos = Vector3d(pvt.p[0]*cos(ang) + pvt.p[1]*sin(ang), 
                                      -pvt.p[0]*sin(ang) + pvt.p[1]*cos(ang), 
                                      pvt.p[2]);

                    double rho = (satPos - recPos).norm();
                    double elev = initialPosFound ? elevation(recPos, satPos) : 30.0;
                    
                    if (elev < 10.0 && initialPosFound) continue;
                    
                    double trop = initialPosFound ? tropDelay(elev) : 0;
                    double prefit = psr - (rho - C_MPS * pvt.clkbias + C_MPS * recClk + trop);
                    
                    EquData ed;
                    ed.prefit = prefit;
                    ed.weight = initialPosFound ? (pow(sin(elev * DEG_TO_RAD), 2)) : 1.0;
                    
                    ed.varCoeffData[vDX] = (recPos[0] - satPos[0]) / (rho > 0 ? rho : 1.0);
                    ed.varCoeffData[vDY] = (recPos[1] - satPos[1]) / (rho > 0 ? rho : 1.0);
                    ed.varCoeffData[vDZ] = (recPos[2] - satPos[2]) / (rho > 0 ? rho : 1.0);
                    ed.varCoeffData[vCDT] = 1.0; 
                    
                    equSys.obsEquData[EquID(sat, "C_IF")] = ed;
                }

                if (equSys.obsEquData.size() < 4) break;

                solver.solve(equSys);
                recPos += solver.getdxyz();
                recClk += solver.getSolution(Parameter::cdt) / C_MPS;

                if (solver.getdxyz().norm() < 1e-3) {
                    initialPosFound = true;
                    break;
                }
            }
            
            if (!initialPosFound) continue;

            // SPV 测速
            EquSys equSysV;
            equSysV.station = "Rover";
            Variable vDVX("Rover", Parameter::dVX);
            Variable vDVY("Rover", Parameter::dVY);
            Variable vDVZ("Rover", Parameter::dVZ);
            Variable vCDTR_DOT("Rover", Parameter::cdtr_dot);
            
            equSysV.varSet.insert({vDVX, vDVY, vDVZ, vCDTR_DOT});

            for (auto const& [sat, typeVal] : obs.satTypeValueData) {
                if (lastEph.find(sat) == lastEph.end()) continue;
                
                string dopKey = "";
                int freqId = -1;
                if (sat.system == "G") {
                    if (typeVal.count("D1")) { dopKey = "D1"; freqId = 1; }
                    else if (typeVal.count("D2")) { dopKey = "D2"; freqId = 2; }
                } else if (sat.system == "C") {
                    if (typeVal.count("D2")) { dopKey = "D2"; freqId = 2; }
                    else if (typeVal.count("D6")) { dopKey = "D6"; freqId = 6; }
                }
                
                if (dopKey == "") continue;
                
                double doppler = typeVal.at(dopKey);
                double lambda = C_MPS / getFreq(sat.system, freqId);
                PVT pvt = lastEph[sat]->svPVT(obs.epoch - 0.07);
                
                Vector3d e = (pvt.p - recPos).normalized();
                double prefit = -doppler * lambda - ((pvt.v - recVel).dot(e) + C_MPS * recClkDrift - C_MPS * pvt.clkdrift);
                
                EquData ed;
                ed.prefit = prefit;
                ed.weight = 1.0; 
                ed.varCoeffData[vDVX] = -e[0];
                ed.varCoeffData[vDVY] = -e[1];
                ed.varCoeffData[vDVZ] = -e[2];
                ed.varCoeffData[vCDTR_DOT] = C_MPS;
                
                equSysV.obsEquData[EquID(sat, dopKey)] = ed;
            }

            if (equSysV.obsEquData.size() >= 4) {
                solver.solve(equSysV);
                recVel += solver.getdxyz();
                recClkDrift += solver.getSolution(Parameter::cdtr_dot) / C_MPS;
            }

            // 输出到文件和屏幕
            CivilTime ct = CommonTime2CivilTime(obs.epoch);
            solFile << ct.year << "/" << ct.month << "/" << ct.day << " " << ct.hour << ":" << ct.minute << ":" << ct.second << ","
                    << recPos.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlign, ",", ",", "", "", "", "")) << ","
                    << recVel.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlign, ",", ",", "", "", "", "")) << ","
                    << recClk << "," << recClkDrift << endl;

            cout << "Epoch: " << ct.hour << ":" << ct.minute << ":" << (int)ct.second 
                 << " Sats: " << obs.satTypeValueData.size() << " P: " << recPos.transpose() << endl;

        } catch (const EndOfFile &e) {
            cout << "End of file reached." << endl;
            break;
        } catch (const exception& e) {
            continue;
        }
    }

    solFile.close();
    return 0;
}
