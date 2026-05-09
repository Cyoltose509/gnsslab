#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include <thread>
#include "OEM7SocketReader.h"
#include "SPPIFCode.h"
#include "CoordConvert.h"
#include <fstream>

int main() {
    std::string ip = "8.148.22.229";
    unsigned short port = 7003;

    std::cout << "Connecting to " << ip << ":" << port << "...\n";

    OEM7SocketReader reader;
    if (!reader.connect(ip, port)) {
        std::cerr << "Failed to connect to " << ip << ":" << port << "\n";
        return 1;
    }
    std::cout << "Connected successfully!\n";

    ofstream outfile("spp_realtime.csv");
    if (!outfile.is_open()) {
        cout << "Failed to create output file." << endl;
        return -1;
    }

    SPPIFCode spp;
    spp.setIFCodeTypes({
        {'G', {"C1", "C2"}},
        {'C', {"C2", "C6"}}
    });
    std::cout << "\n=== Real-time SPP Results ===\n";
    outfile <<
            "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,REF-ECEF-X/m,REF-ECEF-Y/m,REF-ECEF-Z/m,EAST/m,NORTH/m,UP/m,B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,SigmaP,SigmaV,SatCount"
            << endl;

    int epochCount = 0;
    ObsData obs;
    XYZ REF_ECEF = {-2267809.273, 5009323.033, 3221015.978};
    while (true) {
        try {
            if (reader.getNextEpoch(obs)) {

                // 更新星历
                std::map<SatID, Ephemeris *> ephMap;
                for (const auto &[prn, eph]: reader.latestGps) {
                    ephMap[SatID('G', prn)] = const_cast<GPSEphem *>(&eph);
                }
                for (const auto &[prn, eph]: reader.latestBds) {
                    ephMap[SatID('C', prn)] = const_cast<BDSEphem *>(&eph);
                }
                spp.setEphemeris(ephMap);
                spp.solve(obs);

                auto &result = spp.result;
                ENU enu = XYZtoENU(result.xyz, REF_ECEF);
                cout << obs.weekSecond.week << '\t'
                        << fixed << setprecision(3) << obs.weekSecond.sow << '\t'
                        << result.xyz[0] << '\t'
                        << result.xyz[1] << '\t'
                        << result.xyz[2] << '\t'
                        << result.vel[0] << '\t'
                        << result.vel[1] << '\t'
                        << result.vel[2] << '\t'
                        << result.pdop << '\t'
                        << result.sigmaP << '\t'
                        << result.sigmaV << endl;

                // Output to CSV
                outfile << obs.weekSecond.week << ','
                        << fixed << setprecision(3) << obs.weekSecond.sow << ','
                        << fixed << setprecision(4)
                        << result.xyz[0] << ','
                        << result.xyz[1] << ','
                        << result.xyz[2] << ','
                        << REF_ECEF.X() << ','
                        << REF_ECEF.Y() << ','
                        << REF_ECEF.Z() << ','
                        << enu.E() << ','
                        << enu.N() << ','
                        << enu.U() << ','
                        << fixed << setprecision(8)
                        << result.blh[0] * RAD_TO_DEG << ','
                        << result.blh[1] * RAD_TO_DEG << ','
                        << fixed << setprecision(3)
                        << result.blh[2] << ','
                        << result.vel[0] << ','
                        << result.vel[1] << ','
                        << result.vel[2] << ','
                        << result.pdop << ','
                        << result.sigmaP << ','
                        << result.sigmaV << ','
                        << result.numSats
                        << endl;
                epochCount++;

                // 更新初始位置用于下一历元
                spp.xyz = spp.result.xyz;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } catch (const std::exception &e) {
            std::cerr << "Error: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    reader.close();
    outfile.close();
    return 0;
}
