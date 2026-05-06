#include <iostream>
#include <map>
#include <vector>
#include <iomanip>
#include <fstream>
#include "OEM7Reader.h"
#include "SPPIFCode.h"
#include "CoordConvert.h"
#include "Const.h"

using namespace std;

int main() {
    const string filename = "NovatelOEM20211114-01.log";
    OEM7Reader oem7;
    if (!oem7.open(filename)) {
        cout << "Failed to open file: " << filename << endl;
        return -1;
    }

    ofstream outfile("spp_results_v2.csv");
    if (!outfile.is_open()) {
        cout << "Failed to create output file." << endl;
        return -1;
    }

    outfile << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,SigmaP,SigmaV,SatCount" << endl;

    // Initialize SPPIFCode solver
    SPPIFCode spp;

    spp.setIFCodeTypes({
        {'G', {"C1", "C2"}},
        {'C', {"C2", "C6"}}
    });

    ObsData obs;
    int successCount = 0;

    while (oem7.getNextEpoch(obs)) {
        // Prepare ephemeris map for the current epoch
        std::map<SatID, Ephemeris *> ephMap;
        for (auto &[prn, eph]: oem7.latestGps) {
            ephMap[SatID('G', prn)] = &eph;
        }
        for (auto &[prn, eph]: oem7.latestBds) {
            ephMap[SatID('C', prn)] = &eph;
        }

        spp.setEphemeris(ephMap);

        try {
            spp.solve(obs);

            // If solve() didn't throw, we have a solution
            successCount++;
            auto &result = spp.result;


            // Output to CSV
            outfile << obs.weekSecond.week << ','
                    << fixed << setprecision(3) << obs.weekSecond.sow << ','
                    << fixed << setprecision(4)
                    << result.xyz[0] << ','
                    << result.xyz[1] << ','
                    << result.xyz[2] << ','
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
        } catch (const std::exception &e) {
            // Skip epochs where solution failed
            //cout << "Epoch skip: " << e.what() << endl;
        }
    }

    cout << "Finished processing. Results saved to spp_results_v2.csv" << endl;
    outfile.close();
    return 0;
}
