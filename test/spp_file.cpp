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

    ofstream outfile("spp_results.csv");
    if (!outfile.is_open()) {
        cout << "Failed to create output file." << endl;
        return -1;
    }

    outfile <<
            "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,REF-ECEF-X/m,REF-ECEF-Y/m,REF-ECEF-Z/m,EAST/m,NORTH/m,UP/m,B/deg,L/deg,H/m,VX/m,VY/m,VZ/m,PDOP,SigmaP,SigmaV,SatCount"
            << endl;

    // Initialize SPPIFCode solver
    SPPIFCode spp;
    XYZ REF_ECEF = {
        -2267804.526, 5009342.372, 3220991.863
    };
    spp.setIFCodeTypes({
        {'G', {"C1", "C2"}},
        {'C', {"C2", "C6"}}
    });

    ObsData obs;
    int successCount = 0;

    while (oem7.getNextEpoch(obs)) {
        spp.preprocess(obs);
        try {
            spp.solve(obs);

            // If solve() didn't throw, we have a solution
            successCount++;
            auto &result = spp.result;
            ENU enu = XYZtoENU(result.xyz, REF_ECEF);

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
        } catch (const std::exception &e) {
            // Skip epochs where solution failed
            //cout << "Epoch skip: " << e.what() << endl;
        }
    }

    cout << "Finished processing. Results saved to spp_results_v2.csv" << endl;
    outfile.close();
    return 0;
}
