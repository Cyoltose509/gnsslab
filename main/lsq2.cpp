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

    outfile << "Wk,SOW,ECEF-X/m,ECEF-Y/m,ECEF-Z/m,numSats" << endl;

    // Initialize SPPIFCode solver
    SPPIFCode spp;

    spp.setIFCodeTypes({
        {'G',{"C1", "C2"}},
        {'C',{"C2", "C6"}}
    });

    ObsData obs;
    int successCount = 0;

    while (oem7.getNextEpoch(obs)) {
        // Prepare ephemeris map for the current epoch
        std::map<SatID, Ephemeris*> ephMap;
        for (auto & [prn, eph] : oem7.latestGps) {
            ephMap[SatID('G',prn)] = &eph;
        }
        for (auto & [prn, eph] : oem7.latestBds) {
            ephMap[SatID('C',prn)] = &eph;
        }

        spp.setEphemeris(ephMap);

        // Initial guess for position (can be center of earth if unknown)
        // Note: ObsData::antennaPosition is often (0,0,0) unless pre-set
        if (obs.antennaPosition.norm() < 1000.0) {
            obs.antennaPosition = Vector3d::Zero();
        }
        try {
            spp.solve(obs);

            // If solve() didn't throw, we have a solution
            successCount++;
            Vector3d solXyz = spp.getXYZ();

            // Output to CSV
            outfile << obs.weekSecond.week << ","
                    << fixed << setprecision(3) << obs.weekSecond.sow << ","
                    << fixed << setprecision(4)
                    << solXyz[0] << ","
                    << solXyz[1] << ","
                    << solXyz[2] << ","
                    << obs.satTypeValueData.size()
                    << endl;

        } catch (const std::exception& e) {
            // Skip epochs where solution failed
             cout << "Epoch skip: " << e.what() << endl;
        }
    }

    cout << "Finished processing. Results saved to spp_results_v2.csv" << endl;
    outfile.close();
    return 0;
}
