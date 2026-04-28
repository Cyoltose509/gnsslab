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

struct GnssConfig {
    map<string, pair<string, string>> dualCode;
    map<string, string> dopplerCode;
    GnssConfig() {
        dualCode["G"] = {"C1", "C2"}; 
        dualCode["C"] = {"C2", "C6"}; 
        dopplerCode["G"] = "D1";
        dopplerCode["C"] = "D2";
    }
} config;

double getTropDelay(double elev_deg) {
    if (elev_deg < 5.0) elev_deg = 5.0;
    return 2.41 / sin(elev_deg * DEG_TO_RAD); 
}

double getIFObs(const SatID& sat, const TypeValueMap& obs) {
    if (config.dualCode.count(sat.system)) {
        auto types = config.dualCode.at(sat.system);
        if (obs.count(types.first) && obs.count(types.second)) {
            double f1 = getFreq(sat.system, types.first);
            double f2 = getFreq(sat.system, types.second);
            if (f1 > 0 && f2 > 0)
                return (f1*f1 * obs.at(types.first) - f2*f2 * obs.at(types.second)) / (f1*f1 - f2*f2);
        }
        if (obs.count(types.first)) return obs.at(types.first);
    }
    return 0.0;
}

int main() {
    const std::string filename = "NovatelOEM20211114-01.log";
    OEM7Reader oem7;
    if (oem7.open(filename)) return -1;
    return 0;
}
