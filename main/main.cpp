#include <sstream>
#include<cmath>
#include <string>
#include  <fstream>
#include"OEM7Reader.h"


int main() {
    const std::string filename = "NovatelOEM20211114-01.log";
    std::ofstream outFile("output.txt");
    if (!outFile.is_open()) {
        throw InvalidRequest("Could not open output file.");
        return 1;
    }



    const auto ct = CivilTime2CommonTime(CivilTime(2021, 11, 14, 7, 25, 0.00001788));

    OEM7Reader oem7;
    oem7.open(filename);
    for (int i = 1; i < 100; i++) {
        auto data = oem7.readOne();
        if (data) {
            PVT pvt = data->svPVT(ct);
            outFile << data->name() << " "
                    << std::fixed << std::setprecision(6)
                    << pvt.p[0] * 0.001 << " " << pvt.p[1] * 0.001 << " " << pvt.p[2] * 0.001 << " "
                    << std::fixed << std::setprecision(3)
                    << pvt.v[0] << " " << pvt.v[1] << " " << pvt.v[2] << " "
                    << std::fixed << std::setprecision(6)
                    << pvt.clkbias * 1e6 << " " << pvt.clkdrift * 1e6 << endl;
        }
    }

    return 0;
}
