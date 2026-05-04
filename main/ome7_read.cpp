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
    }

    const auto ct = CivilTime2CommonTime(CivilTime(2021, 11, 14, 7, 25, 0.00001788));


    return 0;
}
