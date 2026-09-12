#include "OsbBias.h"
#include <fstream>
#include <sstream>


bool OsbBias::read(const std::string &path) {
    std::ifstream in(path);
    if (!in) return false;

    std::string line;
    bool inSol = false;
    while (std::getline(in, line)) {
        // 去掉前导空白（BIA 数据行是缩进的）
        if (size_t p = line.find_first_not_of(" \t"); p != std::string::npos) line = line.substr(p);
        if (line.rfind("+BIAS/SOLUTION", 0) == 0) {
            inSol = true;
            continue;
        }
        if (line.rfind("-BIAS/SOLUTION", 0) == 0) {
            inSol = false;
            continue;
        }
        if (!inSol) continue;
        if (line.size() < 3) continue;
        if (line.rfind("OSB", 0) != 0) continue;

        std::istringstream ss(line);
        std::string tag, group, satStr, obs, t0, t1, unit;
        double val = 0.0, std = 0.0;
        ss >> tag >> group >> satStr >> obs >> t0 >> t1 >> unit >> val >> std;
        if (satStr.size() < 3) continue;
        SatID sat(satStr[0], std::stoi(satStr.substr(1, 2)));
        // 文件值单位是 ns；换算为米
        double meters = val * 1e-9 * C_MPS;
        data[{sat, obs}] = meters;
    }
    return !data.empty();
}

double OsbBias::getBias(const SatID &sat, const std::string &obsCode) const {
    const auto it = data.find({sat, obsCode});
    if (it == data.end()) return 0.0;
    return it->second;
}
