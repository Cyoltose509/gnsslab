#include <iostream>
#include <string>

#include "CliCommon.h"
#include "Log.h"
#include "SolverOptions.h"
#include "core/IniFile.h"
#include "solvers/ppp/Task.h"

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "ppp.ini";
    IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolverOptions cfg = loadConfig(f, Solvers::Mode::PPP);
    Log::init("cli_gnsslab.log");
    std::cout << "[PPP] obs=" << cfg.obsPath
            << " 运动学=" << (cfg.kinematic ? "是" : "否") << "\n";
    const auto task = Solvers::PPP::makeTask(cfg);
    return Cli::runTask(*task, cfg);
}
