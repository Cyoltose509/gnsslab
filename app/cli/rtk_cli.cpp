#include <iostream>
#include <string>

#include "CliCommon.h"
#include "Log.h"
#include "SolverOptions.h"
#include "core/IniFile.h"
#include "solvers/rtk/Task.h"

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "rtk.ini";
    IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolverOptions cfg = loadConfig(f, Solvers::Mode::RTK);
    Log::init("cli_gnsslab.log");
    std::cout << "[RTK] 模式=" << cfg.source << " base=" << cfg.basePath
            << " rover=" << cfg.roverPath
            << " nav=" << (cfg.navPath.empty() ? "(无)" : cfg.navPath) << "\n";
    const auto task = Solvers::RTK::makeTask(cfg);
    return Cli::runTask(*task, cfg);
}
