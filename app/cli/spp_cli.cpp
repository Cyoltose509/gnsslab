#include <iostream>
#include <string>

#include "CliCommon.h"
#include "Log.h"
#include "SolverOptions.h"
#include "core/IniFile.h"
#include "solvers/spp/Task.h"

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "spp.ini";
    IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolverOptions cfg = loadConfig(f, Solvers::Mode::SPP);
    Log::init("cli_gnsslab.log");
    std::cout << "[SPP] 模式=" << cfg.source << " obs=" << cfg.obsPath
            << " nav=" << (cfg.navPath.empty() ? "(伴生自动扫描)" : cfg.navPath) << "\n";
    const auto task = Solvers::SPP::makeTask(cfg);
    return Cli::runTask(*task, cfg);
}
