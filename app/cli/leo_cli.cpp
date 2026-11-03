#include <iostream>
#include <string>

#include "CliCommon.h"
#include "Log.h"
#include "SolverOptions.h"
#include "core/IniFile.h"
#include "solvers/leo/Task.h"

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "leo.ini";
    IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolverOptions cfg = loadConfig(f, Solvers::Mode::LEO);
    Log::init("cli_gnsslab.log");
    std::cout << "[LEO] obs=" << cfg.obsPath
            << " 参考轨道=" << (cfg.refSp3.empty() ? "(无)" : cfg.refSp3) << "\n";
    const auto task = Solvers::LEO::makeTask(cfg);
    return Cli::runTask(*task, cfg);
}
