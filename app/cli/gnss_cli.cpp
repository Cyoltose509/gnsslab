// 用法：gnss_cli.exe <ini 文件>；mode 取值 spp / ppp_static / ppp_kin / rtk / leo。

#include <iostream>
#include <memory>
#include <string>

#include "CliCommon.h"
#include "Log.h"
#include "SolverOptions.h"
#include "core/IniFile.h"
#include "solvers/leo/Task.h"
#include "solvers/ppp_kinematic/Task.h"
#include "solvers/ppp_static/Task.h"
#include "solvers/rtk/Task.h"
#include "solvers/spp/Task.h"

namespace {
    /// 按模式建任务。PPP 静态与动态是两个独立模式，各走各的解算器与参数。
    std::shared_ptr<Solvers::Task> makeTaskFor(const Solvers::Mode mode, const SolverOptions &cfg) {
        switch (mode) {
            case Solvers::Mode::SPP: return Solvers::SPP::makeTask(cfg);
            case Solvers::Mode::PPPStatic: return Solvers::PPPStatic::makeTask(cfg);
            case Solvers::Mode::PPPKinematic: return Solvers::PPPKinematic::makeTask(cfg);
            case Solvers::Mode::RTK: return Solvers::RTK::makeTask(cfg);
            case Solvers::Mode::LEO: return Solvers::LEO::makeTask(cfg);
        }
        return Solvers::SPP::makeTask(cfg);
    }

    void printSummary(const Solvers::Mode mode, const SolverOptions &cfg) {
        std::cout << "[" << Solvers::modeLabel(mode) << "] 模式=" << cfg.source
                  << " obs=" << cfg.obsPath;
        if (mode == Solvers::Mode::RTK)
            std::cout << " base=" << cfg.basePath << " rover=" << cfg.roverPath;
        if (mode == Solvers::Mode::LEO)
            std::cout << " 参考轨道=" << (cfg.refSp3.empty() ? "(无)" : cfg.refSp3);
        std::cout << " nav=" << (cfg.navPath.empty() ? "(无)" : cfg.navPath) << "\n";
    }
}

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "gnss.ini";
    IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const Solvers::Mode mode = modeFromKey(f.get("solver", "mode", "spp"));
    const SolverOptions cfg = loadConfig(f, mode);
    Log::init("cli_gnsslab.log");
    printSummary(mode, cfg);
    const auto task = makeTaskFor(mode, cfg);
    return Cli::runTask(*task, cfg);
}
