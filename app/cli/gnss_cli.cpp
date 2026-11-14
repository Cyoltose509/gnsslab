#include <charconv>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "CliCommon.h"
#include "Log.h"
#include "SolverOptions.h"
#include "core/IniFile.h"
#include "download/local/Store.h"
#include "download/remote/Catalog.h"
#include "solvers/ins/Task.h"
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
            case Solvers::Mode::INS: return Solvers::INS::makeTask(cfg);
        }
        return Solvers::SPP::makeTask(cfg);
    }

    void printSummary(const Solvers::Mode mode, const SolverOptions &cfg) {
        std::cout << "[" << Solvers::modeLabel(mode) << "] 模式=" << cfg.source
                  << " obs=" << cfg.obsPath;
        if (mode == Solvers::Mode::RTK)
            std::cout << " base=" << cfg.basePath << " rover=" << cfg.roverPath;
        if (mode == Solvers::Mode::INS)
            std::cout << " imu=" << cfg.ins.imuPath;
        std::cout << " nav=" << (cfg.navPath.empty() ? "(无)" : cfg.navPath) << "\n";
    }

    bool collectArgs(const int argc, char **argv, std::string &iniPath, bool &iniGiven,
                     std::map<std::string, std::string> &over) {
        for (int i = 1; i < argc; ++i) {
            const std::string a = argv[i];
            if (a.rfind("--", 0) != 0) {
                if (!iniGiven) {
                    iniPath = a;
                    iniGiven = true;
                } else if (iniPath != a) {
                    std::cerr << "只接受一个 ini 路径，多余参数: " << a << "\n";
                    return false;
                }
                continue;
            }
            std::string key = a.substr(2), val;
            bool hasVal = false;
            if (const auto eq = key.find('='); eq != std::string::npos) {
                val = key.substr(eq + 1);
                key.resize(eq);
                hasVal = true;
            } else if (i + 1 < argc && argv[i + 1][0] != '-') {
                val = argv[++i];
                hasVal = true;
            }
            over[key] = hasVal ? val : "1"; // 布尔开关省值 = true
        }
        return true;
    }
}

int main(const int argc, char **argv) {
    std::string iniPath = "gnss.ini";
    std::map<std::string, std::string> over;
    if (bool iniGiven = false; !collectArgs(argc, argv, iniPath, iniGiven, over))
        return 2;

    IniFile f;
    if (!f.load(iniPath)) {
        std::cerr << "无法读取 ini: " << iniPath << "\n";
        return 1;
    }
    const CliSource src(f, std::move(over));

    const std::string modeKey = src.get("solver", "mode", "spp");
    const auto modeOpt = modeFromKey(modeKey);
    if (!modeOpt) {
        std::cerr << "[solver] mode = " << modeKey
                << " 不认识（spp | ppp_static | ppp_kin | rtk | leo | ins）\n";
        return 2;
    }
    const Solvers::Mode mode = *modeOpt;

    Cli::ExportOpt opt = Cli::loadExportOpt(src, mode);

    SolverOptions cfg = loadConfig(src, mode);
    // 没指定天线表时，用本地库里已装好的那一份（只读本地，不做任何联网下载）；都没有按无 PCO/PCV 处理。
    if (cfg.atx.empty())
        cfg.atx = Download::Store::activeFile(Download::Catalog::kATXID);
    for (const auto &k: src.unusedKeys())
        std::cerr << "警告: --" << k << " 没有被读取（拼写错误，或当前模式不用它）\n";

    Log::init("cli_gnsslab.log");
    printSummary(mode, cfg);
    const auto task = makeTaskFor(mode, cfg);
    return Cli::runTask(*task, cfg, opt);
}
