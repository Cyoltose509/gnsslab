#include <iostream>
#include <string>
#include <vector>


#include "SolverTask.h"
#include "SPP.h"
#include "PPPStatic.h"
#include "PPPKinematic.h"
#include "ObsFormat.h"
#include "Log.h"
#include "SolverOptions.h"
#include "CliCommon.h"
#include "SolveDriver.h"
#include "core/IniFile.h"

namespace {
    void runPpp(const SolverOptions &cfg) {
        std::vector<GnssTask::EpochData> epochs;

        auto init = [&](auto &ppp) -> bool {
            if (!cfg.sp3.empty()) ppp.loadSp3(cfg.sp3);
            if (!cfg.clk.empty()) ppp.loadClk(cfg.clk, cfg.systems);
            if (!cfg.navPath.empty()) ppp.loadBrdc(cfg.navPath);
            if (!cfg.atx.empty()) ppp.loadAtx(cfg.atx);
            if (!cfg.osb.empty()) ppp.loadOsb(cfg.osb);
            if (!cfg.erp.empty()) ppp.loadErp(cfg.erp);
            if (!cfg.tro.empty()) ppp.loadTro(cfg.tro);
            if (!cfg.ionex.empty()) ppp.loadIonex(cfg.ionex);

            // 统一参数注入：PPPStatic::applyOptions 逐字还原原先的 ~21 项手写设置。
            ppp.applyOptions(cfg);
            return true;
        };

        Eigen::Vector3d ref(0, 0, 0);

        if (cfg.obsPath.empty()) {
            std::cerr << "PPP 需要观测文件 [input] obs\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Ppp, cfg);
            return;
        }
        SolveDriver::RinexFileSource rsrc;
        SolveDriver::Oem7FileSource osrc;
        SolveDriver::ObsSource *src = nullptr;
        if (const auto fmt = GnssTask::classifyObsPath(cfg.obsPath); fmt == GnssTask::ObsFormat::Oem7) {
            if (!osrc.open(cfg.obsPath)) {
                std::cerr << "PPP 无法打开 OEM7: " << cfg.obsPath << "\n";
                Cli::dumpEmpty(GnssTask::SolverMode::Ppp, cfg);
                return;
            }
            src = &osrc;
        } else if (fmt == GnssTask::ObsFormat::Rinex) {
            if (!rsrc.open(cfg.obsPath)) {
                std::cerr << "PPP 无法打开 RINEX: " << cfg.obsPath << "\n";
                Cli::dumpEmpty(GnssTask::SolverMode::Ppp, cfg);
                return;
            }
            src = &rsrc;
        } else {
            std::cerr << "PPP 无法识别观测文件格式: "
                    << cfg.obsPath << "\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Ppp, cfg);
            return;
        }

        if (cfg.kinematic) {
            PPPKinematic k;
            init(k);
            epochs = SolveDriver::solveAll(*src, k, cfg.sampleInterval);
        } else {
            PPPStatic s;
            init(s);
            epochs = SolveDriver::solveAll(*src, s, cfg.sampleInterval);
        }

        if (cfg.hasRef) ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
        else SolveDriver::firstSolvedRef(epochs, ref);
        Cli::dumpCsv(epochs, GnssTask::SolverMode::Ppp, ref, false, cfg);
    }
}

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "ppp.ini";
    AppConfig::IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolverOptions cfg = loadConfig(f);
    Log::init("cli_gnsslab.log");
    std::cout << "[PPP] obs=" << cfg.obsPath
            << " 运动学=" << (cfg.kinematic ? "是" : "否") << "\n";
    runPpp(cfg);
    return 0;
}
