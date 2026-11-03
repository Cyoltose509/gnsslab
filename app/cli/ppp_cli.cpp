#include <iostream>
#include <string>
#include <vector>


#include "solvers/common/Core.h"
#include "SPP.h"
#include "PPPStatic.h"
#include "PPPKinematic.h"
#include "Observer.h"
#include "Log.h"
#include "SolverOptions.h"
#include "StringUtils.h"
#include "CliCommon.h"
#include "solvers/common/Driver.h"
#include "io/ProductLoad.h"
#include "core/IniFile.h"

namespace {
    void runPpp(const SolverOptions &cfg) {
        std::vector<Solvers::Epoch> epochs;

        if (!ProductLoad::requireSatellite("PPP", cfg.sp3, cfg.navPath)) {
            Cli::dumpEmpty(Solvers::Mode::PPP, cfg);
            return;
        }

        auto init = [&](auto &ppp) -> bool {
            std::vector<std::future<bool> > jobs;
            const ProductLoad::Paths paths{cfg.sp3, cfg.clk, cfg.navPath, cfg.osb, cfg.att, cfg.erp, cfg.atx};
            ProductLoad::core(ppp, jobs, "PPP", paths, cfg.systems);
            ProductLoad::loadPPPExtras(ppp, jobs, cfg.tro, cfg.ionex, cfg.met, cfg.vmf, cfg.gpt3);
            for (auto &j: jobs) j.get();

            ppp.applyOptions(cfg);
            return true;
        };

        Eigen::Vector3d ref(0, 0, 0);

        Observer::RINEXFile rsrc;
        Observer::NovAtelFile osrc;
        Observer::Source *src = Cli::openObsSource(cfg.obsPath, "PPP", rsrc, osrc);
        if (!src) {
            Cli::dumpEmpty(Solvers::Mode::PPP, cfg);
            return;
        }

        if (cfg.kinematic) {
            PPPKinematic k;
            init(k);
            epochs = Solvers::Driver::solveAll(*src, k, cfg.sampleInterval);
        } else {
            PPPStatic s;
            init(s);
            epochs = Solvers::Driver::solveAll(*src, s, cfg.sampleInterval);
        }

        if (cfg.hasRef) ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
        else Solvers::Driver::firstSolvedRef(epochs, ref);
        Cli::dumpCSV(epochs, Solvers::Mode::PPP, ref, false, cfg);
    }
}

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
    runPpp(cfg);
    return 0;
}
