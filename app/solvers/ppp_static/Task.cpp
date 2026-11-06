#include "Task.h"

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "Observer.h"
#include "PPPStatic.h"
#include "SPP.h"
#include "StringUtils.h"
#include "io/ProductLoad.h"
#include "solvers/common/CSV.h"

namespace Solvers::PPPStatic {
    void Task::applyConfig(const SolverOptions &cfg) {
        opts = cfg;
        filePath = cfg.obsPath;
        fileName = Str::baseName(cfg.obsPath);
        isRINEX = Observer::isRINEX(Observer::classifyObsPath(cfg.obsPath));
    }

    bool Task::initSolver(::SPP &solver) {
        std::vector<std::future<bool> > jobs;
        auto &ppp = static_cast<::PPPStatic &>(solver);
        const auto products = ProductLoad::resolve(opts, modeKey(mode));
        ProductLoad::core(ppp, jobs, "PPP", products.paths, opts.systems);
        ProductLoad::loadPPPExtras(ppp, jobs, products);
        for (auto &j: jobs) j.get();
        const bool srcOk = ProductLoad::requireSatellite("PPP", products.paths.sp3, products.paths.brdc);
        ProductLoad::warnTropoFallback("PPP", opts.spp.tropoMap, ppp.tropoGridReady());
        return srcOk ? true : ProductLoad::fail(this);
    }

    std::unique_ptr<::SPP> Task::makeSolver() {
        return std::make_unique<::PPPStatic>();
    }

    std::unique_ptr<Realtime::Session> Task::makeSession() {
        return nullptr;
    }

    void Task::exportCSV(std::ostream &out) const {
        CSV::Writer w;
        w.row("Wk", "SOW", "Status",
              "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m",
              "Float-3dRMS/m", "Float-SigmaP/m",
              "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m",
              "Fix-3dRMS/m", "Fix-SigmaP/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "SatCount",
              "REF-X/m", "REF-Y/m", "REF-Z/m", "Vel-X/m", "Vel-Y/m", "Vel-Z/m").endRow();
        const int cols = w.colCount();
        const Eigen::Vector3d &ref = refECEF;
        for (const auto &r: epochs) {
            if (!r.solved) {
                csvInvalidRow(w, r, cols);
                continue;
            }
            const auto &res = r.sppResult;
            const bool fixed = r.isFixed(mode);
            const Eigen::Vector3d fixE = fixed ? res.xyzFixed : Eigen::Vector3d::Zero();
            w.cell(r.week).cell(CSV::fx(r.sow, 3)).cell(csvStatStr(res.stat));
            csvWriteSolBlock(w, res.xyzFloat, ref, res.sigmaPFloat, false);
            csvWriteSolBlock(w, fixE, ref, res.sigmaPFixed, !fixed);
            csvDopCells(w, res).cell(res.ratio).cell(r.numSatsResult);
            w.cell(CSV::f(ref.x(), 11)).cell(CSV::f(ref.y(), 11)).cell(CSV::f(ref.z(), 11))
                    .cell(CSV::f(res.vel[0], 6)).cell(CSV::f(res.vel[1], 6)).cell(CSV::f(res.vel[2], 6));
            w.endRow();
        }
        out << w.str();
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
