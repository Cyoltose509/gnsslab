#include "Task.h"

#include <future>
#include <ostream>
#include <string>
#include <vector>

#include "Log.h"
#include "PPPStatic.h"
#include "PPPKinematic.h"
#include "SPP.h"
#include "StringUtils.h"
#include "Observer.h"
#include "io/ProductLoad.h"
#include "solvers/common/CSV.h"

namespace Solvers::PPP {
    namespace {
        template<typename PPPSolver>
        void loadProductsInto(Task &task, PPPSolver &ppp, std::vector<std::future<bool> > &jobs) {
            const auto products = ProductLoad::resolve(task.core->opts, modeKey(Mode::PPP));
            ProductLoad::core(ppp, jobs, "PPP", products.paths, task.core->enabledSystems);
            ProductLoad::loadPPPExtras(ppp, jobs, products);
        }
    }

    void Task::applyConfig(const SolverOptions &cfg) {
        core->mode = Mode::PPP;
        core->opts = cfg;
        core->filePath = cfg.obsPath;
        core->fileName = Str::baseName(cfg.obsPath);
        core->cutoffDeg = cfg.cutoffDeg;
        core->enabledSystems = cfg.systems;
        core->sampleInterval = cfg.sampleInterval;
        core->isRINEX = Observer::isRINEX(Observer::classifyObsPath(cfg.obsPath));
        core->view.obsPathBuf = cfg.obsPath;
        kinematic = cfg.kinematic != 0;
    }

    bool Task::initSolver(void *solver) {
        std::vector<std::future<bool> > jobs;
        const ::SPP *ppp = nullptr;
        const auto loadInto = [&](auto *p) {
            loadProductsInto(*this, *p, jobs);
            ppp = p;
        };
        if (kinematic) loadInto(static_cast<PPPKinematic *>(solver));
        else loadInto(static_cast<PPPStatic *>(solver));
        for (auto &j: jobs) j.get();
        const auto products = ProductLoad::resolve(core->opts, modeKey(Mode::PPP));
        const bool srcOk = ProductLoad::requireSatellite("PPP", products.paths.sp3, products.paths.brdc);
        ProductLoad::warnTropoFallback("PPP", core->opts.spp.tropoMap, ppp->tropoGridReady());
        return srcOk ? true : ProductLoad::fail(core.get());
    }

    ::SPP *Task::makeSolver() {
        return kinematic ? static_cast<::SPP *>(new PPPKinematic) : static_cast<::SPP *>(new PPPStatic);
    }

    void Task::exportCSV(std::ostream &out) const {
        const Eigen::Vector3d &ref = core->refECEF;
        CSV::Writer w;
        w.row("Wk", "SOW", "Status",
              "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m",
              "Float-3dRMS/m", "Float-SigmaP/m",
              "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m",
              "Fix-3dRMS/m", "Fix-SigmaP/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "SatCount",
              "REF-X/m", "REF-Y/m", "REF-Z/m", "Vel-X/m", "Vel-Y/m", "Vel-Z/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: core->epochs) {
            if (!r.solved) {
                csvInvalidRow(w, r, cols);
                continue;
            }
            const auto &res = r.sppResult;
            const bool fixed = res.stat == 1;
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
