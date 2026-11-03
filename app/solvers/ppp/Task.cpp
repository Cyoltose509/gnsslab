#include "Task.h"

#include <future>
#include <string>
#include <vector>

#include "Log.h"
#include "PPPStatic.h"
#include "PPPKinematic.h"
#include "SPP.h"
#include "StringUtils.h"
#include "Observer.h"
#include "io/ProductLoad.h"

namespace Solvers::PPP {
    namespace {
        template<typename PPPSolver>
        void loadProductsInto(Task &task, PPPSolver &ppp, std::vector<std::future<bool> > &jobs) {
            const auto products = ProductLoad::resolve(task.core->opts);
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
        const auto products = ProductLoad::resolve(core->opts);
        const bool srcOk = ProductLoad::requireSatellite("PPP", products.paths.sp3, products.paths.brdc);
        ProductLoad::warnTropoFallback("PPP", core->opts.spp.tropoMap, ppp->tropoGridReady());
        return srcOk ? true : ProductLoad::fail(core.get());
    }

    ::SPP *Task::makeSolver() {
        return kinematic ? static_cast<::SPP *>(new PPPKinematic) : static_cast<::SPP *>(new PPPStatic);
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
