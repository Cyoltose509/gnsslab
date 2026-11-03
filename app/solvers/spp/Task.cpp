#include "Task.h"

#include <string>

#include "Log.h"
#include "Observer.h"
#include "SPP.h"
#include "StringUtils.h"
#include "io/ObsSource.h"
#include "io/ProductLoad.h"
#include "solvers/spp/Realtime.h"

namespace Solvers::SPP {
    void Task::applyConfig(const SolverOptions &cfg) {
        core->mode = Mode::SPP;
        core->opts = cfg;
        core->filePath = cfg.obsPath;
        core->fileName = Str::baseName(cfg.obsPath);
        core->brdcPathBuf = cfg.navPath;
        core->cutoffDeg = cfg.cutoffDeg;
        core->enabledSystems = cfg.systems;
        core->sampleInterval = cfg.sampleInterval;
        core->isRINEX = Observer::isRINEX(Observer::classifyObsPath(cfg.obsPath));
        core->view.obsPathBuf = cfg.obsPath;
        core->rtEndpoints.resize(1);
        core->rtEndpoints[0].ip = cfg.ip;
        core->rtEndpoints[0].port = std::to_string(cfg.port);
    }

    bool Task::initSolver(void *solver) {
        ::SPP &spp = *static_cast<::SPP *>(solver);
        std::vector<std::string> navs;
        if (!core->brdcPathBuf.empty()) navs.push_back(core->brdcPathBuf);
        else navs = ObsSource::scanNavFiles(core->filePath);
        for (const auto &nav: navs) {
            try {
                spp.loadBRDC(nav);
                core->hasNav = true;
                LOG_INFO << "SPP 读取广播星历: " << nav;
            } catch (const std::exception &e) {
                LOG_ERROR << "SPP 读取广播星历失败 " << nav << ": " << e.what();
            }
        }
        const auto products = ProductLoad::resolve(core->opts);
        ProductLoad::loadSPPGrids(spp, products.vmf, products.gpt3);
        ProductLoad::warnTropoFallback("SPP", core->opts.spp.tropoMap, spp.tropoGridReady());
        return true;
    }

    void Task::beforeSolveEpoch(void *solver, Observer::Item *item) {
        ::SPP &spp = *static_cast<::SPP *>(solver);
        const auto &[obs, eph] = *item;
        spp.ephTable.mergeEmbedded(eph, obs.epoch);
        if (!core->isRINEX && eph) core->hasNav = true;
        if (core->isRINEX && !core->hasNav) core->noEphSolve = true;
    }

    ::SPP *Task::makeSolver() {
        return new ::SPP;
    }

    std::unique_ptr<Realtime::Session> Task::makeSession() {
        return std::make_unique<Realtime::SPPSession>(
            std::static_pointer_cast<Task>(shared_from_this()), core->rtEndpoints[0]);
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
