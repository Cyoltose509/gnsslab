#pragma once

#include <memory>
#include <string>

#include "solvers/common/Core.h"
#include "view/GuiHelpers.h"
#include "view/GuiParamPage.h"
#include "core/PathMemory.h"

namespace Solvers::SPP {
    struct Task : Solvers::Task {
        std::shared_ptr<Solvers::Core> solveResult() override { return core; }

        void renderConfigPanel() override;

        bool initSolver(void *solver) override;

        void beforeSolveEpoch(void *solver, Observer::Item *item) override;

        std::shared_ptr<Solvers::Core> core;

    protected:
        ::SPP *makeSolver() override;

        std::unique_ptr<Realtime::Session> makeSession() override;

    public:
        Task() {
            core = std::make_shared<Solvers::Core>();
            const auto &pm = AppConfig::PathMemory::instance();
            core->obsPathBuf = pm.get("spp_obs");
            core->brdcPathBuf = pm.get("spp_brdc");
            core->rtEndpoints.resize(1);
            core->rtEndpoints[0].ip = pm.get("rt_ip", "47.114.134.129");
            core->rtEndpoints[0].port = pm.get("rt_port", "7190");
            Helpers::loadCommonCfg("spp", Solvers::Mode::SPP, &core->opts,
                                   &core->cutoffDeg, &core->sampleInterval, &core->enabledSystems);
        }
    };
}
