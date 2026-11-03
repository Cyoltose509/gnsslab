#pragma once


#include <string>
#include <memory>

#include "solvers/common/Core.h"
#include "view/GuiHelpers.h"
#include "view/GuiParamPage.h"
#include "core/PathMemory.h"
#include "core/Products.h"

namespace Solvers::PPP {
    struct Task : Solvers::Task {
        std::string obsPathBuf;

        bool kinematic = false;

        std::shared_ptr<Solvers::Core> core;

        std::shared_ptr<Solvers::Core> solveResult() override { return core; }

        void renderConfigPanel() override;

        bool initSolver(void *solver) override;

        bool wantsKinematic() const override { return kinematic; }

    protected:
        ::SPP *makeSolver() override;

    public:
        Task() {
            core = std::make_shared<Solvers::Core>();
            const auto &pm = AppConfig::PathMemory::instance();
            obsPathBuf = pm.get("ppp_obs");
            kinematic = pm.get("ppp_kinematic") == "1";
            Helpers::loadCommonCfg("ppp", Solvers::Mode::PPP, &core->opts,
                                   &core->cutoffDeg, &core->sampleInterval, &core->enabledSystems);
            AppConfig::Products::instance().load();
        }
    };
}
