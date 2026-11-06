#pragma once

#include <memory>

#include "solvers/common/Core.h"
#include "solvers/ppp_static/Task.h"

namespace Solvers::PPPKinematic {
    struct Task : PPPStatic::Task {
        Task() { mode = Mode::PPPKinematic; }

        void loadFromMemory() override;

        void renderConfigPanel() override;

    protected:
        std::unique_ptr<::SPP> makeSolver() override;
    };

    inline SolverOptions defaultOptions() {
        SolverOptions o{};
        o.posNoise = 100.0;
        o.spp.sigIFCode = 0.1;
        o.spp.sigmaPhase = 0.003;
        o.spp.ambNoise = 1.0E-7;
        o.spp.firstEpochPosVar = 1.0;
        o.spp.minSatsForUpdate = 4;
        return o;
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
