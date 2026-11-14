#pragma once

#include <cmath>
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
        o.posProcSigma = 10.0;
        o.spp.codeSigma = 0.1;
        o.spp.phaseSigma = 0.003;
        o.spp.ambProcSigma = std::sqrt(1.0E-7); // 旧方差口径默认值 1e-7 的精确开方（σ 口径）
        o.spp.firstEpochPosSigma = 1.0;
        o.spp.minSatsForUpdate = 4;
        return o;
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
