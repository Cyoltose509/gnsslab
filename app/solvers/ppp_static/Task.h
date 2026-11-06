#pragma once

#include <memory>

#include "solvers/common/Core.h"
#include "solvers/spp/Task.h"

namespace Solvers::PPPStatic {
    struct Task : SPP::Task {
        Task() { mode = Mode::PPPStatic; }

        void applyConfig(const SolverOptions &cfg) override;

        void loadFromMemory() override;

        void renderConfigPanel() override;

        bool initSolver(::SPP &solver) override;

        /// PPP 结果表（float/fix 双块）
        void exportCSV(std::ostream &out) const override;

    protected:
        std::unique_ptr<::SPP> makeSolver() override;

        /// PPP 不做实时
        std::unique_ptr<Realtime::Session> makeSession() override;
    };

    inline SolverOptions defaultOptions() {
        SolverOptions o{};
        o.posNoise = 0.0;
        return o;
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
