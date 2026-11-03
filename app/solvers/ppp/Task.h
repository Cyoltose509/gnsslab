#pragma once

#include <memory>

#include "solvers/common/Core.h"

namespace Solvers::PPP {
    /// PPP 静态/动态解算：配置与结果在 core，界面在 Panel.cpp。
    struct Task : Solvers::Task {
        std::shared_ptr<Solvers::Core> core;

        bool kinematic = false;

        std::shared_ptr<Solvers::Core> solveResult() override { return core; }

        void applyConfig(const SolverOptions &cfg);

        void loadFromMemory();

        void renderConfigPanel();

        bool initSolver(void *solver) override;

        bool wantsKinematic() const override { return kinematic; }

        Task() {
            core = std::make_shared<Solvers::Core>();
            core->mode = Mode::PPP;
        }

    protected:
        ::SPP *makeSolver() override;
    };

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
