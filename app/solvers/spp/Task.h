#pragma once

#include <memory>

#include "solvers/common/Core.h"

namespace Solvers::SPP {
    /// SPP 文件/实时解算：配置与结果在 core，界面在 Panel.cpp。
    struct Task : Solvers::Task {
        std::shared_ptr<Solvers::Core> core;

        std::shared_ptr<Solvers::Core> solveResult() override { return core; }

        /// 按 SolverOptions 灌入配置（CLI 与 GUI 共用）。
        void applyConfig(const SolverOptions &cfg);

        /// 从 PathMemory 恢复上次界面配置（仅 GUI）。
        void loadFromMemory();

        /// 配置面板（仅 GUI，定义在 Panel.cpp）。
        void renderConfigPanel();

        bool initSolver(void *solver) override;

        void beforeSolveEpoch(void *solver, Observer::Item *item) override;

        Task() {
            core = std::make_shared<Solvers::Core>();
            core->mode = Mode::SPP;
        }

    protected:
        ::SPP *makeSolver() override;

        std::unique_ptr<Realtime::Session> makeSession() override;
    };

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
