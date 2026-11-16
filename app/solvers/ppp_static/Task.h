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

        [[nodiscard]] std::vector<Export::Col> exportFields(Export::Format fmt) const override;

        [[nodiscard]] std::vector<Export::Col> exportDefaults(Export::Format fmt) const override;

        void writeCSVParams(CSV::Meta &m) const override;

    public:
        virtual void writeTropoParams(CSV::Meta &m) const;

        std::unique_ptr<::SPP> makeSolver() override;

        /// PPP 不做实时
    };

    inline SolverOptions defaultOptions() {
        SolverOptions o{};
        o.posProcSigma = 0.0;
        return o;
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
