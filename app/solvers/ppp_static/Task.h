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

        /// 默认列就是 `exportFields` 全集；SPP 的默认集另有取舍，这个 override 必须盖住它。
        [[nodiscard]] std::vector<Export::Col> exportDefaults(Export::Format fmt) const override;

        void writeCsvParams(CSV::Meta &m) const override;

    protected:
        /// 对流层梯度/映射函数/ZTD 参数块：LEO 不估对流层，覆盖为空。
        virtual void writeTropoParams(CSV::Meta &m) const;

        /// 运动检测参数插槽：LEO 在周跳段之前插两项，PPP 静态/动态没有。
        virtual void writeMotionParams(CSV::Meta &m) const {}

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
