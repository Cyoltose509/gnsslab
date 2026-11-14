#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "solvers/leo/Antenna.h"
#include "solvers/common/Core.h"
#include "solvers/ppp_kinematic/Task.h"

namespace Solvers::LEO {
    struct Task : PPPKinematic::Task {
        std::vector<std::string> auxFiles;

        Antenna::Correction antenna;
        Antenna::Correction antennaDef;

        struct TextBox {
            char text[3][32] = {};
        };

        TextBox antennaArpText, antennaPcoText;

        std::string antennaObsPath;

        void applyConfig(const SolverOptions &cfg) override;

        void loadFromMemory() override;

        void renderConfigPanel() override;

        bool initSolver(::SPP &solver) override;

        bool afterSolveEpoch(::SPP &solver, Epoch &data, Observer::Item &item, int index) override;

        [[nodiscard]] std::vector<Export::Col> exportFields(Export::Format fmt) const override;

        [[nodiscard]] std::vector<Export::Col> exportDefaults(Export::Format fmt) const override;

        Task() { mode = Mode::LEO; }

    protected:
        std::unique_ptr<::SPP> makeSolver() override;

        /// LEO 不估对流层梯度与 ZTD，参数配置里没有这一块。
        void writeTropoParams(CSV::Meta &m) const override {
        }

        /// 运动检测两项插在周跳参数之前。
        void writeMotionParams(CSV::Meta &m) const override;
    };

    /// LEO 的参数默认值：动态滤波初值，关掉周跳与固体潮，运动阈值放宽到 100 m。
    inline SolverOptions defaultOptions() {
        SolverOptions o{};
        o.spp.phaseSigma = 0.003;
        o.spp.firstEpochPosSigma = 1.0;
        o.spp.ambProcSigma = std::sqrt(1.0E-7); // 旧方差口径默认值 1e-7 的精确开方（σ 口径）
        o.spp.minSatsForUpdate = 4;
        o.spp.useCycleSlip = false;
        o.spp.applySolidTide = false;
        o.motionThresh = 100.0;
        return o;
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
