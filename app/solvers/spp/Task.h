#pragma once

#include <memory>

#include "solvers/common/Core.h"
#include "solvers/common/Task.h"

class SPP;

namespace Observer {
    struct Item;
}

namespace Solvers::SPP {
    /// SPP 文件/实时解算
    struct Task : Solvers::Task {
        /// 广播星历路径
        std::string navPathBuf;

        /// 按 SolverOptions 灌入配置
        virtual void applyConfig(const SolverOptions &cfg);

        /// 从 PathMemory 恢复上次界面配置
        virtual void loadFromMemory();

        /// 配置面板
        void renderConfigPanel() override;

        virtual bool initSolver(::SPP &solver);

        virtual void beforeSolveEpoch(::SPP &solver, Observer::Item &item);

        virtual bool afterSolveEpoch(::SPP &solver, Epoch &data, Observer::Item &item, int index);

        void exportCSV(std::ostream &out) const override;

        Task() { mode = Mode::SPP; }

    protected:
        void solveBody() override;

        virtual std::unique_ptr<::SPP> makeSolver();

        std::unique_ptr<Realtime::Session> makeSession() override;
    };

    /// SPP 的参数默认值：广播星历解算用 Hopfield 对流层模型。
    inline SolverOptions defaultOptions() {
        SolverOptions o{};
        o.spp.tropoMap = Troposphere::Map::Hopfield;
        return o;
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
