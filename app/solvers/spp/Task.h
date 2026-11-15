#pragma once

#include <memory>

#include "solvers/common/Core.h"
#include "solver/Filter.h"
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

        /// 界面里单独填的 BRDC 优先于产品池里的广播星历
        std::string navPath() const override;

        /// 前向趟与倒序趟都由 src 解算层的它编排，本类只备历元序列、给逐历元回填回调
        Filter filter;

        /// 按 SolverOptions 灌入配置
        virtual void applyConfig(const SolverOptions &cfg);

        /// 从 PathMemory 恢复上次界面配置
        virtual void loadFromMemory();

        /// 配置面板
        void renderConfigPanel() override;

        virtual bool initSolver(::SPP &solver);

        virtual void beforeSolveEpoch(::SPP &solver, Observer::Item &item);

        virtual bool afterSolveEpoch(::SPP &solver, Epoch &data, Observer::Item &item, int index);

        [[nodiscard]] std::vector<Export::Col> exportFields(Export::Format fmt) const override;

        [[nodiscard]] std::vector<Export::Col> exportDefaults(Export::Format fmt) const override;

        void writeCSVParams(CSV::Meta &m) const override;

        Task();

        ~Task();

    public:
        void solveBody() override;

        /// 前向趟：建惰性历元序列与会话，逐历元解算并回填；会话留给倒序趟按索引重放。
        void runForward();

        /// 倒序趟：另建解算器从末历元重放，与正序解按位置协方差融合后写回原槽位。
        void runBackward();

        void onReset() override;

        virtual std::unique_ptr<::SPP> makeSolver();

        std::unique_ptr<Realtime::Session> makeSession() override;

    private:
        /// 前向趟的解算器：倒序趟要从它接管精密产品，活到两趟都跑完。
        std::unique_ptr<::SPP> fwdSolver;
        /// 前向趟建好的会话（含惰性历元序列），倒序趟复用同一份。
        Filter::Session fwdSession;
    };

    /// SPP 的参数默认值：广播星历解算用 Hopfield 对流层模型；
    /// 滤波档默认开常速度模型（加速度 σ 与钟差随机游走），否则选了 forward/backward 也只是逐个历元重解。
    inline SolverOptions defaultOptions() {
        SolverOptions o{};
        o.spp.tropoMap = Troposphere::Map::Hopfield;
        o.accProcSigma = 0.01;
        o.spp.clockProcSigma = 0.3;
        return o;
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
