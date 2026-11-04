#pragma once

#include <atomic>
#include <chrono>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "Config.h"
#include "Epoch.h"
#include "Observer.h"
#include "Session.h"

namespace Solvers {
    struct Core;

    inline int nextCfgIndex() {
        static std::atomic seq{0};
        return seq.fetch_add(1) + 1;
    }

    /// 解算任务基类：线程机械与错误上报在基类，模式差异走虚钩子。
    /// 文件解算 startFile()（readBody/solveBody 双线程），实时 startRealtime()（makeSession 会话）。
    struct Task : std::enable_shared_from_this<Task> {
        enum class State { Config, Running, Done };

        /// 任务种类：求解任务走 Mode 体系；工具任务（标定/对准）没有求解模式，自备配置面板与结果页。
        enum class Kind { Solver, Tool };

        virtual Kind kind() const { return Kind::Solver; }

        std::atomic<State> state{State::Config};

        std::thread worker;
        std::atomic<bool> loading{false};
        std::atomic<bool> done{false};
        std::atomic<bool> stop{false};

        std::string fileName;
        bool isRealtime = false;

        bool errorFlag() const;

        std::string errorMessage() const;

        void setError(std::string msg);

        void clearError();

        void syncErrorFrom(const Task &src);

        virtual std::shared_ptr<Core> solveResult() = 0;

        Mode mode();

        void startFile();

        void startRealtime();

        /// 结果表格的表头与列顺序由各模式自己定；基类不导出（工具任务没有 CSV）。
        virtual void exportCSV(std::ostream &out) const;

        /// 等解算线程收尾（CLI 与 GUI 的 join 语义合一）。
        void waitDone();

        bool ok() const;

        /// 停止条件：stop 位、实时时限、历元上限。实时会话与文件解算共用。
        virtual bool shouldStop();

        int maxEpochs = 0;
        int realtimeSec = 0;

        virtual bool initSolver(void *solver) { return true; }

        virtual bool wantsKinematic() const { return false; }

        int cfgIndex = nextCfgIndex();

        virtual ~Task() {
            stop = true;
            if (worker.joinable()) worker.join();
        }

    protected:
        /// 单站解算器工厂；solveBody 默认实现消费，不走的模式（RTK）不覆盖。
        virtual ::SPP *makeSolver() { return nullptr; }

        /// 文件读取线程体；默认单站（读入 obsQueue + 质量输入），RTK 覆盖为双站配对管线。
        virtual void readBody();

        /// 解算线程体；默认经 makeSolver 逐历元出队解算，RTK 覆盖为 pairQueue 消费。
        virtual void solveBody();

        virtual void beforeSolveEpoch(void *solver, Observer::Item *item);

        virtual bool afterSolveEpoch(void *solver, Epoch *data, Observer::Item *item, int index);

        /// 实时会话工厂；不做实时的模式不覆盖。
        virtual std::unique_ptr<Realtime::Session> makeSession() { return nullptr; }

    private:
        void fileRead();

        std::chrono::steady_clock::time_point tStart = std::chrono::steady_clock::now();

        mutable std::mutex errorMutex;
        std::atomic<bool> hasError{false};
        std::string errorMsg;
    };
}
