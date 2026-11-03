#pragma once

#include <atomic>
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

        virtual void renderConfigPanel() = 0;

        Mode mode();

        void startFile();

        void startRealtime();

        void writeCsv(std::ostream &out);

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

        virtual bool shouldStop() const;

        /// 实时会话工厂；不做实时的模式不覆盖。
        virtual std::unique_ptr<Realtime::Session> makeSession() { return nullptr; }

    private:
        void fileRead();

        mutable std::mutex errorMutex;
        std::atomic<bool> hasError{false};
        std::string errorMsg;
    };
}
