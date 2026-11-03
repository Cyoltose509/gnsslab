#include "Core.h"

#include <chrono>
#include <fstream>
#include <limits>
#include <string>

#include "CSV.h"
#include "Log.h"
#include "SPP.h"
#include "solvers/common/Driver.h"
#include "solvers/common/Session.h"
#include "view/GuiSolverPanel.h"

namespace Solvers {
    bool Task::errorFlag() const { return hasError.load(); }

    std::string Task::errorMessage() const {
        std::lock_guard lk(errorMutex);
        return errorMsg;
    }

    void Task::setError(std::string msg) {
        std::lock_guard lk(errorMutex);
        errorMsg = std::move(msg);
        hasError.store(true);
    }

    void Task::clearError() {
        std::lock_guard lk(errorMutex);
        errorMsg.clear();
        hasError.store(false);
    }

    void Task::syncErrorFrom(const Task &src) {
        bool h = false;
        std::string m;
        {
            std::lock_guard lk(src.errorMutex);
            h = src.hasError.load();
            m = src.errorMsg;
        }
        {
            std::lock_guard lk(errorMutex);
            errorMsg = std::move(m);
            hasError.store(h);
        }
    }

    void Task::beforeSolveEpoch(void *solver, Observer::Item *item) {
        if (!item) return;
        static_cast<::SPP *>(solver)->ephTable.mergeEmbedded(item->eph, item->obs.epoch);
    }

    bool Task::afterSolveEpoch(void *, Epoch *data, Observer::Item *, int) {
        if (!data->solved) return false;
        if (const auto core = solveResult()) {
            std::lock_guard lk(core->mutex);
            if (!core->initializedRefECEF) {
                core->refECEF = data->sppResult.xyz;
                core->initializedRefECEF = true;
            }
        }
        return true;
    }

    Mode Task::mode() {
        const auto core = solveResult();
        return core ? core->mode : Mode::SPP;
    }

    bool Task::shouldStop() const {
        return stop.load();
    }

    void Task::fileRead() {
        const auto core = solveResult();
        try {
            LOG_INFO << modeLabel(core->mode) << " 读取文件: " << core->filePath;
            if (const int ok = readObsToQueue(core, core->stop); ok >= 0) {
                bool hasQcInput = false;
                {
                    std::lock_guard lk(core->qcInputMutex);
                    hasQcInput = !core->qcInput.empty();
                }
                if (hasQcInput) Solvers::SolverPanel::launchQC(core);
                else core->qcReady = true;
            }
        } catch (const std::exception &e) {
            core->setError(e.what());
        }
        core->loading = false;
    }

    void Task::readBody() {
        fileRead();
    }

    namespace {
        struct TaskQueueSource : Observer::Source {
            std::shared_ptr<Core> core;

            explicit TaskQueueSource(std::shared_ptr<Core> c) : core(std::move(c)) {
            }

            bool next(Observer::Item &out) override {
                return nextObsQueue(core, out);
            }
        };
    }

    void Task::solveBody() {
        const auto core = solveResult();
        const std::unique_ptr<::SPP> solver(makeSolver());
        if (!solver) {
            core->setError("解算器未就绪");
            core->solvingDone = true;
            return;
        }
        const auto tSolve0 = std::chrono::steady_clock::now();
        try {
            if (!waitNavReady(core)) {
                LOG_WARN << "[解算] 任务被取消(stop 已置位)：解算线程未启动，0 历元";
                core->solvingDone = true;
                return;
            }

            std::string sys;
            for (const char c: core->enabledSystems) sys += c;
            LOG_INFO << "[解算] 开始: 截止高度角 " << core->cutoffDeg << "°, 系统="
                    << (sys.empty() ? std::string("全部") : sys)
                    << (core->sampleInterval > 0.0
                            ? ", 采样间隔 " + std::to_string(core->sampleInterval) + "s"
                            : ", 采样间隔按文件");
            SolverOptions opts = core->opts;
            opts.cutoffDeg = core->cutoffDeg;
            opts.systems = core->enabledSystems;
            opts.sampleInterval = core->sampleInterval;
            opts.kinematic = wantsKinematic();
            solver->applyOptions(opts);
            if (!initSolver(solver.get())) {
                LOG_ERROR << "[解算] 解算器初始化失败（产品加载返回 false）";
                core->solvingDone = true;
                return;
            }

            TaskQueueSource src(core);
            Driver::solveStream(src, *solver,
                                     [this, core](::SPP &s, Observer::Item &item, Epoch &data) {
                                         int idx = 0;
                                         {
                                             std::lock_guard lk(core->mutex);
                                             idx = static_cast<int>(core->epochs.size());
                                         }
                                         bool ok = false;
                                         try {
                                             ok = afterSolveEpoch(&s, &data, &item, idx);
                                         } catch (const std::exception &e) {
                                             LOG_ERROR << "[解算] 历元异常 (sod=" << item.obs.epoch.m_sod << "): " << e.what();
                                         } catch (...) {
                                             LOG_ERROR << "[解算] 历元未知异常 (sod=" << item.obs.epoch.m_sod << ")";
                                         }
                                         if (!ok) data.solved = false;
                                         commitEpoch(core, std::move(data));
                                         return true;
                                     },
                                     [this, core](::SPP &s, Observer::Item &item) {
                                         beforeSolveEpoch(&s, &item);
                                     },
                                     core->sampleInterval);
        } catch (const std::exception &e) {
            LOG_ERROR << "[解算] 解算线程异常: " << e.what();
            core->setError(e.what());
        }
        std::size_t totalEpochs = 0;
        {
            std::lock_guard lk(core->mutex);
            totalEpochs = core->epochs.size();
        }
        LOG_INFO << "[解算] 结束: " << totalEpochs << " 历元, 耗时 "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - tSolve0).count() << " ms";
        core->solvingDone = true;
    }

    void Task::startFile() {
        worker = std::thread([self = shared_from_this()] {
            std::thread thRead([self] { self->readBody(); });
            std::thread thSolve([self] { self->solveBody(); });
            thRead.join();
            thSolve.join();
            self->state = State::Done;
            self->done = true;
            self->loading = false;
            syncTaskStatus(self, self->solveResult());
        });
    }

    void Task::startRealtime() {
        if (worker.joinable()) worker.join();
        stop = false;
        if (const auto core = solveResult()) {
            core->stop = false;
            core->loading = true;
        }
        state = State::Running;
        loading = true;
        worker = std::thread([self = shared_from_this()] {
            if (auto session = self->makeSession()) session->run();
        });
    }

    void Task::writeCsv(std::ostream &out) {
        if (const auto core = solveResult())
            exportCSV(out, core->epochs, core->mode, core->refECEF, core->hasRefOrbit);
    }

    void syncTaskStatus(const std::shared_ptr<Task> &task, const std::shared_ptr<Core> &core) {
        core->loading.store(task->loading.load());
        core->done.store(task->done.load());
        core->syncErrorFrom(*task);
    }
}
