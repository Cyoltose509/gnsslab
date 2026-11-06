#include "Task.h"

#include <chrono>
#include <fstream>
#include <memory>
#include <queue>
#include <string>

#include "Log.h"
#include "novatel/LogReader.h"
#include "rinex/RINEXObsReader.h"
#include "solvers/common/Realtime.h"

namespace Solvers {
    namespace {
        constexpr int kMaxEpochs = 200000; // 1 Hz 超过 55 h，用于挡住解析器不抛 EOF 的死循环

        std::atomic<int> cfgSeq{0};

        /// RINEX 取帧：EOF 当正常读完，其余解析异常冒泡给调用方上报。
        struct RINEXQueueSource final : Observer::Source {
            RINEXObsReader &reader;
            int safety = 0;

            explicit RINEXQueueSource(RINEXObsReader &r) : reader(r) {
            }

            bool next(Observer::Item &out) override {
                if (++safety >= kMaxEpochs) return false;
                try {
                    out.obs = reader.parseRINEXObs();
                } catch (const EndOfFile &) {
                    return false;
                }
                out.eph = nullptr;
                return true;
            }
        };
    }

    int Task::nextCfgIndex() {
        return cfgSeq.fetch_add(1) + 1;
    }

    int Task::failRead(const std::string &msg) {
        {
            std::lock_guard lk(queueMutex);
            if (readDone.exchange(true)) return -1;
        }
        LOG_ERROR << msg;
        setError(msg);
        wakeQueues();
        return -1;
    }

    void Task::markNavReady() {
        {
            std::lock_guard lk(queueMutex);
            navReady = true;
        }
        queueCv.notify_all();
    }

    void Task::finishRead() {
        {
            std::lock_guard lk(queueMutex);
            readDone = true;
        }
        queueCv.notify_all();
        queueSpaceCv.notify_all();
    }

    int Task::feedToQueue(Observer::Source &src) {
        int ok = 0;
        while (!stop.load() && !epochLimitReached()) {
            Observer::Item item;
            if (!src.next(item)) break;
            view.pushQCInput(item.obs, item.obs.weekSecond.sow);
            view.markQcDirty();
            {
                std::unique_lock lk(queueMutex);
                queueSpaceCv.wait(lk, [this] {
                    return obsQueue.size() < queueDepth || stop.load() || solvingDone.load();
                });
                if (stop.load() || solvingDone.load()) break;
                obsQueue.push(std::move(item));
            }
            queueCv.notify_one();
            ++ok;
        }
        return ok;
    }

    int Task::readRINEXToQueue() {
        std::fstream obsFile(filePath.c_str(), std::ios::in);
        if (!obsFile) return failRead("无法打开 RINEX: " + filePath);
        const auto reader = Observer::makeRINEXReader(obsFile);
        markNavReady();

        RINEXQueueSource src(*reader);
        int ok = 0;
        try {
            ok = feedToQueue(src);
        } catch (const std::exception &e) {
            LOG_ERROR << "RINEX parse err (" << filePath << "): " << e.what();
            return failRead("RINEX parse err: " + std::string(e.what()));
        }
        if (src.safety >= kMaxEpochs) {
            LOG_WARN << "RINEX 历元数触及 " << kMaxEpochs << " 上限，后续历元未读取: " << filePath;
        }
        if (ok == 0) setError("RINEX 无历元: " + filePath + "（文件是否被取消/解压失败/选错文件？）");
        finishRead();
        return ok;
    }

    int Task::readNovAtelToQueue() {
        Observer::NovAtelFile src;
        if (!src.open(filePath)) return failRead("无法打开 NovAtel 日志: " + filePath);
        markNavReady();

        int ok = 0;
        try {
            ok = feedToQueue(src);
        } catch (const std::exception &e) {
            return failRead("NovAtel parse err: " + std::string(e.what()));
        }
        if (ok == 0) {
            LOG_ERROR << "NovAtel 日志无历元: " << filePath;
            setError("NovAtel 日志无历元");
        }
        hasNav = ok > 0;
        finishRead();
        return ok;
    }

    int Task::readObsToQueue() {
        return isRINEX ? readRINEXToQueue() : readNovAtelToQueue();
    }

    bool Task::ok() const {
        return !errorFlag();
    }

    void Task::wakeQueues() {
        queueCv.notify_all();
        queueSpaceCv.notify_all();
    }

    void Task::cancel() {
        {
            std::lock_guard lk(queueMutex);
            stop = true;
        }
        wakeQueues();
    }

    void Task::finishSolving() {
        {
            std::lock_guard lk(queueMutex);
            solvingDone = true;
        }
        wakeQueues();
    }

    bool Task::epochLimitReached() {
        if (maxEpochs <= 0) return false;
        std::lock_guard lk(mutex);
        return static_cast<int>(epochs.size()) >= maxEpochs;
    }

    bool Task::shouldStop() {
        if (stop.load() || solvingDone.load()) return true;
        if (realtimeSec > 0) {
            const auto elapsed = std::chrono::steady_clock::now() - tStart;
            if (elapsed >= std::chrono::seconds(realtimeSec)) return true;
        }
        return epochLimitReached();
    }

    void Task::waitDone() {
        if (worker.joinable()) worker.join();
    }

    void Task::fileRead() {
        try {
            LOG_INFO << modeLabel(mode) << " 读取文件: " << filePath;
            if (readObsToQueue() >= 0) view.endQCInput();
        } catch (const std::exception &e) {
            failRead(e.what());
        } catch (...) {
            failRead("读取发生未知异常");
        }
        loading = false;
    }

    void Task::readBody() {
        fileRead();
    }

    void Task::solveBody() {
        setError("任务未实现文件解算");
        finishSolving();
    }

    void Task::resetForSolve() {
        if (worker.joinable()) worker.join();

        stop = false;
        done = false;
        loading = true;
        state = State::Running;
        tStart = std::chrono::steady_clock::now();
        clearError();

        readDone = false;
        solvingDone = false;
        navReady = false;
        hasNav = false;
        noEphSolve = false;
        initializedRefECEF = false;
        {
            std::lock_guard lk(queueMutex);
            std::queue<Observer::Item> empty;
            std::swap(obsQueue, empty);
        }
        {
            std::lock_guard lk(mutex);
            epochs.clear();
        }

        {
            std::lock_guard lk(view.plotMutex);
            view.plotData.clear();
            view.plotSeries = PlotSeries{};
            view.plotSeries.revision = view.plotData.revision; // 复位后不触发「有新数据」
        }
        view.clearQCInput(opts.sampleInterval);
        view.qcReport.reset();
        view.qcReady = false;
        view.qcDirty = false;
        view.extraQcTabs.clear();
        view.skyTracks.clear();
        view.skyTracksBuilt = 0;
        view.leoTraj.clear();
        view.leoRefTraj.clear();
        view.rtnTimes.clear();
        view.rtnR.clear();
        view.rtnT.clear();
        view.rtnN.clear();
        view.rtnD3.clear();
        view.hasLeo = false;
        view.leoPos3d.setZero();
        view.leoRef3d.setZero();
        view.selectedEpoch = -1;
        view.selectedSatIdx = -1;

        onReset();
    }

    void Task::startFile() {
        resetForSolve();
        worker = std::thread([self = shared_from_this()] {
            std::thread thRead([self] { self->readBody(); });
            std::thread thSolve([self] { self->solveBody(); });
            thRead.join();
            thSolve.join();
            self->state = State::Done;
            self->done = true;
            self->loading = false;
        });
    }

    void Task::startRealtime() {
        resetForSolve();
        worker = std::thread([self = shared_from_this()] {
            if (auto session = self->makeSession()) session->run();
            self->view.endQCInput();
        });
    }

    void Task::exportCSV(std::ostream &) const {
    }
}
