#define COM_NO_WINDOWS_H
#define _HAS_STD_BYTE 0

#include "Realtime.h"

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "Log.h"
#include "novatel/SocketReader.h"
#include "RTK.h"
#include "SolverOptions.h"
#include "solvers/common/Driver.h"
#include "solvers/common/Core.h"
#include "StringUtils.h"
#include "Task.h"

namespace Solvers::Realtime {
    RTKSession::RTKSession(std::shared_ptr<RTK::Task> task,
                           Solvers::RealtimeEndpoint base, Solvers::RealtimeEndpoint rover)
        : task(std::move(task)), baseEp(std::move(base)), roverEp(std::move(rover)) {
        SolverOptions opts = this->task->core->opts;
        opts.cutoffDeg = this->task->cutoffDeg;
        opts.systems = this->task->enabledSystems;
        opts.sampleInterval = this->task->sampleInterval;
        this->task->rtk.applyOptions(opts);
        this->task->rtk.mBaseXyzUser = this->task->useBaseCoord
                                           ? this->task->baseXyz
                                           : Eigen::Vector3d::Zero();

        {
            std::lock_guard lk(this->task->core->view.qcInputMutex);
            this->task->core->view.qcInput.clear();
        }
        this->task->core->view.qcTabLabel = "质量分析[流动站]";
        baseQc = std::make_shared<Solvers::Core>();
        baseQc->mode = Solvers::Mode::RTK;
        baseQc->isRealtime = true;
        baseQc->filePath = this->task->core->rtEndpoints[0].ip;
        baseQc->fileName = Str::baseName(this->task->core->rtEndpoints[0].ip);
        baseQc->sampleInterval = this->task->sampleInterval;
        {
            std::lock_guard lk(this->task->core->mutex);
            this->task->core->view.extraQcTabs.clear();
            this->task->core->view.extraQcTabs.emplace_back("质量分析[基准站]", baseQc);
        }
    }

    RTKSession::~RTKSession() = default;

    const char *RTKSession::tag() const noexcept { return "RTK实时"; }

    std::string RTKSession::target() const {
        return "base=" + baseEp.ip + ":" + baseEp.port + " rover=" + roverEp.ip + ":" + roverEp.port;
    }

    void RTKSession::clearError() { task->core->clearError(); }

    void RTKSession::setConnectError() {
        task->core->setError("正在尝试连接 base " + baseEp.ip + ":" + baseEp.port
                             + " / rover " + roverEp.ip + ":" + roverEp.port + " ...");
    }

    void RTKSession::setStreamBroken() {
        task->core->setError("实时数据流中断（连接关闭或长时间无历元），正在重连 ...");
    }

    void RTKSession::onException(const std::exception &e) {
        task->core->setError(std::string("RTK实时运行异常: ") + e.what());
    }

    void RTKSession::finish() {
        task->loading = false;
        task->done = true;
        task->core->loading = false;
        task->core->done = true;
    }

    int RTKSession::streamBreakBackoffMs() const noexcept { return 1000; }

    bool RTKSession::connect() {
        baseRdr = std::make_unique<NovAtel::SocketReader>();
        roverRdr = std::make_unique<NovAtel::SocketReader>();
        const auto bPort = static_cast<unsigned short>(Str::toInt(baseEp.port));
        const auto rPort = static_cast<unsigned short>(Str::toInt(roverEp.port));
        if (!baseRdr->connect(baseEp.ip, bPort) || !roverRdr->connect(roverEp.ip, rPort)) {
            close();
            return false;
        }
        baseRdr->setReceiveTimeout(100);
        roverRdr->setReceiveTimeout(100);
        return true;
    }

    void RTKSession::close() noexcept {
        try {
            if (baseRdr) baseRdr->close();
            if (roverRdr) roverRdr->close();
        } catch (...) {
        }
        baseRdr.reset();
        roverRdr.reset();
    }

    bool RTKSession::stopped() { return task->shouldStop() || task->core->stop.load(); }

    bool RTKSession::pump() {
        Observer::DualPair pairing;
        pairing.setInterval(task->sampleInterval);
        const EpochDecimator dec(task->sampleInterval);
        std::chrono::steady_clock::time_point lastQcTime;

        std::chrono::steady_clock::time_point lastCommit = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point lastStallWarn{};
        long long lastCommitSow = 0;
        bool stallWarned = false;
        constexpr auto kStallWarn = std::chrono::seconds(30);
        constexpr auto kStallReopen = std::chrono::seconds(120);
        auto stallExceeded = [&] {
            return std::chrono::steady_clock::now() - lastCommit > kStallReopen;
        };
        auto getBase = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> int {
            if (stallExceeded()) return -1;
            if (!baseRdr->getNextEpoch(o, false)) return baseRdr->isConnected() ? 0 : -1;
            e = baseRdr->ephSnapshot();
            return 1;
        };
        auto getRover = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> int {
            if (stallExceeded()) return -1;
            if (!roverRdr->getNextEpoch(o, false)) return roverRdr->isConnected() ? 0 : -1;
            e = roverRdr->ephSnapshot();
            return 1;
        };

        return RTK::pump(pairing, getBase, getRover,
                         [&] { return task->stop.load() || task->core->stop.load(); },
                         [&] {
                             if (task->stop.load() || task->core->stop.load()) return false;
                             const auto now = std::chrono::steady_clock::now();
                             if (const auto idle = now - lastCommit;
                                 idle > kStallWarn && now - lastStallWarn > std::chrono::seconds(10)) {
                                 lastStallWarn = now;
                                 stallWarned = true;
                                 const auto sec = std::chrono::duration_cast<std::chrono::seconds>(idle).count();
                                 const std::string msg = "已 " + std::to_string(sec) + " s 无新解（最后提交历元 SOW "
                                                         + std::to_string(lastCommitSow) + "），数据在流但未产出解算结果";
                                 LOG_WARN << "[RTK实时] " << msg;
                                 task->core->setError("实时流" + msg);
                             }
                             return true;
                         },
                         [&](const ObsData &b, const ObsData &r, EphemerisTable *eph) -> bool {
                             {
                                 std::lock_guard lk(task->core->view.qcInputMutex);
                                 pushQcWindow(task->core->view.qcInput, r, r.weekSecond.sow);
                             }
                             {
                                 std::lock_guard lk(baseQc->view.qcInputMutex);
                                 pushQcWindow(baseQc->view.qcInput, b, b.weekSecond.sow);
                             }
                             if (const auto now = std::chrono::steady_clock::now();
                                 now - lastQcTime > std::chrono::milliseconds(500)) {
                                 lastQcTime = now;
                                 task->core->view.qcDirty = true;
                                 baseQc->view.qcDirty = true;
                             }
                             if (!dec.accept(r.weekSecond.sow)) return true;
                             bool ok = false;
                             try {
                                 ok = task->rtk.processEpoch(b, r, *eph);
                             } catch (const std::exception &e) {
                                 LOG_ERROR << "[RTK实时] processEpoch 异常: " << e.what();
                                 ok = false;
                             } catch (...) {
                                 ok = false;
                             }

                             if (ok) {
                                 if (const Result &res = task->rtk.result; res.numSats > 0) {
                                     Solvers::Driver::commitRtk(task->core, task->rtk, r);
                                     lastCommit = std::chrono::steady_clock::now();
                                     lastCommitSow = static_cast<long long>(r.weekSecond.sow);
                                     if (stallWarned) {
                                         stallWarned = false;
                                         LOG_INFO << "[RTK实时] 恢复产出新解，最后提交历元 SOW " << lastCommitSow;
                                         task->core->clearError();
                                     }
                                 }
                             }
                             return true;
                         });
    }
}
