#define COM_NO_WINDOWS_H
#define _HAS_STD_BYTE 0

#include "Realtime.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "Decimator.h"
#include "DualPair.h"
#include "EphemerisTable.h"
#include "Log.h"
#include "novatel/SocketReader.h"
#include "RTK.h"
#include "SolverOptions.h"
#include "solvers/common/Core.h"
#include "StringUtils.h"
#include "Task.h"

namespace Solvers::Realtime {
    namespace {
        using PollStream = std::function<int(ObsData &, std::shared_ptr<EphemerisTable> &)>;
        using OnPair = std::function<bool(const ObsData &, const ObsData &, EphemerisTable *)>;

        /// 双流配对泵：两路取数 → DualPair 配对 → onPair。返回 false 表示流断开或长时间无数据。
        bool pumpStreams(Observer::DualPair &pairing,
                         const PollStream &getBase, const PollStream &getRover,
                         const std::function<bool()> &shouldStop,
                         const std::function<bool()> &onAttempt, const OnPair &onPair) {
            bool haveBase = false, haveRover = false;
            ObsData base, rover;
            std::shared_ptr<EphemerisTable> baseEph, roverEph;

            constexpr auto kIdleEpochMs = std::chrono::milliseconds(10000);
            bool basePolling = false, roverPolling = false;
            std::chrono::steady_clock::time_point basePollSince{}, roverPollSince{};

            while (!shouldStop()) {
                if (!haveBase) {
                    if (!basePolling) {
                        basePolling = true;
                        basePollSince = std::chrono::steady_clock::now();
                    }
                    const int r = getBase(base, baseEph);
                    if (r < 0) return false;
                    if (r > 0) {
                        pairing.observeBase(base.weekSecond.sow);
                        haveBase = true;
                        basePolling = false;
                    } else if (std::chrono::steady_clock::now() - basePollSince > kIdleEpochMs) {
                        return false;
                    }
                }
                if (!haveRover) {
                    if (!roverPolling) {
                        roverPolling = true;
                        roverPollSince = std::chrono::steady_clock::now();
                    }
                    const int r = getRover(rover, roverEph);
                    if (r < 0) return false;
                    if (r > 0) {
                        pairing.observeRover(rover.weekSecond.sow);
                        haveRover = true;
                        roverPolling = false;
                    } else if (std::chrono::steady_clock::now() - roverPollSince > kIdleEpochMs) {
                        return false;
                    }
                }
                if (!haveBase || !haveRover) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    continue;
                }
                if (!onAttempt()) return true;
                const double tol = pairing.pairWindow();
                if (const auto act = Observer::DualPair::classify(base.epoch, rover.epoch, tol);
                    act == Observer::DualPair::Action::Pair) {
                    if (!onPair(base, rover, Observer::DualPair::selectEph(baseEph, roverEph))) return true;
                    haveBase = haveRover = false;
                } else if (act == Observer::DualPair::Action::AdvanceBase) {
                    haveBase = false;
                } else {
                    haveRover = false;
                }
            }
            return true;
        }
    }

    RTKSession::RTKSession(std::shared_ptr<RTK::Task> task,
                           RealtimeEndpoint base, RealtimeEndpoint rover)
        : task(std::move(task)), baseEp(std::move(base)), roverEp(std::move(rover)) {
        SolverOptions opts = this->task->opts;
        this->task->rtk.applyOptions(opts);
        this->task->rtk.mBaseXyzUser = this->task->useBaseCoord
                                           ? this->task->baseXyz
                                           : Eigen::Vector3d::Zero();

        this->task->view.clearQCInput(this->task->opts.sampleInterval);
        this->task->view.qcTabLabel = "质量分析[流动站]";

        baseTab = std::make_shared<QcTab>();
        baseTab->label = "质量分析[基准站]";
        baseTab->fileName = Str::baseName(this->task->rtEndpoints[0].ip);
        baseTab->view.clearQCInput(this->task->opts.sampleInterval);
        {
            std::lock_guard lk(this->task->mutex);
            this->task->view.extraQcTabs.clear();
            this->task->view.extraQcTabs.push_back(baseTab);
        }
    }

    RTKSession::~RTKSession() = default;

    const char *RTKSession::tag() const noexcept { return "RTK实时"; }

    std::string RTKSession::target() const {
        return "base=" + baseEp.ip + ":" + baseEp.port + " rover=" + roverEp.ip + ":" + roverEp.port;
    }

    void RTKSession::clearError() { task->clearError(); }

    void RTKSession::setConnectError() {
        task->setError("正在尝试连接 base " + baseEp.ip + ":" + baseEp.port
                       + " / rover " + roverEp.ip + ":" + roverEp.port + " ...");
    }

    void RTKSession::setStreamBroken() {
        task->setError("实时数据流中断（连接关闭或长时间无历元），正在重连 ...");
    }

    void RTKSession::onException(const std::exception &e) {
        task->setError(std::string("RTK实时运行异常: ") + e.what());
    }

    void RTKSession::finish() {
        task->loading = false;
        task->done = true;
        if (baseTab) baseTab->view.endQCInput();
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

    bool RTKSession::stopped() { return task->shouldStop(); }

    bool RTKSession::pump() {
        Observer::DualPair pairing;
        pairing.setInterval(task->opts.sampleInterval);
        const Observer::Decimator dec(task->opts.sampleInterval);

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

        return pumpStreams(pairing, getBase, getRover,
                           [&] { return stopped(); },
                           [&] {
                               if (stopped()) return false;
                               const auto now = std::chrono::steady_clock::now();
                               if (const auto idle = now - lastCommit;
                                   idle > kStallWarn && now - lastStallWarn > std::chrono::seconds(10)) {
                                   lastStallWarn = now;
                                   stallWarned = true;
                                   const auto sec = std::chrono::duration_cast<std::chrono::seconds>(idle).count();
                                   const std::string msg = "已 " + std::to_string(sec) + " s 无新解（最后提交历元 SOW "
                                                           + std::to_string(lastCommitSow) + "），数据在流但未产出解算结果";
                                   LOG_WARN << "[RTK实时] " << msg;
                                   task->setError("实时流" + msg);
                               }
                               return true;
                           },
                           [&](const ObsData &b, const ObsData &r, EphemerisTable *eph) -> bool {
                               task->view.pushQCInput(r, r.weekSecond.sow);
                               baseTab->view.pushQCInput(b, b.weekSecond.sow);
                               task->view.markQcDirty();
                               baseTab->view.markQcDirty();
                               if (!dec.accept(r.weekSecond.sow)) return true;
                               if (task->solvePair(b, r, *eph)) {
                                   lastCommit = std::chrono::steady_clock::now();
                                   lastCommitSow = static_cast<long long>(r.weekSecond.sow);
                                   if (stallWarned) {
                                       stallWarned = false;
                                       LOG_INFO << "[RTK实时] 恢复产出新解，最后提交历元 SOW " << lastCommitSow;
                                       task->clearError();
                                   }
                               }
                               return true;
                           });
    }
}
