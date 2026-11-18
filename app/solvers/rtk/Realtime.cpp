#include "Realtime.h"
#include "FileUtils.h"

#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "Decimator.h"
#include "solver/rtk/DualPair.h"
#include "core/geom/EphemerisTable.h"
#include "Log.h"
#include "util/Clock.h"
#include "novatel/SocketReader.h"
#include "solver/rtk/RTK.h"
#include "SolverOptions.h"
#include "solvers/common/Core.h"
#include "StringUtils.h"
#include "Task.h"
#include "i18n/I18n.h"

namespace Solvers::Realtime {
    RTKSession::RTKSession(std::shared_ptr<RTK::Task> task,
                           RealtimeEndpoint base, RealtimeEndpoint rover)
        : task(std::move(task)), baseEp(std::move(base)), roverEp(std::move(rover)) {
        SolverOptions opts = this->task->opts;
        if (const SolverOptions::FilterMode rt = filterForRealtime(opts.filter); rt != opts.filter) {
            LOG_WARN << "[RTK-RT] " << I18n::tr("log.rt.no_backward", filterLabel(opts.filter),
                                                         filterLabel(rt));
            opts.filter = rt;
        }
        this->task->rtk.applyOptions(opts);
        this->task->rtk.mBaseUser = this->task->useBaseCoord
                                           ? this->task->baseXyz
                                           : Eigen::Vector3d::Zero();

        this->task->view.clearQCInput(this->task->opts.sampleInterval);
        this->task->view.qcTabKey = "solver.tab.qc_rover";

        baseTab = std::make_shared<QcTab>();
        baseTab->labelKey = "solver.tab.qc_base";
        baseTab->fileName = File::baseName(this->task->rtEndpoints[0].ip);
        baseTab->view.clearQCInput(this->task->opts.sampleInterval);
        {
            std::lock_guard lk(this->task->mutex);
            this->task->view.extraQcTabs.clear();
            this->task->view.extraQcTabs.push_back(baseTab);
        }
    }

    RTKSession::~RTKSession() = default;

    const char *RTKSession::tag() const noexcept { return "RTK-RT"; }

    std::string RTKSession::target() const {
        return "base=" + baseEp.ip + ":" + baseEp.port + " rover=" + roverEp.ip + ":" + roverEp.port;
    }

    void RTKSession::clearError() { task->clearError(); }

    void RTKSession::setConnectError() {
        task->setError(I18n::tr("rtk.rt.connecting", baseEp.ip + ":" + baseEp.port,
                                roverEp.ip + ":" + roverEp.port));
    }

    void RTKSession::setStreamBroken() {
        task->setError(I18n::tr("rtk.rt.stream_broken"));
    }

    void RTKSession::onException(const std::exception &e) {
        task->setError(I18n::tr("rtk.rt.run_error", e.what()));
    }

    void RTKSession::finish() {
        task->loading = false;
        task->done = true;
        if (baseTab) baseTab->view.endQCInput();
    }

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
        DualPair pairing;
        pairing.setInterval(task->opts.sampleInterval);
        const Observer::Decimator dec(task->opts.sampleInterval);

        constexpr auto kReadIdle = Clock::milliseconds(10000); // 单路这么久没历元算断流
        constexpr auto kStallWarn = Clock::seconds(30);        // 有历元但长时间没新解
        constexpr auto kStallWarnGap = Clock::seconds(10);
        constexpr auto kStallReconnect = Clock::seconds(120);

        auto lastCommit = Clock::now();
        auto lastStallWarn = Clock::Point{}; // 哨兵：尚未告过警，over() 对它恒真
        auto baseWaitSince = lastCommit, roverWaitSince = lastCommit;
        long long lastCommitSow = 0;
        bool stallWarned = false;

        ObsData base, rover;
        DualPair::EphPtr baseEph, roverEph;
        bool haveBase = false, haveRover = false;

        while (!stopped()) {
            if (!haveBase) {
                if (Clock::over(lastCommit, kStallReconnect)) return false;
                if (!baseRdr->getNextEpoch(base, false)) {
                    if (!baseRdr->isConnected() || Clock::over(baseWaitSince, kReadIdle)) return false;
                } else {
                    baseEph = baseRdr->ephSnapshot();
                    pairing.observeBase(base.weekSecond.sow);
                    haveBase = true;
                }
            }
            if (!haveRover) {
                if (Clock::over(lastCommit, kStallReconnect)) return false;
                if (!roverRdr->getNextEpoch(rover, false)) {
                    if (!roverRdr->isConnected() || Clock::over(roverWaitSince, kReadIdle)) return false;
                } else {
                    roverEph = roverRdr->ephSnapshot();
                    pairing.observeRover(rover.weekSecond.sow);
                    haveRover = true;
                }
            }
            if (!haveBase || !haveRover) {
                Clock::sleep(Clock::milliseconds(20));
                continue;
            }
            if (stopped()) return true;

            if (const auto idle = Clock::since(lastCommit); idle > kStallWarn && Clock::over(lastStallWarn, kStallWarnGap)) {
                lastStallWarn = Clock::now();
                stallWarned = true;
                LOG_WARN << "[RTK-RT] " << I18n::tr("rtk.rt.idle", idle.sec(), lastCommitSow);
                task->setError(I18n::tr("rtk.rt.idle_error", idle.sec(), lastCommitSow));
            }

            // 配对窗口按两路历元间隔估；对不齐时只推进落后的一侧。
            const auto act = DualPair::classify(base.epoch, rover.epoch, pairing.pairWindow());
            if (act == DualPair::Action::AdvanceBase) {
                haveBase = false;
                baseWaitSince = Clock::now();
                continue;
            }
            if (act == DualPair::Action::AdvanceRover) {
                haveRover = false;
                roverWaitSince = Clock::now();
                continue;
            }

            haveBase = haveRover = false;
            baseWaitSince = roverWaitSince = Clock::now();
            const auto eph = DualPair::selectEph(baseEph, roverEph);

            task->view.pushQCInput(rover, rover.weekSecond.sow);
            baseTab->view.pushQCInput(base, base.weekSecond.sow);
            task->view.markQcDirty();
            baseTab->view.markQcDirty();
            if (!dec.accept(rover.weekSecond.sow)) continue;

            // 单历元失败只提交空解，不是断流；只有成功历元才算一次 commit。
            if (task->solvePair(base, rover, *eph)) {
                lastCommit = Clock::now();
                lastCommitSow = static_cast<long long>(rover.weekSecond.sow);
                if (stallWarned) {
                    stallWarned = false;
                    LOG_INFO << "[RTK-RT] " << I18n::tr("rtk.rt.recovered", lastCommitSow);
                    task->clearError();
                }
            }
        }
        return true;
    }
}
