#include "Realtime.h"
#include "FileUtils.h"

#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "Decimator.h"
#include "DualPair.h"
#include "core/geom/EphemerisTable.h"
#include "Log.h"
#include "util/Clock.h"
#include "novatel/SocketReader.h"
#include "RTK.h"
#include "SolverOptions.h"
#include "solvers/common/Core.h"
#include "StringUtils.h"
#include "Task.h"

namespace Solvers::Realtime {
    RTKSession::RTKSession(std::shared_ptr<RTK::Task> task,
                           RealtimeEndpoint base, RealtimeEndpoint rover)
        : task(std::move(task)), baseEp(std::move(base)), roverEp(std::move(rover)) {
        SolverOptions opts = this->task->opts;
        if (const SolverOptions::FilterMode rt = filterForRealtime(opts.filter); rt != opts.filter) {
            LOG_WARN << "[RTK实时] 实时流没有倒序趟，滤波方向 " << filterLabel(opts.filter)
                    << " 按 " << filterLabel(rt) << " 处理";
            opts.filter = rt;
        }
        this->task->rtk.applyOptions(opts);
        this->task->rtk.mBaseXyzUser = this->task->useBaseCoord
                                           ? this->task->baseXyz
                                           : Eigen::Vector3d::Zero();

        this->task->view.clearQCInput(this->task->opts.sampleInterval);
        this->task->view.qcTabLabel = "质量分析[流动站]";

        baseTab = std::make_shared<QcTab>();
        baseTab->label = "质量分析[基准站]";
        baseTab->fileName = File::baseName(this->task->rtEndpoints[0].ip);
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
        Observer::DualPair::EphPtr baseEph, roverEph;
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
                const std::string msg = "已 " + std::to_string(static_cast<long long>(idle.sec()))
                                        + " s 无新解（最后提交历元 SOW "
                                        + std::to_string(lastCommitSow) + "），数据在流但未产出解算结果";
                LOG_WARN << "[RTK实时] " << msg;
                task->setError("实时流" + msg);
            }

            // 配对窗口按两路历元间隔估；对不齐时只推进落后的一侧。
            const auto act = Observer::DualPair::classify(base.epoch, rover.epoch, pairing.pairWindow());
            if (act == Observer::DualPair::Action::AdvanceBase) {
                haveBase = false;
                baseWaitSince = Clock::now();
                continue;
            }
            if (act == Observer::DualPair::Action::AdvanceRover) {
                haveRover = false;
                roverWaitSince = Clock::now();
                continue;
            }

            haveBase = haveRover = false;
            baseWaitSince = roverWaitSince = Clock::now();
            const auto eph = Observer::DualPair::selectEph(baseEph, roverEph);

            task->view.pushQCInput(rover, rover.weekSecond.sow);
            baseTab->view.pushQCInput(base, base.weekSecond.sow);
            task->view.markQcDirty();
            baseTab->view.markQcDirty();
            if (!dec.accept(rover.weekSecond.sow)) continue;

            if (!task->solvePair(base, rover, *eph)) return true;
            lastCommit = Clock::now();
            lastCommitSow = static_cast<long long>(rover.weekSecond.sow);
            if (stallWarned) {
                stallWarned = false;
                LOG_INFO << "[RTK实时] 恢复产出新解，最后提交历元 SOW " << lastCommitSow;
                task->clearError();
            }
        }
        return true;
    }
}
