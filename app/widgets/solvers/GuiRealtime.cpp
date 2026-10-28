#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#define COM_NO_WINDOWS_H
#define _HAS_STD_BYTE 0

#include "GuiRealtime.h"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Const.h"
#include "Log.h"
#include "OEM7SocketReader.h"
#include "RTK.h"
#include "SolverOptions.h"
#include "SolveDriver.h"
#include "GuiHelpers.h"
#include "GuiRtkProcessor.h"
#include "GuiSolverPanel.h"
#include "GuiSppProcessor.h"

namespace Solvers::Realtime {
    namespace {
        constexpr size_t kQcWindow = 7200;

        void pushQcWindow(std::vector<QC::QCObsEpoch> &dst, const ObsData &epoch, const double sow) {
            dst.push_back(QC::makeQCObsEpoch(epoch, sow));
            if (dst.size() > kQcWindow)
                dst.erase(dst.begin(), dst.begin() + static_cast<std::ptrdiff_t>(dst.size() - kQcWindow));
        }
    }
    void Session::Run() {
        while (!Stopped()) {
            try {
                if (!Connect()) {
                    if (!loggedConnectFail) {
                        LOG_WARN << "[" << Tag() << "] 连接失败 " << Target()
                                << "，每 " << ConnectFailBackoffMs() / 1000 << "s 重试";
                        loggedConnectFail = true;
                    }
                    SetConnectError();
                    Sleep(ConnectFailBackoffMs());
                    continue;
                }
                ClearError();
                if (loggedConnectFail) {
                    LOG_INFO << "[" << Tag() << "] 已连接 " << Target();
                    loggedConnectFail = false;
                }

                const bool stopped = Pump();
                Close();
                if (stopped) break;   // Pump 说"该收工了"：Run 到此为止，绝不回头再 Connect（否则重连空转）

                if (!Stopped()) {
                    if (StreamBreakBackoffMs() > 0)
                        LOG_WARN << "[" << Tag() << "] 数据流中断，"
                                << StreamBreakBackoffMs() / 1000 << "s 后重连 " << Target();
                    SetStreamBroken();
                    Sleep(StreamBreakBackoffMs());
                }
            } catch (const std::exception &e) {
                Close();
                OnException(e);
                Sleep(ConnectFailBackoffMs());
            }
        }
        Finish();
    }

    void Session::Sleep(const int ms) {
        if (ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }


    SppSession::SppSession(std::shared_ptr<SppProcessor::SppTask> task, GnssTask::RealtimeEndpoint endpoint)
        : task(std::move(task)), endpoint(std::move(endpoint)) {
        this->task->core->initializedRefECEF = false;
        this->task->core->refAutoConfirmed = false;
    }

    SppSession::~SppSession() = default;

    const char *SppSession::Tag() const noexcept { return "实时"; }

    std::string SppSession::Target() const { return endpoint.ip + ":" + endpoint.port; }

    void SppSession::ClearError() { task->core->clearError(); }

    void SppSession::SetConnectError() { task->core->setError("正在尝试连接 " + Target() + "..."); }

    void SppSession::OnException(const std::exception &e) {
        task->core->setError(std::string("运行异常: ") + e.what());
    }

    void SppSession::Finish() {
        task->loading = false;
        task->done = true;
        task->core->loading = false;
        task->core->done = true;
    }

    bool SppSession::Connect() {
        reader = std::make_unique<OEM7SocketReader>();
        if (const auto port = static_cast<unsigned short>(std::atoi(endpoint.port.c_str())); !reader->connect(endpoint.ip, port)) {
            reader.reset();
            return false;
        }
        reader->setReceiveTimeout(200);
        return true;
    }

    void SppSession::Close() noexcept {
        try {
            if (reader) reader->close();
        } catch (...) {
        }
        reader.reset();
    }

    bool SppSession::Stopped() const { return task->stop.load() || task->core->stop.load(); }

    bool SppSession::Pump() {
        ObsData obs;
        while (!Stopped()) {
            try {
                if (reader->getNextEpoch(obs, false)) {   // 非阻塞：单流实时轮询
                    auto eph = reader->ephSnapshot(); // OEM7 内嵌星历统一经 ephTable 传入
                    spp.ephTable = *eph;
                    spp.preprocess(obs);
                    {
                        std::lock_guard lk(task->core->qcInputMutex);
                        pushQcWindow(task->core->qcInput, obs, obs.weekSecond.sow);
                    }

                    GnssTask::EpochData data;
                    data.getFromObs(obs);
                    if (!task->core->hasNav && !eph->empty()) task->core->hasNav = true;

                    try {
                        spp.processEpoch(obs);
                        data.getFromSPP(spp);
                        if (!task->core->initializedRefECEF) {
                            task->core->refECEF = data.sppResult.xyz;
                            task->core->initializedRefECEF = true;
                        }
                    } catch (...) {
                        data.solved = false;
                    }
                    GnssTask::CommitEpoch(task->core, std::move(data));

                    if (const auto now = std::chrono::steady_clock::now();
                        now - lastQcTime > std::chrono::milliseconds(500)) {
                        lastQcTime = now;
                        SolverPanel::LaunchQC(task->core);
                    }
                } else {
                    if (!reader->isConnected() || reader->isStale(std::chrono::milliseconds(5000)))
                        return false; // 跳出进行重连
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            } catch (const std::exception &) {
                if (!reader->isConnected()) return false;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        return true;
    }

    RtkSession::RtkSession(std::shared_ptr<RtkProcessor::RtkTask> task,
                           GnssTask::RealtimeEndpoint base, GnssTask::RealtimeEndpoint rover)
        : task(std::move(task)), baseEp(std::move(base)), roverEp(std::move(rover)) {
        // ⚠️ 构造体内必须走 this->task：形参 task 与成员同名且已被 move 成空指针，
        // 直接写 task-> 会解引用 nullptr（实测 0xC0000005）。
        SolverOptions opts;
        opts.cutoffDeg = this->task->cutoffDeg;
        opts.systems = this->task->enabledSystems;
        this->task->rtk.applyOptions(opts);
        this->task->rtk.mBaseXyzUser = this->task->useBaseCoord
                                           ? this->task->baseXyz
                                           : Eigen::Vector3d::Zero();

        {
            std::lock_guard lk(this->task->core->qcInputMutex);
            this->task->core->qcInput.clear();
        }
        this->task->core->qcTabLabel = "质量分析[流动站]";
        baseQc = std::make_shared<GnssTask::SolveTask>();
        baseQc->mode = GnssTask::SolverMode::Rtk;
        baseQc->isRealtime = true;
        baseQc->filePath = this->task->core->rtEndpoints[0].ip;
        baseQc->fileName = Helpers::baseName(this->task->core->rtEndpoints[0].ip);
        {
            std::lock_guard lk(this->task->core->mutex);
            this->task->core->extraQcTabs.clear();
            this->task->core->extraQcTabs.push_back({"质量分析[基准站]", baseQc});
        }
    }

    RtkSession::~RtkSession() = default;

    const char *RtkSession::Tag() const noexcept { return "RTK实时"; }

    std::string RtkSession::Target() const {
        return "base=" + baseEp.ip + ":" + baseEp.port + " rover=" + roverEp.ip + ":" + roverEp.port;
    }

    void RtkSession::ClearError() { task->core->clearError(); }

    void RtkSession::SetConnectError() {
        task->core->setError("正在尝试连接 base " + baseEp.ip + ":" + baseEp.port
                              + " / rover " + roverEp.ip + ":" + roverEp.port + " ...");
    }

    void RtkSession::SetStreamBroken() {
        task->core->setError("实时数据流中断（连接关闭或长时间无历元），正在重连 ...");
    }

    void RtkSession::OnException(const std::exception &e) {
        task->core->setError(std::string("RTK实时运行异常: ") + e.what());
    }

    void RtkSession::Finish() {
        task->loading = false;
        task->done = true;
        task->core->loading = false;
        task->core->done = true;
    }

    int RtkSession::StreamBreakBackoffMs() const noexcept { return 1000; }

    bool RtkSession::Connect() {
        baseRdr = std::make_unique<OEM7SocketReader>();
        roverRdr = std::make_unique<OEM7SocketReader>();
        const auto bPort = static_cast<unsigned short>(std::atoi(baseEp.port.c_str()));
        const auto rPort = static_cast<unsigned short>(std::atoi(roverEp.port.c_str()));
        if (!baseRdr->connect(baseEp.ip, bPort) || !roverRdr->connect(roverEp.ip, rPort)) {
            Close();
            return false;
        }
        baseRdr->setReceiveTimeout(100);
        roverRdr->setReceiveTimeout(100);
        return true;
    }

    void RtkSession::Close() noexcept {
        try {
            if (baseRdr) baseRdr->close();
            if (roverRdr) roverRdr->close();
        } catch (...) {
        }
        baseRdr.reset();
        roverRdr.reset();
    }

    bool RtkSession::Stopped() const { return task->stop.load() || task->core->stop.load(); }

    bool RtkSession::Pump() {
        SolveDriver::DualObsPairing pairing;
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
            if (!baseRdr->getNextEpoch(o, false)) return baseRdr->isConnected() ? 0 : -1;   // 非阻塞：与 rover 并发轮询
            e = baseRdr->ephSnapshot();
            return 1;
        };
        auto getRover = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> int {
            if (stallExceeded()) return -1;
            if (!roverRdr->getNextEpoch(o, false)) return roverRdr->isConnected() ? 0 : -1;   // 非阻塞：与 base 并发轮询
            e = roverRdr->ephSnapshot();
            return 1;
        };

        return SolveDriver::pumpRealtimePair(pairing, getBase, getRover,
            [&] { return task->stop.load() || task->core->stop.load(); },
            [&] {
                if (task->stop.load() || task->core->stop.load()) return false;
                const auto now = std::chrono::steady_clock::now();
                if (const auto idle = now - lastCommit; idle > kStallWarn && now - lastStallWarn > std::chrono::seconds(10)) {
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
                        SolveDriver::commitRtk(task->core, task->rtk, r);
                        lastCommit = std::chrono::steady_clock::now();
                        lastCommitSow = static_cast<long long>(r.weekSecond.sow);
                        if (stallWarned) {
                            stallWarned = false;
                            LOG_INFO << "[RTK实时] 恢复产出新解，最后提交历元 SOW " << lastCommitSow;
                            task->core->clearError();
                        }

                        {
                            std::lock_guard lk(task->core->qcInputMutex);
                            pushQcWindow(task->core->qcInput, r, r.weekSecond.sow);
                        }
                        {
                            std::lock_guard lk(baseQc->qcInputMutex);
                            pushQcWindow(baseQc->qcInput, b, b.weekSecond.sow);
                        }

                        if (const auto now = std::chrono::steady_clock::now();
                            now - lastQcTime > std::chrono::milliseconds(500)) {
                            lastQcTime = now;
                            SolverPanel::LaunchQC(task->core);
                            SolverPanel::LaunchQC(baseQc);
                        }
                    }
                }
                return true;
            });
    }

    void RunSpp(const std::shared_ptr<SppProcessor::SppTask> &task,
                const GnssTask::RealtimeEndpoint &endpoint) {
        SppSession session(task, endpoint);
        session.Run();
    }

    void RunRtk(const std::shared_ptr<RtkProcessor::RtkTask> &task,
                const GnssTask::RealtimeEndpoint &base,
                const GnssTask::RealtimeEndpoint &rover) {
        RtkSession session(task, base, rover);
        session.Run();
    }
}
