#define COM_NO_WINDOWS_H
#define _HAS_STD_BYTE 0

#include "Realtime.h"

#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "EpochDecimator.h"
#include "Log.h"
#include "novatel/SocketReader.h"
#include "SolverOptions.h"
#include "solvers/common/Core.h"
#include "StringUtils.h"
#include "Task.h"

namespace Solvers::Realtime {
    SPPSession::SPPSession(std::shared_ptr<SPP::Task> task, Solvers::RealtimeEndpoint endpoint)
        : task(std::move(task)), endpoint(std::move(endpoint)) {
        this->task->core->initializedRefECEF = false;
    }

    SPPSession::~SPPSession() = default;

    const char *SPPSession::tag() const noexcept { return "实时"; }

    std::string SPPSession::target() const { return endpoint.ip + ":" + endpoint.port; }

    void SPPSession::clearError() { task->core->clearError(); }

    void SPPSession::setConnectError() { task->core->setError("正在尝试连接 " + target() + "..."); }

    void SPPSession::onException(const std::exception &e) {
        task->core->setError(std::string("运行异常: ") + e.what());
    }

    void SPPSession::finish() {
        task->loading = false;
        task->done = true;
        task->core->loading = false;
        task->core->done = true;
    }

    bool SPPSession::connect() {
        reader = std::make_unique<NovAtel::SocketReader>();
        if (const auto port = static_cast<unsigned short>(Str::toInt(endpoint.port)); !reader->connect(endpoint.ip, port)) {
            reader.reset();
            return false;
        }
        reader->setReceiveTimeout(200);
        return true;
    }

    void SPPSession::close() noexcept {
        try {
            if (reader) reader->close();
        } catch (...) {
        }
        reader.reset();
    }

    bool SPPSession::stopped() { return task->shouldStop() || task->core->stop.load(); }

    bool SPPSession::pump() {
        const EpochDecimator dec(task->core->sampleInterval);
        std::chrono::steady_clock::time_point lastQcTime;
        return SPP::pump(*reader, spp, dec,
                         [&] { return stopped(); },
                         [&](ObsData &obs, const EphemerisTable &eph, const bool accepted) {
                             {
                                 std::lock_guard lk(task->core->view.qcInputMutex);
                                 pushQcWindow(task->core->view.qcInput, obs, obs.weekSecond.sow);
                             }
                             if (!accepted) return;
                             Solvers::Epoch data;
                             data.getFromObs(obs);
                             if (!task->core->hasNav && !eph.empty()) task->core->hasNav = true;
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
                             Solvers::commitEpoch(task->core, std::move(data));
                             if (const auto now = std::chrono::steady_clock::now();
                                 now - lastQcTime > std::chrono::milliseconds(500)) {
                                 lastQcTime = now;
                                 task->core->view.qcDirty = true;
                             }
                         });
    }
}
