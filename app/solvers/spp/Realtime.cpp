#define COM_NO_WINDOWS_H
#define _HAS_STD_BYTE 0

#include "Realtime.h"

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "Decimator.h"
#include "Log.h"
#include "novatel/SocketReader.h"
#include "SolverOptions.h"
#include "solvers/common/Core.h"
#include "StringUtils.h"
#include "Task.h"

namespace Solvers::Realtime {
    SPPSession::SPPSession(std::shared_ptr<SPP::Task> task, Solvers::RealtimeEndpoint endpoint)
        : task(std::move(task)), endpoint(std::move(endpoint)) {
        this->task->initializedRefECEF = false;
        this->task->view.clearQCInput(this->task->opts.sampleInterval);
    }

    SPPSession::~SPPSession() = default;

    const char *SPPSession::tag() const noexcept { return "实时"; }

    std::string SPPSession::target() const { return endpoint.ip + ":" + endpoint.port; }

    void SPPSession::clearError() { task->clearError(); }

    void SPPSession::setConnectError() { task->setError("正在尝试连接 " + target() + "..."); }

    void SPPSession::onException(const std::exception &e) {
        task->setError(std::string("运行异常: ") + e.what());
    }

    void SPPSession::finish() {
        task->loading = false;
        task->done = true;
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

    bool SPPSession::stopped() { return task->shouldStop(); }

    bool SPPSession::pump() {
        const Observer::Decimator dec(task->opts.sampleInterval);
        ObsData obs;
        while (!stopped()) {
            try {
                if (reader->getNextEpoch(obs, false)) {
                    const auto eph = reader->ephSnapshot();
                    spp.ephTable = *eph;
                    spp.preprocess(obs);
                    task->view.pushQCInput(obs, obs.weekSecond.sow);
                    task->view.markQcDirty();
                    if (!dec.accept(obs.weekSecond.sow)) continue;

                    Epoch data;
                    data.getFromObs(obs);
                    if (!task->hasNav && !eph->empty()) task->hasNav = true;
                    try {
                        spp.processEpoch(obs);
                        data.getFromSPP(spp);
                    } catch (...) {
                        data.solved = false;
                    }
                    task->commitEpoch(std::move(data), [&](int, const Epoch &stored) {
                        if (stored.solved && !task->initializedRefECEF) {
                            task->refECEF = stored.sppResult.xyz;
                            task->initializedRefECEF = true;
                        }
                    });
                } else if (!reader->isConnected() || reader->isStale(std::chrono::milliseconds(5000))) {
                    return false;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            } catch (const std::exception &) {
                if (!reader->isConnected()) return false;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        return true;
    }
}
