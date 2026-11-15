#include "Realtime.h"

#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "Decimator.h"
#include "io/ProductPaths.h"
#include "io/ProductReader.h"
#include "Log.h"
#include "util/Clock.h"
#include "novatel/SocketReader.h"
#include "SolverOptions.h"
#include "solvers/common/Config.h"
#include "solvers/common/Core.h"
#include "StringUtils.h"
#include "Task.h"
#include "i18n/I18n.h"

namespace Solvers::Realtime {
    SPPSession::SPPSession(std::shared_ptr<SPP::Task> task, Solvers::RealtimeEndpoint endpoint)
        : task(std::move(task)), endpoint(std::move(endpoint)) {
        this->task->initializedRefECEF = false;
        this->task->view.clearQCInput(this->task->opts.sampleInterval);

        // 文件档由 Task::initSolver 灌选项，实时链路没人代劳：这里自己按同一份 opts 起解，
        // 否则滤波方向、截止高度角、系统集与对流层模型全部落到 SPPConfig 的默认值。
        SolverOptions opts = this->task->opts;
        if (const SolverOptions::FilterMode rt = filterForRealtime(opts.filter); rt != opts.filter) {
            LOG_WARN << "[SPP-RT] " << I18n::tr("log.rt.no_backward", filterLabel(opts.filter),
                                                         filterLabel(rt));
            opts.filter = rt;
        }
        spp.applyOptions(opts);
        const auto d = AppConfig::ProductPaths::of(modeKey(Mode::SPP)).resolved(opts);
        spp.prod = ProductReader::load(d, "SPP-RT", opts.systems,
                                     {ProductType::VMF, ProductType::GPT3})
                       .prod;
        Troposphere::warnFallback("SPP-RT", opts.spp.tropoMap, spp.tropoGridReady());
    }

    SPPSession::~SPPSession() = default;

    const char *SPPSession::tag() const noexcept { return "SPP-RT"; }

    std::string SPPSession::target() const { return endpoint.ip + ":" + endpoint.port; }

    void SPPSession::clearError() { task->clearError(); }

    void SPPSession::setConnectError() { task->setError(I18n::tr("rt.connecting", target())); }

    void SPPSession::onException(const std::exception &e) {
        task->setError(I18n::tr("rt.run_error", e.what()));
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
                    spp.prod->eph = *eph;
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
                } else if (!reader->isConnected() || reader->isStale(Clock::milliseconds(5000))) {
                    return false;
                } else {
                    Clock::sleep(Clock::milliseconds(50));
                }
            } catch (const std::exception &) {
                if (!reader->isConnected()) return false;
                Clock::sleep(Clock::milliseconds(100));
            }
        }
        return true;
    }
}
