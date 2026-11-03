#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "SPP.h"
#include "EpochDecimator.h"
#include "solvers/common/Session.h"

namespace NovAtel {
class SocketReader;
}

namespace Solvers::SPP {
    struct Task;
}

namespace Solvers::SPP {
    /// SPP 实时内层泵：逐历元取数、更新星历快照并做前置处理。onEpoch 对每个收到的历元回调
    /// （accepted = 通过抽稀，obs 可被求解器就地改写）。返回 false 表示流断开或长时间无数据（5s），
    /// 由调用方决定重连或退出。
    template<class ShouldStop, class OnEpoch>
    bool pump(NovAtel::SocketReader &reader, ::SPP &spp, const EpochDecimator &dec,
              ShouldStop &&shouldStop, OnEpoch &&onEpoch) {
        ObsData obs;
        while (!shouldStop()) {
            try {
                if (reader.getNextEpoch(obs, false)) {
                    const auto eph = reader.ephSnapshot();
                    spp.ephTable = *eph;
                    spp.preprocess(obs);
                    onEpoch(obs, *eph, dec.accept(obs.weekSecond.sow));
                } else if (!reader.isConnected() || reader.isStale(std::chrono::milliseconds(5000))) {
                    return false;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            } catch (const std::exception &) {
                if (!reader.isConnected()) return false;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        return true;
    }
}

namespace Solvers::Realtime {
    class SPPSession final : public Session {
    public:
        SPPSession(std::shared_ptr<SPP::Task> task, Solvers::RealtimeEndpoint endpoint);

        ~SPPSession() override;

    protected:
        const char *tag() const noexcept override;

        std::string target() const override;

        bool connect() override;

        void close() noexcept override;

        bool pump() override;

        bool stopped() override;

        void clearError() override;

        void setConnectError() override;

        void onException(const std::exception &e) override;

        void finish() override;

    private:
        std::shared_ptr<SPP::Task> task;
        Solvers::RealtimeEndpoint endpoint;
        std::unique_ptr<NovAtel::SocketReader> reader;
        ::SPP spp;
        std::chrono::steady_clock::time_point lastQcTime{};
    };
}
