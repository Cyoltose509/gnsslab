#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "DualPair.h"
#include "EphemerisTable.h"
#include "solvers/common/Session.h"

namespace NovAtel {
class SocketReader;
}

namespace Solvers {
    struct Core;
}

namespace Solvers::RTK {
    struct Task;
}

namespace Solvers::RTK {
    /// RTK 实时双流配对泵：基准/流动两路取数 → DualPair 配对 → onPair 回调。
    /// 返回 false 表示流断开或长时间无数据，由调用方决定重连或退出。
    template<class GetBase, class GetRover, class OnAttempt, class OnPair>
    bool pump(Observer::DualPair &pairing,
              GetBase getBase, GetRover getRover,
              const std::function<bool()> &shouldStop,
              OnAttempt onAttempt, OnPair onPair) {
        bool haveBase = false, haveRover = false;
        ObsData base, rover;
        std::shared_ptr<EphemerisTable> baseEph, roverEph;

        constexpr auto kIdleEpochMs = std::chrono::milliseconds(10000);
        bool basePolling = false, roverPolling = false;
        std::chrono::steady_clock::time_point basePollSince{}, roverPollSince{};

        while (!shouldStop()) {
            if (!haveBase) {
                if (!basePolling) { basePolling = true; basePollSince = std::chrono::steady_clock::now(); }
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
                if (!roverPolling) { roverPolling = true; roverPollSince = std::chrono::steady_clock::now(); }
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
            if (const auto act = Observer::DualPair::classify(base.epoch, rover.epoch, tol); act == Observer::DualPair::Action::Pair) {
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

namespace Solvers::Realtime {
    class RTKSession final : public Session {
    public:
        RTKSession(std::shared_ptr<RTK::Task> task,
                   Solvers::RealtimeEndpoint base,
                   Solvers::RealtimeEndpoint rover);

        ~RTKSession() override;

    protected:
        const char *tag() const noexcept override;

        std::string target() const override;

        bool connect() override;

        void close() noexcept override;

        bool pump() override;

        bool stopped() override;

        void clearError() override;

        void setConnectError() override;

        void setStreamBroken() override;

        void onException(const std::exception &e) override;

        void finish() override;

        int streamBreakBackoffMs() const noexcept override;

    private:
        std::shared_ptr<RTK::Task> task;
        Solvers::RealtimeEndpoint baseEp, roverEp;
        std::unique_ptr<NovAtel::SocketReader> baseRdr, roverRdr;
        std::shared_ptr<Solvers::Core> baseQc;
    };
}
