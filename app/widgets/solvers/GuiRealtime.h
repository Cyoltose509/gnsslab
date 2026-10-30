#pragma once

#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include "SolverTask.h"

class OEM7SocketReader;

namespace Solvers::SppProcessor {
    struct SppTask;
}

namespace Solvers::RtkProcessor {
    struct RtkTask;
}

namespace Solvers::Realtime {
    class Session {
    public:
        virtual ~Session() = default;

        void Run();

    protected:
        virtual const char *Tag() const noexcept = 0;
        virtual std::string Target() const = 0;

        virtual bool Connect() = 0;

        virtual void Close() noexcept = 0;

        virtual bool Pump() = 0;

        virtual bool Stopped() const = 0;

        virtual void ClearError() = 0;

        virtual void SetConnectError() = 0;

        virtual void SetStreamBroken() {
        }
        virtual void OnException(const std::exception &e) = 0;

        virtual void Finish() = 0;

        virtual int ConnectFailBackoffMs() const noexcept { return 2000; }
        virtual int StreamBreakBackoffMs() const noexcept { return 0; }

    private:
        static void Sleep(int ms);

        bool loggedConnectFail = false;
    };

    class SppSession final : public Session {
    public:
        SppSession(std::shared_ptr<SppProcessor::SppTask> task, GnssTask::RealtimeEndpoint endpoint);

        ~SppSession() override;

    protected:
        const char *Tag() const noexcept override;

        std::string Target() const override;

        bool Connect() override;

        void Close() noexcept override;

        bool Pump() override;

        bool Stopped() const override;

        void ClearError() override;

        void SetConnectError() override;

        void OnException(const std::exception &e) override;

        void Finish() override;

    private:
        std::shared_ptr<SppProcessor::SppTask> task;
        GnssTask::RealtimeEndpoint endpoint;
        std::unique_ptr<OEM7SocketReader> reader;
        SPP spp;
        std::chrono::steady_clock::time_point lastQcTime{};
    };

    class RtkSession final : public Session {
    public:
        RtkSession(std::shared_ptr<RtkProcessor::RtkTask> task,
                   GnssTask::RealtimeEndpoint base,
                   GnssTask::RealtimeEndpoint rover);

        ~RtkSession() override;

    protected:
        const char *Tag() const noexcept override;

        std::string Target() const override;

        bool Connect() override;

        void Close() noexcept override;

        bool Pump() override;

        bool Stopped() const override;

        void ClearError() override;

        void SetConnectError() override;

        void SetStreamBroken() override;

        void OnException(const std::exception &e) override;

        void Finish() override;

        int StreamBreakBackoffMs() const noexcept override;

    private:
        std::shared_ptr<RtkProcessor::RtkTask> task;
        GnssTask::RealtimeEndpoint baseEp, roverEp;
        std::unique_ptr<OEM7SocketReader> baseRdr, roverRdr;
        std::shared_ptr<GnssTask::SolveTask> baseQc;
    };

    void RunSpp(const std::shared_ptr<SppProcessor::SppTask> &task,
                const GnssTask::RealtimeEndpoint &endpoint);

    void RunRtk(const std::shared_ptr<RtkProcessor::RtkTask> &task,
                const GnssTask::RealtimeEndpoint &base,
                const GnssTask::RealtimeEndpoint &rover);
}
