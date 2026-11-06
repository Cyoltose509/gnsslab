#pragma once

#include <memory>
#include <string>

#include "solvers/common/Config.h"
#include "solvers/common/Realtime.h"

namespace NovAtel {
    class SocketReader;
}

namespace Solvers {
    struct QcTab;
}

namespace Solvers::RTK {
    struct Task;
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
        std::shared_ptr<Solvers::QcTab> baseTab;
    };
}
