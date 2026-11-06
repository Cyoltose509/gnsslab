#pragma once

#include <memory>
#include <string>

#include "SPP.h"
#include "solvers/common/Config.h"
#include "solvers/common/Realtime.h"

namespace NovAtel {
    class SocketReader;
}

namespace Solvers::SPP {
    struct Task;
}

namespace Solvers::Realtime {
    class SPPSession final : public Session {
    public:
        SPPSession(std::shared_ptr<SPP::Task> task, RealtimeEndpoint endpoint);

        ~SPPSession() override;

    protected:
        [[nodiscard]] const char *tag() const noexcept override;

        [[nodiscard]] std::string target() const override;

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
        RealtimeEndpoint endpoint;
        std::unique_ptr<NovAtel::SocketReader> reader;
        ::SPP spp;
    };
}
