#pragma once

#include "SolverTask.h"
#include <string>
#include <memory>

namespace Solvers::RealtimeProcessor {

    struct ConnectionConfig {
        std::string ip = "47.114.134.129";
        int port = 7190;
    };

    void SolveRealtimeThread(const std::shared_ptr<GnssTask::SolveTask> &task, const ConnectionConfig &config);

}
