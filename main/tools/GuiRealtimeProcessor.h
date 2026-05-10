#pragma once

#include "GuiOem7Processor.h"
#include <string>
#include <memory>

namespace GuiRealtimeProcessor {
    // Reuse SppTask from GuiOem7Processor
    using SppTask = GuiOem7Processor::SppTask;

    struct ConnectionConfig {
        std::string ip = "8.148.22.229";
        int port = 7003;
    };

    void SolveRealtimeThread(const std::shared_ptr<SppTask> &task, const ConnectionConfig& config);

    // We can mostly use GuiOem7Processor::RenderTask, 
    // but maybe we want a slightly different one for real-time (e.g. auto-scroll to latest)
    void RenderTask(const std::shared_ptr<SppTask> &task);
}
