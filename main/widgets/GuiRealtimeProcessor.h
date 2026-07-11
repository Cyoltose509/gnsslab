#pragma once

#include "GuiFileProcessor.h"
#include <string>
#include <memory>

namespace GuiRealtimeProcessor {
    // Reuse SppTask from GuiFileProcessor
    using SppTask = GuiFileProcessor::SppTask;

    struct ConnectionConfig {
        std::string ip = "47.114.134.129";
        int port = 7190;
    };

    void SolveRealtimeThread(const std::shared_ptr<SppTask> &task, const ConnectionConfig &config);


    // We can mostly use GuiFileProcessor::RenderTask, 
    // but maybe we want a slightly different one for real-time (e.g. auto-scroll to latest)
    void RenderTask(const std::shared_ptr<SppTask> &task);
}
