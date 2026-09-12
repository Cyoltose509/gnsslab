#pragma once

#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>

#include "GnssStruct.h"
#include "GuiFileProcessor.h"   // 复用 SppTask / SppEpochData / PlotData / RenderTask / LaunchQC

namespace GuiPppProcessor {
    // PPP 解算任务。复用 GuiFileProcessor::SppTask 承载历元序列、图表数据、QC、选中态等渲染/存储
    // 基础设施，使"定位解算"与"质量分析"标签页可直接复用 GuiFileProcessor 的渲染逻辑；
    // PPP 特有的精密产品（SP3/CLK/ATX/OSB）路径单独持有，并在解算线程内加载到 PPP 解算器。
    struct PppTask {
        enum class State { Config, Running, Done };
        State state{State::Config};

        std::thread worker;
        std::atomic<bool> loading{false};
        std::atomic<bool> done{false};
        std::atomic<bool> stop{false};
        bool hasError = false;
        std::string errorMsg;
        std::string fileName;
        std::string filePath;

        // 精密产品路径（Application::OpenPppSolve 写入；开始解算前持久化到 ini）
        std::string obsPathBuf;    // 观测文件 (RINEX .??O)
        std::string sp3PathBuf;    // 精密轨道 SP3
        std::string brdcPathBuf;   // 广播星历（SP3/CLK 缺失卫星时回退）
        std::string clkPathBuf;    // 精密钟差 CLK
        std::string atxPathBuf;    // 天线文件 ATX（卫星 PCO）
        std::string osbPathBuf;    // 码偏差 OSB/BIA

        std::shared_ptr<GuiFileProcessor::SppTask> core;  // 渲染/存储基础设施

        PppTask() { core = std::make_shared<GuiFileProcessor::SppTask>(); }

        ~PppTask() {
            stop = true;
            if (worker.joinable()) worker.join();
        }
    };

    void RenderConfigPanel(const std::shared_ptr<PppTask> &task);
    void RenderTask(const std::shared_ptr<PppTask> &task);
} // namespace GuiPppProcessor
