#pragma once

#include <string>
#include <memory>

#include "SolverTask.h"

namespace GuiPppProcessor {
    struct PppTask : GnssTask::SolverTaskBase {
        std::string obsPathBuf; // 观测文件 (RINEX .??O)
        std::string sp3PathBuf; // 精密轨道 SP3
        std::string brdcPathBuf; // 广播星历（SP3/CLK 缺失卫星时回退）
        std::string clkPathBuf; // 精密钟差 CLK
        std::string atxPathBuf; // 天线文件 ATX（卫星 PCO）
        std::string osbPathBuf; // 码偏差 OSB/BIA
        std::string erpPathBuf; // 地球自转参数 ERP（极移潮汐）
        std::string troPathBuf; // 对流层 TRO 产品（ZTD 约束）
        std::string ionexPathBuf; // 电离层 IONEX GIM（二阶电离层校正）

        bool kinematic = false; // 动态(运动学)解算开关：开=接收机随载体移动，逐历元 SPP 重线性化跟上高速运动体(LEO 等)

        std::shared_ptr<GnssTask::SolveTask> core;

        std::shared_ptr<GnssTask::SolveTask> solveResult() override { return core; }

        void renderConfigPanel() override;

        PppTask() { core = std::make_shared<GnssTask::SolveTask>(); }
    };
} // namespace GuiPppProcessor
