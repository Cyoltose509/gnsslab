#pragma once

#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <algorithm>

#include "GnssStruct.h"
#include "SolverTask.h"   // 复用 SolveTask / EpochData / PlotData / RenderTask / LaunchQC
#include "core/AppConfig.h"     // 辅助文件列表 / 参考 PRN 跨会话记忆

namespace GuiLeoProcessor {
    struct LeoTask : GnssTask::SolverTaskBase {

        std::string leoObsPathBuf;     // LEO 星载 RINEX
        std::string gnssSp3PathBuf;    // GNSS 精密轨道 SP3
        std::string gnssBrdcPathBuf;   // GNSS 广播星历（回退）
        std::string gnssClkPathBuf;    // GNSS 精密钟差 CLK
        std::string osbPathBuf;        // 码偏差 OSB/BIA
        std::string atxPathBuf;        // 天线文件 ATX
        std::string refSp3PathBuf;     // 参考轨道 SP3（精度评估用）

        std::vector<std::string> auxFiles;
        int refPrn = 49;               // 参考轨道卫星 PRN

        std::shared_ptr<GnssTask::SolveTask> core;  // 渲染/存储基础设施

        // 渲染层的两个多态点（见 GuiSolverPanel::RenderTask）
        std::shared_ptr<GnssTask::SolveTask> solveResult() override { return core; }
        void renderConfigPanel() override;   // 定义见 .cpp

        LeoTask() {
            core = std::make_shared<GnssTask::SolveTask>();
            const auto &cfg = AppConfig::instance();
            for (auto &r : cfg.getRecent("leo_aux")) {
                if (std::find(auxFiles.begin(), auxFiles.end(), r) == auxFiles.end())
                    auxFiles.push_back(r);
            }
            const std::string prnStr = cfg.get("leo_ref_prn");
            if (!prnStr.empty()) {
                try { refPrn = std::stoi(prnStr); } catch (...) {}
            }
        }

    };

}
