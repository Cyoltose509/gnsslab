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
#include "core/PathMemory.h"     // 辅助文件列表 / 参考 PRN 跨会话记忆
#include "Sp3OrbitReader.h"   // 参考轨道 SP3 读取（initSolver 加载、afterSolveEpoch 查 PVT）

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

        // ---- 解算钩子（骨架见 RunSingleStationSolve）----
        bool initSolver(void *solver) override;
        bool afterSolveEpoch(void *solver, GnssTask::EpochData *data,
                             GnssTask::ObsItem *item, int index) override;

        // 参考轨道读取与判定：initSolver 负责加载，afterSolveEpoch 逐历元查 PVT，故须跨钩子共享。
        // refSat 在 initSolver 里按 SP3 实际卫星改写（不参与构造期初始化：refPrn 那时才定下来）。
        Sp3OrbitReader refRdr;
        SatID refSat;
        bool hasRefOrbit = false;

        LeoTask() {
            core = std::make_shared<GnssTask::SolveTask>();
            const auto &cfg = GnssCore::PathMemory::instance();
            // 回填各路径输入框（此前只回填了 aux 历史与 refPrn，主路径框仍全空）
            leoObsPathBuf = cfg.get("leo_obs");
            gnssSp3PathBuf = cfg.get("leo_gnss_sp3");
            gnssBrdcPathBuf = cfg.get("leo_gnss_brdc");
            gnssClkPathBuf = cfg.get("leo_gnss_clk");
            osbPathBuf = cfg.get("leo_osb");
            atxPathBuf = cfg.get("leo_atx");
            refSp3PathBuf = cfg.get("leo_ref_sp3");
            for (auto &r : cfg.getRecent("leo_aux")) {
                if (std::find(auxFiles.begin(), auxFiles.end(), r) == auxFiles.end())
                    auxFiles.push_back(r);
            }
            const std::string prnStr = cfg.get("leo_ref_prn");
            if (!prnStr.empty()) {
                try { refPrn = std::stoi(prnStr); } catch (...) {}
            }
            refSat = SatID('L', refPrn);   // 兜底初值，initSolver 里按 SP3 实际内容改写
        }

    };

}
