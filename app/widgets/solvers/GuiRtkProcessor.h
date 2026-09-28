#pragma once

#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <set>

#include "GnssStruct.h"
#include "SolverTask.h"   // 复用 SolveTask / EpochData / PlotData / RenderTask / LaunchQC
#include "EphemerisTable.h"
#include "RTK.h"                  // RTK 求解器（RtkTask 持有其实例）

namespace GuiRtkProcessor {
    struct RtkTask : GnssTask::SolverTaskBase {

        std::string baseObsPathBuf;
        std::string roverObsPathBuf;
        std::string navPathBuf;

        std::set<char> enabledSystems{'G', 'C'};
        // 解算参数直接持一份 RTK::Config：默认值只在 RTK 里声明一次，GUI 不再抄一遍
        RTK::Config rtkCfg{};
        double cutoffDeg = rtkCfg.cutoffElevRad * 180.0 / PI; // 面板按度编辑，初值取自 solver 默认

        std::shared_ptr<GnssTask::SolveTask> core; // 渲染/存储基础设施

        // 渲染层的两个多态点（见 GuiSolverPanel::RenderTask）
        std::shared_ptr<GnssTask::SolveTask> solveResult() override { return core; }
        void renderConfigPanel() override;   // 定义见 .cpp

        RTK rtk{};

        struct RtkPair {
            ObsData base;
            ObsData rover;
            std::shared_ptr<EphemerisTable> eph;
            // 两站近似坐标随历元传递。原先读写两线程共用 RTK::mBaseXyz / mRoverApprox：
            // 读取线程写、解算线程已被 processEpoch 读，是实打实的数据竞争。
            Vector3d baseApprox = Vector3d::Zero();
            Vector3d roverApprox = Vector3d::Zero();
        };

        std::queue<RtkPair> pairQueue;
        std::mutex pairMtx;
        std::condition_variable pairCv;
        std::atomic<bool> readDone{false};
        std::atomic<bool> solvingDone{false};  // 全部历元已解算完成（命名对齐 SPP/PPP/LEO）
        std::atomic<int> totalEpochs{0};
        EphemerisTable navTable; // 全量广播星历（RINEX 路径，逐历元共享传给 processEpoch）

        RtkTask() { core = std::make_shared<GnssTask::SolveTask>(); }

        ~RtkTask() override {
            if (core) core->stop = true;
        }
    };


} // namespace GuiRtkProcessor
