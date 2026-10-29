#pragma once

#include <string>
#include <memory>
#include <algorithm>

#include "GnssStruct.h"
#include "SolverTask.h"
#include "core/PathMemory.h"
#include "Sp3OrbitReader.h"

namespace Solvers::LeoProcessor {
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

        std::shared_ptr<GnssTask::SolveTask> solveResult() override { return core; }
        void renderConfigPanel() override;

        bool initSolver(void *solver) override;
        bool afterSolveEpoch(void *solver, GnssTask::EpochData *data,
                             GnssTask::ObsItem *item, int index) override;
        // beforeSolveEpoch 不 override：走基类默认实现（OEM7 星历快照回退注入）

        Sp3OrbitReader refRdr;
        SatID refSat;
        bool hasRefOrbit = false;

        LeoTask() {
            core = std::make_shared<GnssTask::SolveTask>();
            const auto &cfg = AppConfig::PathMemory::instance();
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
            if (const std::string prnStr = cfg.get("leo_ref_prn"); !prnStr.empty()) {
                try { refPrn = std::stoi(prnStr); } catch (...) {}
            }
            core->cutoffDeg = cfg.getNum("leo_cutoff", core->cutoffDeg, 0.0, 90.0);
            core->sampleInterval = cfg.getNum("leo_interval", core->sampleInterval, 0.0, 3600.0);
            if (const std::string sysStr = cfg.get("leo_systems"); !sysStr.empty()) core->enabledSystems = GnssTask::stringToSysSet(sysStr);
            refSat = SatID('L', refPrn);   // 兜底初值，initSolver 里按 SP3 实际内容改写
        }

    };

}
