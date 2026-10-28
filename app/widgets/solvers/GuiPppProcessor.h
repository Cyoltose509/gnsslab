#pragma once


#include <string>
#include <memory>

#include "SolverTask.h"
#include "core/PathMemory.h"

namespace Solvers::PppProcessor {
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

        bool initSolver(void *solver) override;
        // afterSolveEpoch 不 override：走基类默认实现（首历元参考点初始化）
        // beforeSolveEpoch 不 override：走基类默认实现（OEM7 星历快照回退注入）

        // 回填上次选择：PathMemory 此前只写(开始解算/浏览时 set)不读，重启后面板全空。
        PppTask() {
            core = std::make_shared<GnssTask::SolveTask>();
            const auto &pm = AppConfig::PathMemory::instance();
            obsPathBuf = pm.get("ppp_obs");
            brdcPathBuf = pm.get("ppp_brdc");
            sp3PathBuf = pm.get("ppp_sp3");
            clkPathBuf = pm.get("ppp_clk");
            atxPathBuf = pm.get("ppp_atx");
            osbPathBuf = pm.get("ppp_osb");
            erpPathBuf = pm.get("ppp_erp");
            troPathBuf = pm.get("ppp_tro");
            ionexPathBuf = pm.get("ppp_ionex");
            kinematic = pm.get("ppp_kinematic") == "1";
            // 截止高度角 / 星座筛选：记忆进 ini（默认 15° / 全系统，见 SolverTask.h）
            core->cutoffDeg = pm.getNum("ppp_cutoff", core->cutoffDeg, 0.0, 90.0);
            if (const std::string sysStr = pm.get("ppp_systems"); !sysStr.empty()) core->enabledSystems = GnssTask::stringToSysSet(sysStr);
        }
    };
}
