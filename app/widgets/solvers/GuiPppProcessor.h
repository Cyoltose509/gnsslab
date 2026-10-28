#pragma once


#include <string>
#include <memory>

#include "SolverTask.h"
#include "core/PathMemory.h"

namespace Solvers::PppProcessor {
    struct PppTask : GnssTask::SolverTaskBase {
        std::string obsPathBuf;
        std::string sp3PathBuf;
        std::string brdcPathBuf;
        std::string clkPathBuf;
        std::string atxPathBuf;
        std::string osbPathBuf;
        std::string erpPathBuf;
        std::string troPathBuf;
        std::string ionexPathBuf;

        bool kinematic = false;

        std::shared_ptr<GnssTask::SolveTask> core;

        std::shared_ptr<GnssTask::SolveTask> solveResult() override { return core; }

        void renderConfigPanel() override;

        bool initSolver(void *solver) override;

        bool wantsKinematic() const override { return kinematic; }

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
            core->cutoffDeg = pm.getNum("ppp_cutoff", core->cutoffDeg, 0.0, 90.0);
            if (const std::string sysStr = pm.get("ppp_systems"); !sysStr.empty()) core->enabledSystems = GnssTask::stringToSysSet(sysStr);
        }
    };
}
