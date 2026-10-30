#pragma once

#include <string>
#include <memory>
#include <sstream>
#include <cstdio>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <set>

#include "GnssStruct.h"
#include "SolverTask.h"
#include "GuiHelpers.h"
#include "EphemerisTable.h"
#include "RtkConfig.h"
#include "RTK.h"
#include "core/PathMemory.h"

namespace Solvers::RtkProcessor {

    using GnssTask::sysSetToString;
    using GnssTask::stringToSysSet;

    struct RtkTask : GnssTask::SolverTaskBase {
        std::string baseObsPathBuf;
        std::string roverObsPathBuf;
        std::string navPathBuf;

        bool useBaseCoord = false;
        Eigen::Vector3d baseXyz{0, 0, 0};

        std::set<char> enabledSystems{'G', 'R', 'E', 'C'};

        RtkConfig rtkCfg{};
        double cutoffDeg = rtkCfg.cutoffElevRad * RAD_TO_DEG;
        double sampleInterval = 0.0;

        std::shared_ptr<GnssTask::SolveTask> core;
        std::shared_ptr<GnssTask::SolveTask> solveResult() override { return core; }

        void renderConfigPanel() override;

        RTK rtk{};

        struct RtkPair {
            ObsData base;
            ObsData rover;
            std::shared_ptr<EphemerisTable> eph;
        };

        std::queue<RtkPair> pairQueue;
        std::mutex pairMtx;
        std::condition_variable pairCv;
        std::atomic<bool> readDone{false};
        std::atomic<bool> solvingDone{false};
        std::atomic<int> totalEpochs{0};
        EphemerisTable navTable;


        RtkTask() {
            core = std::make_shared<GnssTask::SolveTask>();
            const auto &pm = AppConfig::PathMemory::instance();
            baseObsPathBuf = pm.get("rtk_base");
            roverObsPathBuf = pm.get("rtk_rover");
            navPathBuf = pm.get("rtk_nav");
            if (const std::string sysStr = pm.get("rtk_systems"); !sysStr.empty()) enabledSystems = stringToSysSet(sysStr);
            cutoffDeg = pm.getNum("rtk_cutoff", cutoffDeg, 0.0, 90.0);
            sampleInterval = pm.getNum("rtk_interval", sampleInterval, 0.0, 3600.0);
            core->rtEndpoints.resize(2);
            core->rtEndpoints[0].ip = pm.get("rtk_rt_base_ip", "8.148.22.229");
            core->rtEndpoints[0].port = pm.get("rtk_rt_base_port", "2002");
            core->rtEndpoints[1].ip = pm.get("rtk_rt_rover_ip", "8.148.22.229");
            core->rtEndpoints[1].port = pm.get("rtk_rt_rover_port", "3002");
            if (const std::string manualStr = pm.get("rtk_base_manual"); !manualStr.empty() && manualStr != "0") useBaseCoord = true;
            if (const std::string xyzStr = pm.get("rtk_base_xyz"); !xyzStr.empty()) {
                double v[3]{0, 0, 0};
                int n = 0;
                std::stringstream ss(xyzStr);
                std::string tok;
                while (n < 3 && std::getline(ss, tok, ',') && std::sscanf(tok.c_str(), "%lf", &v[n]) == 1) ++n;
                if (n == 3) baseXyz = Eigen::Vector3d(v[0], v[1], v[2]);
            }
            Helpers::loadSolverParams(GnssTask::SolverMode::Rtk, core->opts);
        }

        ~RtkTask() override {
            if (core) core->stop = true;
        }
    };
}
