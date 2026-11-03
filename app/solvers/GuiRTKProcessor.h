#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <set>
#include <vector>

#include "GnssStruct.h"
#include "solvers/common/Core.h"
#include "view/GuiHelpers.h"
#include "view/GuiParamPage.h"
#include "EphemerisTable.h"
#include "RTKConfig.h"
#include "RTK.h"
#include "StringUtils.h"
#include "core/PathMemory.h"

namespace Solvers::RTK {

    struct Task : Solvers::Task {
        std::string baseObsPathBuf;
        std::string roverObsPathBuf;
        std::string navPathBuf;

        bool useBaseCoord = false;
        Eigen::Vector3d baseXyz{0, 0, 0};

        std::set<char> enabledSystems{'G', 'R', 'E', 'C'};

        RTKConfig rtkCfg{};
        double cutoffDeg = rtkCfg.cutoffElevRad * RAD_TO_DEG;
        double sampleInterval = 0.0;

        std::shared_ptr<Solvers::Core> core;
        std::shared_ptr<Solvers::Core> solveResult() override { return core; }

        void renderConfigPanel() override;

        ::RTK rtk{};

        struct RTKPair {
            ObsData base;
            ObsData rover;
            std::shared_ptr<EphemerisTable> eph;
        };

        std::queue<RTKPair> pairQueue;
        std::mutex pairMtx;
        std::condition_variable pairCv;
        std::atomic<bool> readDone{false};
        std::atomic<bool> solvingDone{false};
        std::atomic<int> totalEpochs{0};
        EphemerisTable navTable;


        Task() {
            core = std::make_shared<Solvers::Core>();
            const auto &pm = AppConfig::PathMemory::instance();
            baseObsPathBuf = pm.get("rtk_base");
            roverObsPathBuf = pm.get("rtk_rover");
            navPathBuf = pm.get("rtk_nav");
            Helpers::loadCommonCfg("rtk", Solvers::Mode::RTK, &core->opts,
                                   &cutoffDeg, &sampleInterval, &enabledSystems);
            core->rtEndpoints.resize(2);
            core->rtEndpoints[0].ip = pm.get("rtk_rt_base_ip", "8.148.22.229");
            core->rtEndpoints[0].port = pm.get("rtk_rt_base_port", "2002");
            core->rtEndpoints[1].ip = pm.get("rtk_rt_rover_ip", "8.148.22.229");
            core->rtEndpoints[1].port = pm.get("rtk_rt_rover_port", "3002");
            if (const std::string manualStr = pm.get("rtk_base_manual"); !manualStr.empty() && manualStr != "0") useBaseCoord = true;
            if (const std::string xyzStr = pm.get("rtk_base_xyz"); !xyzStr.empty()) {
                // "x,y,z"：三段都得是数，缺一段/有一段不是数就整份丢掉 —— 别把解析失败当 0 用
                const auto parts = Str::split(xyzStr, ',', false);
                double v[3]{0, 0, 0};
                bool ok = parts.size() == 3;
                for (std::size_t i = 0; ok && i < 3; ++i) {
                    const char *cur = parts[i].c_str();
                    ok = Str::nextDouble(cur, cur + parts[i].size(), v[i]);
                }
                if (ok) baseXyz = Eigen::Vector3d(v[0], v[1], v[2]);
            }
            Helpers::loadSolverParams(Solvers::Mode::RTK, core->opts);
        }

        ~Task() override {
            if (core) core->stop = true;
        }

    protected:
        void readBody() override;

        void solveBody() override;

        std::unique_ptr<Realtime::Session> makeSession() override;
    };
}
