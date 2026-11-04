#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>

#include "EphemerisTable.h"
#include "GnssStruct.h"
#include "RTK.h"
#include "RTKConfig.h"
#include "solvers/common/Core.h"

namespace Solvers::RTK {
    /// RTK 双站解算：文件走配对管线，实时走双流泵；界面在 Panel.cpp。
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

        std::shared_ptr<Solvers::Core> solveResult() override { return core; }

        void applyConfig(const SolverOptions &cfg);

        /// 把当前历元的 RTK 结果提交进 core；首解时用基准站坐标设 ENU 基准。
        void commitRtkEpoch(const ObsData &rover);

        void loadFromMemory();

        void renderConfigPanel();

        void exportCSV(std::ostream &out) const override;

        Task() {
            core = std::make_shared<Solvers::Core>();
            core->mode = Mode::RTK;
        }

        ~Task() override {
            if (core) core->stop = true;
        }

    protected:
        void readBody() override;

        void solveBody() override;

        std::unique_ptr<Realtime::Session> makeSession() override;
    };

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
