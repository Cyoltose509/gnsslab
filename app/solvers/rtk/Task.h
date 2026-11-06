#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include "EphemerisTable.h"
#include "GnssStruct.h"
#include "RTK.h"
#include "RTKConfig.h"
#include "solvers/common/Task.h"

namespace Solvers::RTK {
    /// RTK 双站解算：文件走配对管线，实时走双流泵；界面在 Panel.cpp。
    struct Task : Solvers::Task {
        std::string baseObsPathBuf;
        std::string roverObsPathBuf;
        std::string navPathBuf;

        bool useBaseCoord = false;
        Eigen::Vector3d baseXyz{0, 0, 0};

        ::RTK rtk{};

        struct RTKPair {
            ObsData base;
            ObsData rover;
            std::shared_ptr<EphemerisTable> eph;
        };

        std::queue<RTKPair> pairQueue;
        std::mutex pairMtx;
        std::condition_variable pairCv;
        /// 配对队列腾出空位时通知读取端。
        std::condition_variable pairSpaceCv;
        EphemerisTable navTable;

        void applyConfig(const SolverOptions &cfg);


        /// 解一个配对历元并在出解时提交；processEpoch 的异常只废掉本历元。
        bool solvePair(const ObsData &base, const ObsData &rover, EphemerisTable &eph);

        void loadFromMemory();

        void renderConfigPanel() override;

        void exportCSV(std::ostream &out) const override;

        Task() {
            mode = Mode::RTK;
            opts.cutoffDeg = RTKConfig{}.cutoffElevRad * RAD_TO_DEG;
        }

    protected:
        void readBody() override;

        void solveBody() override;

        void onReset() override;

        void wakeQueues() override;

        std::unique_ptr<Realtime::Session> makeSession() override;
    };

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
