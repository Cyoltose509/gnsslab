#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include "CSV.h"
#include "Config.h"
#include "Task.h"
#include "view/View.h"

namespace Solvers {
    /// 一次解算的配置与结果：CLI 与 GUI 共用，呈现状态另存于 View。
    struct Core : Task {
        std::string filePath;
        bool isRINEX = false;

        std::mutex mutex;

        std::set<char> enabledSystems{'G', 'R', 'E', 'C'};
        double cutoffDeg = 15.0;
        double sampleInterval = 0.0;

        SolverOptions opts;

        std::string brdcPathBuf;

        std::atomic<float> readProgress{0.0f};
        std::atomic<int> solvingProgress{0};
        std::atomic<int> solvedCount{0};
        std::atomic<int> totalEpochs{0};

        std::mutex queueMutex;
        std::condition_variable queueCv;
        std::queue<Observer::Item> obsQueue;
        std::atomic<bool> navReady{false};
        std::atomic<bool> readDone{false};
        std::atomic<bool> solvingDone{false};

        std::vector<Epoch> epochs;
        XYZ refECEF{0, 0, 0};
        bool initializedRefECEF = false;
        bool hasRefOrbit = false;

        bool isKinematic = false;

        View view;

        bool hasNav = false;
        bool noEphSolve = false;

        std::vector<RealtimeEndpoint> rtEndpoints;

        Mode mode{Mode::SPP};

        std::shared_ptr<Core> solveResult() override { return std::static_pointer_cast<Core>(shared_from_this()); }

        void exportCSV(std::ostream &out) const {
            Solvers::exportCSV(out, epochs, mode, refECEF, hasRefOrbit);
        }
    };

    int readRINEXObsToQueue(const std::shared_ptr<Core> &core, const std::atomic<bool> &stopFlag);

    int readObsToQueue(const std::shared_ptr<Core> &core, const std::atomic<bool> &stopFlag);

    bool waitNavReady(const std::shared_ptr<Core> &core);

    bool nextObsQueue(const std::shared_ptr<Core> &core, Observer::Item &out);

    void commitEpoch(const std::shared_ptr<Core> &core, Epoch &&item,
                     const std::function<void(int, Epoch &)> &onStored = nullptr);

    void syncTaskStatus(const std::shared_ptr<Task> &task, const std::shared_ptr<Core> &core);
}
