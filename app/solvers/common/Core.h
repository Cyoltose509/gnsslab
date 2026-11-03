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
#include "view/QualityControl.h"

namespace Solvers {
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
        PlotData plotData;
        std::mutex plotMutex;
        XYZ refECEF{0, 0, 0};
        bool initializedRefECEF = false;
        bool hasRefOrbit = false;

        bool isKinematic = false;

        bool hasLeo = false;
        Eigen::Vector3d leoPos3d{0, 0, 0};
        Eigen::Vector3d leoRef3d{0, 0, 0};
        std::vector<TrajPoint> leoTraj;
        std::vector<TrajPoint> leoRefTraj;
        std::vector<SatVis> gnssVis;
        std::map<SatID, std::vector<TrajPoint> > gnssTraj;
        std::vector<double> rtnTimes, rtnR, rtnT, rtnN, rtnD3;

        std::mutex qcInputMutex;
        std::vector<QC::QCObsEpoch> qcInput;

        std::map<SatID, std::vector<SkyPoint> > skyTracks;
        int skyTracksBuilt = 0;

        int selectedEpoch = -1;
        int selectedSatIdx = -1;
        int resMode = 0;

        std::string obsPathBuf;
        std::vector<RealtimeEndpoint> rtEndpoints;

        Mode mode{Mode::SPP};

        std::shared_ptr<QualityReport> qcReport;
        std::atomic<bool> qcReady{false};
        std::mutex qcMutex;
        std::thread qcWorker;

        std::atomic<bool> qcComputing{false};
        bool qcLazy = false;
        bool hasNav = false;
        bool noEphSolve = false;

        std::string qcTabLabel = "质量分析";
        std::vector<std::pair<std::string, std::shared_ptr<Core> > > extraQcTabs;

        ~Core() override {
            stop = true;
            if (qcWorker.joinable()) {
                if (qcWorker.get_id() == std::this_thread::get_id()) qcWorker.detach();
                else qcWorker.join();
            }
        }

        std::shared_ptr<Core> solveResult() override { return std::static_pointer_cast<Core>(shared_from_this()); }

        void exportCSV(std::ostream &out) const {
            Solvers::exportCSV(out, epochs, mode, refECEF, hasRefOrbit);
        }

        void renderConfigPanel() override {
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
