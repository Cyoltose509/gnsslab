#pragma once

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

struct HWND__; //NOLINT
typedef HWND__ *HWND;

#include "GnssStruct.h"
#include "SPPIFCode.h"

namespace GuiOem7Processor {
    struct SppEpochData {
        unsigned int week = 0;
        double sow = 0;
        bool solved = false;

        // Observation data
        int numObs = 0;
        std::vector<SatID> satIds;
        std::vector<double> elevations;
        std::vector<double> azimuths;
        std::vector<double> pranges;
        std::vector<bool> rejected;

        // Result data (if solved)
        Eigen::Vector3d xyz{0, 0, 0};
        Eigen::Vector3d blh{0, 0, 0};
        Eigen::Vector3d vel{0, 0, 0};
        double pdop = 0;
        double sigmaP = 0;
        double sigmaV = 0;
        int numSatsResult = 0;
        void getFromSPP(const SPPIFCode &spp);
        void getFromObs(const ObsData &obs);
    };

    struct SppTask {
        std::string filePath;
        std::string fileName;
        bool isRealtime = false; // Flag to distinguish file vs realtime

        std::thread worker;
        std::atomic<bool> loading{false};
        std::atomic<bool> done{false};
        std::atomic<bool> stop{false}; // New stop flag
        bool hasError = false;
        std::mutex mutex;

        std::vector<SppEpochData> epochs;

        unsigned int weekFirst = 0, weekLast = 0;
        double sowFirst = 0, sowLast = 0;

        int selectedEpoch = -1;

        std::string errorMsg;

        ~SppTask() {
            stop = true; 
            if (worker.joinable()) worker.join();
        }
    };

    void SolveThread(const std::shared_ptr<SppTask> &task);

    void RenderTask(const std::shared_ptr<SppTask> &task,bool isRealtime = false);

    std::string ShowOpenFileDialog(HWND hwnd);

    void ExportCsv(const std::shared_ptr<SppTask> &task, HWND hwnd);
} // namespace GuiOem7Processor
