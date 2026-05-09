#pragma once

#ifndef GNSSLAB_GUI_OEM7_PROCESSOR_H
#define GNSSLAB_GUI_OEM7_PROCESSOR_H

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

struct HWND__;
typedef HWND__ *HWND;

// Eigen 和 Windows 不在同一个头文件里，避免 byte 冲突
#include <Eigen/Eigen>
#include "GnssStruct.h"

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

        // Result data (if solved)
        Eigen::Vector3d xyz{0, 0, 0};
        Eigen::Vector3d blh{0, 0, 0};
        Eigen::Vector3d vel{0, 0, 0};
        double pdop = 0;
        double sigmaP = 0;
        double sigmaV = 0;
        int numSatsResult = 0;
    };

    struct SppTask {
        std::string filePath;
        std::string fileName;

        std::thread worker;
        std::atomic<bool> loading{false};
        std::atomic<bool> done{false};
        bool hasError = false;
        std::mutex mutex;

        std::vector<SppEpochData> epochs;

        unsigned int weekFirst = 0, weekLast = 0;
        double sowFirst = 0, sowLast = 0;

        int selectedEpoch = -1;

        std::string errorMsg;

        ~SppTask() {
            if (worker.joinable()) worker.join();
        }
    };

    void SolveThread(const std::shared_ptr<SppTask>& task);
    void RenderTask(const std::shared_ptr<SppTask>& task);
    std::string ShowOpenFileDialog(HWND hwnd);
    void ExportCsv(std::shared_ptr<SppTask> task, HWND hwnd);

} // namespace GuiOem7Processor

#endif
