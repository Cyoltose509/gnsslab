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
#include "CoordConvert.h"

namespace GuiFileProcessor {
    struct SppEpochData {
        unsigned int week = 0;
        double sow = 0;
        bool solved = false;

        // Observation data
        int numObs = 0;
        std::vector<SatID> satIds;
        std::vector<PVT> satPVTs;
        std::vector<double> elevations;
        std::vector<double> azimuths;
        std::vector<bool> rejected;
        std::vector<TypeValueMap> allObs;
        Result sppResult;
        int numSatsResult;

        void getFromSPP(const SPPIFCode &spp);

        void getFromObs(const ObsData &obs);
    };

    struct PlotData {
        std::vector<double> times;
        std::vector<double> sigmaPs;
        std::vector<double> sigmaVs;
        std::vector<double> pdops;
        std::vector<double> enu_e;
        std::vector<double> enu_n;
        std::vector<double> enu_u;
        bool newed = false;

        void insert(const int index, const SppEpochData &ep, const ENU &refECEF) {
            times.push_back(index);
            const auto &result = ep.sppResult;
            const bool solved = ep.solved;
            sigmaPs.push_back(solved ? result.sigmaP : 0.0);
            sigmaVs.push_back(solved ? result.sigmaV : 0.0);
            pdops.push_back(solved ? result.pdop : 0.0);
            if (solved) {
                auto enu = XYZtoENU(result.xyz, refECEF);
                enu_e.push_back(enu[0]);
                enu_n.push_back(enu[1]);
                enu_u.push_back(enu[2]);
            } else {
                enu_e.push_back(0);
                enu_n.push_back(0);
                enu_u.push_back(0);
            }
            newed = true;
        }

        void refreshENU(const std::vector<SppEpochData> &ep, const ENU &refECEF) {
            for (int i = 0; i < enu_e.size(); i++) {
                const auto &result = ep[i].sppResult;
                if (ep[i].solved) {
                    auto enu = XYZtoENU(result.xyz, refECEF);
                    enu_e[i] = enu[0];
                    enu_n[i] = enu[1];
                    enu_u[i] = enu[2];
                } else {
                    enu_e[i] = 0;
                    enu_n[i] = 0;
                    enu_u[i] = 0;
                }
            }
        }

        void clear() {
            times.clear();
            sigmaPs.clear();
            sigmaVs.clear();
            pdops.clear();
            enu_e.clear();
            enu_n.clear();
            enu_u.clear();
        }
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

        // 两阶段进度
        enum class Phase { Reading, Solving };
        std::atomic<Phase> phase{Phase::Reading};
        std::atomic<int> solvingProgress{0};  // 已解算历元数
        std::atomic<float> readProgress{0.0f}; // 文件读取进度 (0.0~1.0)
        int totalEpochs{0};                    // 总历元数

        std::vector<SppEpochData> epochs;
        PlotData plotData;
        XYZ refECEF{0, 0, 0};
        bool initializedRefECEF = false;


        int selectedEpoch = -1;
        int selectedSatIdx = -1;

        std::string errorMsg;
        ~SppTask() {
            stop = true;
        }
    };



    void SolveThread(const std::shared_ptr<SppTask> &task);

    void RenderTask(const std::shared_ptr<SppTask> &task, bool isRealtime = false);

    std::string ShowOpenFileDialog(HWND hwnd);

    void ExportCsv(const std::shared_ptr<SppTask> &task, HWND hwnd);
} // namespace GuiFileProcessor
