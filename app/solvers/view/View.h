#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Epoch.h"
#include "QCProcessor.h"

namespace Solvers {
    struct Core;

    /// 解算页面的呈现状态：图表采样、天空图轨迹、选中项、质量分析中间产物。
    /// 只有 GUI 读写，引擎与 CLI 不碰。
    struct View {
        PlotData plotData;
        std::mutex plotMutex;

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
        std::shared_ptr<QC::QualityReport> qcReport;
        std::atomic<bool> qcReady{false};
        std::mutex qcMutex;
        std::thread qcWorker;
        std::atomic<bool> qcComputing{false};
        std::atomic<bool> qcDirty{false};
        std::string qcTabLabel = "质量分析";
        std::vector<std::pair<std::string, std::shared_ptr<Core> > > extraQcTabs;

        std::map<SatID, std::vector<SkyPoint> > skyTracks;
        int skyTracksBuilt = 0;

        int selectedEpoch = -1;
        int selectedSatIdx = -1;
        int resMode = 0;

        std::string obsPathBuf;

        ~View() {
            if (qcWorker.joinable()) {
                if (qcWorker.get_id() == std::this_thread::get_id()) qcWorker.detach();
                else qcWorker.join();
            }
        }
    };
}
