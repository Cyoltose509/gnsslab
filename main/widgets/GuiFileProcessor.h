#pragma once

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

struct HWND__;
typedef HWND__ *HWND;

#include "GnssStruct.h"
#include "SPPIFCode.h"
#include "CoordConvert.h"
#include "QualityControl.h"

namespace GuiFileProcessor {
    struct SppEpochData {
        unsigned int week = 0;
        double sow = 0;
        bool solved = false;

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

        // 后验残差图的稳健 Y 轴范围（解算完成后按数据计算一次，避免个别粗差把图压扁）
        bool resRangeReady = false;
        double resYlo = -8.0, resYhi = 8.0;

        std::map<SatID, std::vector<double>> satResTimes;
        std::map<SatID, std::vector<double>> satResVals;

        void insert(int index, const SppEpochData &ep, const XYZ &refECEF);
        void refreshENU(const std::vector<SppEpochData> &ep, const XYZ &refECEF);
        void clear();
    };

    /// 扫描 RINEX obs 同目录下的伴生导航文件
    std::vector<std::string> ScanNavFiles(const std::string &obsPath);

    struct SppTask {
        std::string filePath;
        std::string fileName;
        bool isRealtime = false;

        std::thread worker;
        std::atomic<bool> loading{false};
        std::atomic<bool> done{false};
        std::atomic<bool> stop{false};
        bool hasError = false;
        std::mutex mutex;

        enum class Phase { Reading, Solving };
        std::atomic<Phase> phase{Phase::Reading};
        std::atomic<int> solvingProgress{0};
        std::atomic<float> readProgress{0.0f};
        int totalEpochs{0};

        std::vector<SppEpochData> epochs;
        PlotData plotData;
        XYZ refECEF{0, 0, 0};
        bool initializedRefECEF = false;

        int selectedEpoch = -1;
        int selectedSatIdx = -1;
        std::string errorMsg;

        // ---- 配置（在解算开始前由 GUI 设定） ----
        enum class State { Config, Running, Done };
        std::atomic<State> state{State::Config};
        bool useIF = true;                // true=IF 载波相位, false=IF-code 纯伪距
        bool useKalman = true;           // true=Kalman 滤波, false=LSQ
        std::set<char> enabledSystems{'G', 'C'};
        std::vector<std::string> navFiles;  // RINEX 伴生文件列表（含状态标记）

        // ---- 质量分析缓存 ----
        bool qcReady = false;
        long qcEpochCount = 0;
        QualityReport qcReport;

        ~SppTask() { stop = true; }
    };

    void SolveThread(const std::shared_ptr<SppTask> &task);
    void RenderTask(const std::shared_ptr<SppTask> &task, bool isRealtime = false);
    std::string ShowOpenFileDialog(HWND hwnd);
    void ExportCsv(const std::shared_ptr<SppTask> &task, HWND hwnd);

    void RenderConfigPanel(const std::shared_ptr<SppTask> &task);
} // namespace GuiFileProcessor
