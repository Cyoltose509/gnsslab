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
        bool usePhase = false;                // true=IF 载波相位(内置Kalman), false=IF-code 纯伪距
        std::set<char> enabledSystems{'G', 'C'};
        std::vector<std::string> navFiles;  // RINEX 伴生文件列表（含状态标记）

        // ---- 质量分析异步计算（后台线程，不阻塞渲染；与定位解算同模式）----
        // 渲染线程只读取 qcReport（shared_ptr），绝不自己重算 → 大文件也不卡。
        std::shared_ptr<QualityReport> qcReport;  // 后台线程算完后整体替换，渲染线程只读
        std::atomic<bool> qcReady{false};
        std::mutex qcMutex;          // 保护 qcReport 的读(渲染)/写(后台线程)
        std::thread qcWorker;        // 后台计算线程（读完后、全部解算完各跑一次）

        // 质量分析计算状态（由后台 qcWorker 写入，渲染线程只读）
        std::atomic<bool>  qcComputing{false}; // true = 正在后台算 QC（区别于「尚未开始」）
        bool hasNav = false;                    // 是否成功加载到伴生星历（无星历仍可做质量分析，仅不能定位）
        bool noEphSolve = false;                // 因缺星历而跳过定位解算

        ~SppTask() {
            stop = true;
            if (qcWorker.joinable()) qcWorker.join();
        }
    };

    void SolveThread(const std::shared_ptr<SppTask> &task);
    void RenderTask(const std::shared_ptr<SppTask> &task, bool isRealtime = false);
    /// 触发一次后台 QC 计算（文件/实时共用）：在后台线程算完整体替换 task->qcReport。
    void LaunchQC(const std::shared_ptr<SppTask> &task);
    std::string ShowOpenFileDialog(HWND hwnd);
    void ExportCsv(const std::shared_ptr<SppTask> &task, HWND hwnd);

    void RenderConfigPanel(const std::shared_ptr<SppTask> &task);
} // namespace GuiFileProcessor
