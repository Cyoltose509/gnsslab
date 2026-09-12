#pragma once

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <map>
#include <set>

struct HWND__;
typedef HWND__ *HWND;

#include "GnssStruct.h"
#include "SPP.h"
#include "CoordConvert.h"
#include "QualityControl.h"
#include "EphemerisTable.h"
#include "GuiCharts.h"

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

        // LEO 专用：逐历元参考轨道(SP3)位置与 RTN(径向/沿迹/法向)误差分量。
        // 由 GuiLeoProcessor 在解算时按对应历元 SP3 填充；无参考轨道时保持默认(0)。
        // 导出 CSV 时：LEO 且有参考轨道 → 输出 REF-* 与 R/T/N 列；否则不输出这两类列。
        Eigen::Vector3d refECEF = Eigen::Vector3d::Zero();
        double rtnR = 0, rtnT = 0, rtnN = 0;

        void getFromSPP(const SPP &spp);
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

        bool resRangeReady = false;
        double resYlo = -8.0, resYhi = 8.0;

        std::map<SatID, std::vector<double>> satResTimes;
        std::map<SatID, std::vector<double>> satResVals;

        void insert(int index, const SppEpochData &ep, const XYZ &refECEF);
        void refreshENU(const std::vector<SppEpochData> &ep, const XYZ &refECEF);
        void clear();
    };

    struct ObsItem {
        ObsData obs;
        std::shared_ptr<EphemerisTable> eph;
    };

    /// 扫描 RINEX obs 同目录下的伴生导航文件
    std::vector<std::string> ScanNavFiles(const std::string &obsPath);

    struct SppTask {
        std::string filePath;
        std::string fileName;
        bool isRealtime = false;
        bool isRinex = false;

        std::thread worker;
        std::atomic<bool> loading{false};
        std::atomic<bool> done{false};
        std::atomic<bool> stop{false};
        bool hasError = false;
        std::mutex mutex;

        enum class State { Config, Running, Done };
        std::atomic<State> state{State::Config};

        std::set<char> enabledSystems{'G', 'C'};
        double cutoffDeg = 10.0;
        std::vector<std::string> navFiles;

        std::atomic<float> readProgress{0.0f};
        std::atomic<int> solvingProgress{0};
        std::atomic<int> solvedCount{0};
        int totalEpochs{0};

        std::mutex queueMutex;
        std::condition_variable queueCv;
        std::queue<ObsItem> obsQueue;
        std::atomic<bool> navReady{false};    // 星历已就绪，解算线程可开始
        std::atomic<bool> readDone{false};     // 全部历元已读入队列
        std::atomic<bool> solvingDone{false};  // 全部历元已解算完成

        // ---- 解算结果----
        std::vector<SppEpochData> epochs;
        PlotData plotData;
        std::mutex plotMutex;
        XYZ refECEF{0, 0, 0};
        bool initializedRefECEF = false;
        bool hasRefOrbit = false;   // LEO 是否加载了参考轨道(SP3)：仅用于导出 CSV 是否输出 REF/RTN 列

        // LEO 定轨轨迹（LEO POD 解算器填写；hasLeo=true 时 RenderPositioningTab 点亮 3D/RTN 图）
        bool hasLeo = false;
        Eigen::Vector3d leoPos3d{0, 0, 0};   // 当前历元 LEO 解算位置 ECEF (m)
        Eigen::Vector3d leoRef3d{0, 0, 0};   // 当前历元参考轨道位置 ECEF (m)
        std::vector<GuiCharts::TrajPoint> leoTraj;      // LEO 解算历史轨迹
        std::vector<GuiCharts::TrajPoint> leoRefTraj;   // 参考轨道历史轨迹
        std::vector<GuiCharts::SatVis> gnssVis;         // 当前历元各 GNSS 卫星 ECEF(m)，LEO 3D 图绘制（取最新历元，与 leoPos3d 对齐）
        std::map<SatID, std::vector<GuiCharts::TrajPoint>> gnssTraj; // GNSS 卫星历史轨迹（米）
        std::vector<double> rtnTimes, rtn_r, rtn_t, rtn_n, rtn_d3;  // R/T/N/3D 偏差序列

        // 质量分析原始观测
        std::vector<QC::QCObsEpoch> qcInput;

        // 天顶图轨迹增量缓存
        using SkyTrackPt = GuiCharts::SkyPoint;
        std::map<SatID, std::vector<SkyTrackPt>> skyTracks;
        int skyTracksBuilt = 0;

        int selectedEpoch = -1;
        int selectedSatIdx = -1;
        std::string errorMsg;

        std::string obsPathBuf;              // 文件页：观测文件路径编辑框
        std::string rtIpBuf = "47.114.134.129";   // 实时页：IP（string 存储，默认配置见 Application::OpenSppSolve）
        std::string rtPortBuf = "7190";           // 实时页：端口（string 存储）

        std::string processorLabel = "SPP";       // 解算器标识：SPP / PPP / LEO，用于导出文件名与标签

        std::shared_ptr<QualityReport> qcReport;  // 后台线程算完后整体替换，渲染线程只读
        std::atomic<bool> qcReady{false};
        std::mutex qcMutex;          // 保护 qcReport 的读(渲染)/写(后台线程)
        std::thread qcWorker;        // 后台计算线程（读完后触发一次）

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
    void LaunchQC(const std::shared_ptr<SppTask> &task);
    void ExportCsv(const std::shared_ptr<SppTask> &task, HWND hwnd);

    void RenderConfigPanel(const std::shared_ptr<SppTask> &task);
} // namespace GuiFileProcessor
