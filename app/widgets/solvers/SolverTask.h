#pragma once

// ---------------------------------------------------------------------------
// 全局解算 Task 基础设施（与具体解算器无关）
//
// 这里存放 SPP / PPP / RTK / LEO 五套 Processor 共用的「任务容器与数据骨架」：
//   SolverTaskBase  任务生命周期状态机（State / worker / loading / done / stop / 错误）
//                   + 渲染层的两个多态点 solveResult() / renderConfigPanel()
//   SolveTask         任务容器：观测队列 + 历元结果 + 绘图缓存 + QC 结果。
//                    五种解算器共用同一套容器，与具体解算器无关（故不带 SPP 前缀）
//   EpochData    单历元解算结果（含 LEO/RTK 的扩展字段）
//   PlotData        绘图序列缓存
//   ObsItem         观测 + 星历的入队单元
//   SolverMode      解算器标识（导出文件名前缀 / CSV 列布局）
//   ScanNavFiles    伴生导航文件扫描
//   TrajPoint/SatVis/SkyPoint  纯数据结构（3D 轨迹、可见星、天顶图点）
//
// 解算主循环的公共部分（等星历就绪 / 取历元 / 入列）也在本文件声明，
// 见 WaitNavReady / NextObsQueue / CommitEpoch。
//
// 职责边界：本头只提供「数据与状态」，不含任何 ImGui 渲染；因此也不包含 GuiCharts.h
// （那会把整个 imgui 拖进任何只想用数据结构的场合）。 GuiCharts.h 反过来包含本头。
// 渲染分发层是 GuiSolverPanel（见 GuiSolverPanel.h），它只依赖这里的两个多态点。
// ---------------------------------------------------------------------------

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
#include <utility>
#include <functional>

#include "GnssStruct.h"
#include "SPP.h"
#include "CoordConvert.h"
#include "QualityControl.h"
#include "EphemerisTable.h"

// 天顶图点：历元序号 + 方位/高度角(度) + 是否参与解算
struct SkyPoint {
    int epIdx = 0;
    float azDeg = 0, elDeg = 0;
    bool used = true;
};

// 单颗 GNSS 卫星当前位置（ECEF, m）+ 是否参与解算
struct SatVis {
    SatID sat;
    char sys = 0;
    Vector3d pos{0, 0, 0};
    bool used = true;
};

// 一条历史轨迹点：历元序号 + ECEF 位置(m)
struct TrajPoint {
    int epIdx = 0;
    Vector3d pos{0, 0, 0};
};

namespace GnssTask {
    // 配置页窗口名的序号源（见 SolverTaskBase::cfgIndex）
    inline int NextCfgIndex() {
        static std::atomic<int> seq{0};
        return seq.fetch_add(1) + 1;
    }

    // 解算 Task 生命周期样板基类：PPP/RTK/LEO/SPP/实时 五套 Processor 的 Task 共享同一套
    // 状态机/后台线程/停止标志，避免各自重复声明 {State,worker,loading,done,stop,...}。
    // 同时是渲染分发层唯一需要的接口（solveResult() + renderConfigPanel()），
    // 于是application侧只需维护一个 shared_ptr<SolverTaskBase> 列表。
    struct SolverTaskBase : std::enable_shared_from_this<SolverTaskBase> {
        enum class State { Config, Running, Done };

        // 后台线程写、渲染线程读 -> 必须 atomic，否则是数据竞争
        std::atomic<State> state{State::Config};

        std::thread worker;
        std::atomic<bool> loading{false};
        std::atomic<bool> done{false};
        std::atomic<bool> stop{false};

        // 标签页标题用的显示名与数据源类型：五种任务的 tab 都要读，故放基类。
        // 原先每个外层 Task 各存一份 fileName，还得从 core 镜像过来，两份容易写歪。
        std::string fileName;
        bool isRealtime = false;

        // ---- 错误状态（后台线程写 / 渲染线程读）----
        // hasError/errorMsg 原为裸成员：std::string 并发读写会破坏其内部指针（不只是脏读），
        // 故收为私有，一律经下面的访问器读写：
        //   errorFlag()                 —— 无锁读标志（atomic，渲染每帧调用）
        //   errorMessage()              —— 加锁取 errorMsg 快照
        //   setError(msg)/clearError()  —— 加锁写 errorMsg 后再置/清标志
        //   syncErrorFrom(src)          —— 整体同步标志与消息（外层 PPP/LEO/RTK Task → core）
        // 锁层级：errorMutex 只保护 errorMsg，是最内层锁——取它时不持有任何其它锁，
        // 持有它时也不再去抢别的锁，因此不会与 mutex / plotMutex / qcInputMutex 成环。
        bool errorFlag() const;

        std::string errorMessage() const;

        void setError(std::string msg);

        void clearError();

        void syncErrorFrom(const SolverTaskBase &src);

        // ---- 渲染层的两个多态点（见 GuiSolverPanel::RenderTask）----
        // 渲染分发层只认这两个接口，因此不再需要每个解算器各声明一个 RenderTask 自由函数：
        //   solveResult()        本任务的结果容器（SPP/实时 直接用 SolveTask 本身，其余持有 core 成员）
        //   renderConfigPanel()  配置阶段的配置面板渲染（各解算器差异最大的一块 UI）
        // 不叫 core()：派生类里已经有一个叫 core 的结果容器成员，同名会冲突。
        // 返回 shared_ptr 而非裸指针：调用方（Application / GuiSolverPanel）本来都持有 shared_ptr，
        // 生命周期由它们保证；散落的裸指针会让渲染层在 shared_ptr 与裸指针之间反复转换。
        virtual std::shared_ptr<SolveTask> solveResult() = 0;

        virtual void renderConfigPanel() = 0;

        // 配置页窗口名后缀：进程内按创建顺序分配，不掺对象地址（避免地址复用撞名）。
        // ImGui 拿完整窗口名当 ini 持久化键，故必须跨运行稳定，否则拖过的尺寸记不住。
        int cfgIndex = NextCfgIndex();

        virtual ~SolverTaskBase() {
            stop = true;
            if (worker.joinable()) worker.join();
        }

    private:
        // 只保护 errorMsg；hasError 自带原子保护，读取方无需加锁。
        mutable std::mutex errorMutex;
        std::atomic<bool> hasError{false};
        std::string errorMsg;
    };


    struct EpochData {
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
        Vector3d refECEF = Vector3d::Zero();
        double rtnR = 0, rtnT = 0, rtnN = 0;

        // RTK 专用：固定解基线(ECEF, m)由 sppResult.xyz − refECEF 给出；此处额外保存浮动解基线、
        // LAMBDA ratio 与解状态(1=Fixed,2=Float)，供"定位解算"标签页状态行与 CSV 导出使用。
        // 由 GuiRtkProcessor 在解算时填充。
        Vector3d rtkBaseFloat{0, 0, 0};
        double rtkRatio = 0.0;
        int rtkStat = 0; // 0=无效, 1=Fixed, 2=Float

        void getFromSPP(const SPP &spp);

        void getFromObs(const ObsData &obs);

        // 按解算器给出的「该星是否可用」判据统一刷 rejected 与 numSatsResult。
        // 三种解算器的排除规则语义本就相同（解算器显式排除，或无有效星历/位置），
        // 各自手写一遍容易漏掉零 PVT 守卫 → 收口到这里，usable(sat, pvt) 只描述解算器差异。
        void applySatRule(const std::function<bool(const SatID &, const PVT &)> &usable);
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

        std::map<SatID, std::vector<double> > satResTimes;
        std::map<SatID, std::vector<double> > satResVals;

        void insert(int index, const EpochData &ep, const XYZ &refECEF);

        void refreshENU(const std::vector<EpochData> &ep, const XYZ &refECEF);

        void clear();
    };

    struct ObsItem {
        ObsData obs;
        std::shared_ptr<EphemerisTable> eph;
    };

    /// 扫描 RINEX obs 同目录下的伴生导航文件
    std::vector<std::string> ScanNavFiles(const std::string &obsPath);

    // 解算器标识：决定导出文件名前缀与 CSV 列布局。
    // 原为 std::string 字面量（"SPP"/"PPP"/"LEO"/"RTK"）并散落比较，拼写漂移编译器查不出来。
    enum class SolverMode { Spp, Ppp, Leo, Rtk };

    inline const char *modeLabel(const SolverMode m) {
        switch (m) {
            case SolverMode::Spp: return "SPP";
            case SolverMode::Ppp: return "PPP";
            case SolverMode::Leo: return "LEO";
            case SolverMode::Rtk: return "RTK";
        }
        return "SPP";
    }

    struct SolveTask : SolverTaskBase {
        std::string filePath;
        bool isRinex = false;

        std::mutex mutex;

        std::set<char> enabledSystems{'G', 'C'};
        double cutoffDeg = 10.0;
        std::vector<std::string> navFiles;

        std::atomic<float> readProgress{0.0f};
        std::atomic<int> solvingProgress{0};
        std::atomic<int> solvedCount{0};
        std::atomic<int> totalEpochs{0};

        std::mutex queueMutex;
        std::condition_variable queueCv;
        std::queue<ObsItem> obsQueue;
        std::atomic<bool> navReady{false};
        std::atomic<bool> readDone{false};
        std::atomic<bool> solvingDone{false};

        // ---- 解算结果----
        std::vector<EpochData> epochs;
        PlotData plotData;
        std::mutex plotMutex;
        XYZ refECEF{0, 0, 0};
        bool initializedRefECEF = false;
        bool hasRefOrbit = false;

        bool hasLeo = false;
        Vector3d leoPos3d{0, 0, 0};
        Vector3d leoRef3d{0, 0, 0};
        std::vector<TrajPoint> leoTraj;
        std::vector<TrajPoint> leoRefTraj;
        std::vector<SatVis> gnssVis;
        std::map<SatID, std::vector<TrajPoint> > gnssTraj;
        std::vector<double> rtnTimes, rtnR, rtnT, rtnN, rtnD3;

        std::mutex qcInputMutex;
        std::vector<QC::QCObsEpoch> qcInput;

        // 天顶图轨迹增量缓存
        using SkyTrackPt = SkyPoint;
        std::map<SatID, std::vector<SkyTrackPt> > skyTracks;
        int skyTracksBuilt = 0;

        int selectedEpoch = -1;
        int selectedSatIdx = -1;

        std::string obsPathBuf;
        std::string rtIpBuf = "47.114.134.129";
        std::string rtPortBuf = "7190";

        SolverMode mode{SolverMode::Spp};

        std::shared_ptr<QualityReport> qcReport;
        std::atomic<bool> qcReady{false};
        std::mutex qcMutex;
        std::thread qcWorker;

        std::atomic<bool> qcComputing{false};
        bool qcLazy = false;
        bool hasNav = false;
        bool noEphSolve = false;

        std::string qcTabLabel = "质量分析";
        std::vector<std::pair<std::string, std::shared_ptr<SolveTask> > > extraQcTabs;

        ~SolveTask() override {
            stop = true;
            if (qcWorker.joinable()) qcWorker.join();
        }

        // 结果即自己：SPP / 实时流没有外层容器，solveResult() 就是自身。
        std::shared_ptr<SolveTask> solveResult() override { return std::static_pointer_cast<SolveTask>(shared_from_this()); }

        void renderConfigPanel() override {
        }
    };

    int ReadRinexObsToQueue(const std::shared_ptr<SolveTask> &core, const std::atomic<bool> &stopFlag);

    bool WaitNavReady(const std::shared_ptr<SolveTask> &core);

    bool NextObsQueue(const std::shared_ptr<SolveTask> &core, ObsItem &out);

    void CommitEpoch(const std::shared_ptr<SolveTask> &core, EpochData &&item,
                     const std::function<void(int, EpochData &)> &onStored = nullptr);


    void SyncTaskStatus(const std::shared_ptr<SolverTaskBase> &task, const std::shared_ptr<SolveTask> &core);

    template<typename Solver>
    void RunSingleStationSolve(const std::shared_ptr<SolveTask> &core,
                               const std::function<bool(Solver &)> &init,
                               const std::function<bool(Solver &, ObsItem &, EpochData &, int)> &body);
}
