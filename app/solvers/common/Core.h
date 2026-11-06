#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Config.h"
#include "Ephemeris.h"
#include "GnssStruct.h"
#include "Observer.h"
#include "QCProcessor.h"

class SPP;
class RTK;

namespace Solvers {
    struct SkyPoint {
        int epIdx = 0;
        float azDeg = 0, elDeg = 0;
        bool used = true;
    };

    struct TrajPoint {
        int epIdx = 0;
        Eigen::Vector3d pos{0, 0, 0};
    };

    /// 逐历元的原始观测：类型名去重成一张表，每颗卫星只存值 + 类型下标，摊成平行数组省内存。
    struct ObsSnapshot {
        /// 本历元出现过的观测类型名（去重）。整份文件里这套名字基本不变，
        /// 相邻历元内容相同时共享同一份，不各存一遍。
        std::shared_ptr<const std::vector<std::string> > types;
        /// 每颗卫星在下面两条数组里的起点，末尾补总长
        std::vector<int> start;
        /// 与 values 一一对应的类型下标
        std::vector<unsigned short> slots;
        std::vector<double> values;

        /// 展开成界面要的「类型 → 值」。
        [[nodiscard]] TypeValueMap of(std::size_t satIndex) const;
    };

    struct Epoch {
        unsigned int week = 0;
        double sow = 0;
        bool solved = false;

        int numObs = 0;
        std::vector<SatID> satIds;
        /// 卫星位置（ECEF）。求解器给的 PVT 里速度与钟差本程序从不读，只留位置。
        std::vector<Eigen::Vector3d> satPos;
        std::vector<double> elevations;
        std::vector<double> azimuths;
        std::vector<bool> rejected;
        ObsSnapshot rawObs;
        Result sppResult;
        int numSatsResult = 0;

        Eigen::Vector3d refECEF = Eigen::Vector3d::Zero();
        double rtnR = 0, rtnT = 0, rtnN = 0;

        Eigen::Vector3d rtkBaseFloat{0, 0, 0};
        Eigen::Vector3d rtkFixedBaseline{0, 0, 0};
        Eigen::Vector3d rtkFloatXyz = Eigen::Vector3d::Zero();
        double rtkRatio = 0.0;
        int rtkStat = 0;

        void getFromObs(const ObsData &obs);

        void getFromSPP(const ::SPP &spp);

        void getFromRTK(const ::RTK &rtk);

        /// 固定解判定：RTK 看自己的整数解状态，其余模式看 spp 结果状态。
        [[nodiscard]] bool isFixed(Mode mode) const;

    private:
        /// 按 satIds 的顺序回填仰角/方位角/PVT，只认「本历元实际使用」的卫星集合；
        /// 各模式在自己入口把求解器的剔除状态折算成该集合。
        void fillSatFields(const SatValueMap &elev, const SatValueMap &azim,
                           const std::map<SatID, PVT> &pvts, const std::set<SatID> &usedSats);
    };

    /// 图表序列：解析线程每提交一个历元就往里追加一格，界面按版本号取只读副本。
    /// 累计量（解算历元数、有速度历元数、固定历元数）在追加时算好。
    struct PlotData {
        std::vector<double> times;
        std::vector<double> sigmaPs;
        std::vector<double> sigmaPFloatVals;
        std::vector<double> sigmaVs;
        std::vector<double> pdops;
        std::vector<double> gdops, hdops, vdops, tdops;
        std::vector<double> enu_e;
        std::vector<double> enu_n;
        std::vector<double> enu_u;
        /// 逐历元的 ECEF（实解 / 浮点解）：换 ENU 基准时据此重算。
        std::vector<Eigen::Vector3d> ecef;
        std::vector<Eigen::Vector3d> floatEcef;
        std::vector<double> solvedFlags;

        std::vector<double> float_e, float_n, float_u;

        std::map<SatID, std::vector<double> > satResTimes;
        std::map<SatID, std::vector<double> > satResVals;
        std::map<SatID, std::vector<double> > satResPhaseTimes;
        std::map<SatID, std::vector<double> > satResPhaseVals;

        int solvedEpochs = 0;
        int velEpochs = 0;
        int fixedEpochs = 0;
        bool sigmaVNonZero = false;
        unsigned revision = 0;

        /// 代次：`clear()` 与 `refreshENU()` 会改写已有格，推进它；只往后追一格的操作不动它。
        /// 界面据此判断只读副本能否只补新增的尾段。
        unsigned gen = 0;

        void insert(int index, const Epoch &ep, const XYZ &refECEF, Mode mode);

        /// 与整份拷贝等价，但只复制 `src` 第 `from` 格之后的部分。
        void appendFrom(const PlotData &src, std::size_t from);

        void refreshENU(const XYZ &refECEF);

        void clear();
    };

    struct QcTab;

    /// 断点感知的时间轴
    struct BrokenAxis {
        std::vector<double> t;
        std::vector<int> kept;
        double threshold = 0.0;
        int consumed = 0;
        double lastTime = 0.0;
        unsigned srcTag = 0;
        void build(const std::vector<double> &times, double thrHint = 0.0, unsigned tag = 0);

        [[nodiscard]] std::vector<double> align(const std::vector<double> &y) const;
    };

    /// 图表序列的只读副本
    struct PlotSeries {
        unsigned revision = 0;
        bool newed = false;

        PlotData data;
        BrokenAxis axis;
        std::map<SatID, BrokenAxis> resAxisCode;
        std::map<SatID, BrokenAxis> resAxisPhase;

        /// E-N 平面图的散点：只留已解算且有限的点，半径一并算好，界面每帧直接画。
        std::vector<double> planE, planN;
        double planR = 1e-6;

        bool hasResidual = false;

        bool hasResRange = false, hasResPhaseRange = false;
        double resYlo = -8.0, resYhi = 8.0;
        double resPhaseYlo = -0.5, resPhaseYhi = 0.5;

        void sync(const PlotData &pp);

        [[nodiscard]] double codeResidual(SatID sat, int idx) const;

        [[nodiscard]] double phaseResidual(SatID sat, int idx) const;
    };

    /// 解算页面的呈现状态
    struct View {
        PlotData plotData;
        PlotSeries plotSeries;
        std::mutex plotMutex;

        bool hasLeo = false;
        Eigen::Vector3d leoPos3d{0, 0, 0};
        Eigen::Vector3d leoRef3d{0, 0, 0};
        std::vector<TrajPoint> leoTraj;
        std::vector<TrajPoint> leoRefTraj;
        std::vector<double> rtnTimes, rtnR, rtnT, rtnN, rtnD3;

        void pushQCInput(const ObsData &epoch, double sow);

        /// 喂完一帧后置脏并按节流放行：500 ms 或每 200 个历元，先到先算。
        /// 实时流靠时间（帧率不定），读文件靠计数（跑得比实时快）。
        void markQcDirty();

        void clearQCInput(double nominalInterval);

        void endQCInput();

        [[nodiscard]] QC::QualityReport qcSnapshot();

        std::shared_ptr<QC::QualityReport> qcReport;
        std::atomic<bool> qcReady{false};
        std::mutex qcMutex;
        std::thread qcWorker;
        std::atomic<bool> qcComputing{false};
        std::atomic<bool> qcDirty{false};
        std::chrono::steady_clock::time_point qcDirtyAt{};
        int qcDirtyTick = 0;
        std::string qcTabLabel = "质量分析";
        std::vector<std::shared_ptr<QcTab> > extraQcTabs;

        std::shared_ptr<void> uiTicket = std::make_shared<int>(0);

        std::map<SatID, std::vector<SkyPoint> > skyTracks;
        int skyTracksBuilt = 0;

        void ensureSkyTracks(const std::vector<Epoch> &epochs);

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

    private:
        std::mutex qcInputMutex;
        QC::Stream qcStream{0.0};
    };

    struct QcTab {
        std::string label;
        std::string fileName;
        View view;
    };

    struct Core {
        enum class State { Config, Running, Done };

        Mode mode{Mode::SPP};

        std::string filePath;
        std::string fileName;
        bool isRINEX = false;
        bool isRealtime = false;

        std::atomic<State> state{State::Config};
        std::atomic<bool> loading{false};
        std::atomic<bool> done{false};
        std::atomic<bool> stop{false};

        bool errorFlag() const;

        std::string errorMessage() const;

        void setError(std::string msg);

        void clearError();

        std::mutex mutex;

        SolverOptions opts;

        std::mutex queueMutex;
        std::condition_variable queueCv;
        std::condition_variable queueSpaceCv;
        std::size_t queueDepth = 128;
        std::queue<Observer::Item> obsQueue;
        std::atomic<bool> navReady{false};
        std::atomic<bool> readDone{false};
        std::atomic<bool> solvingDone{false};

        std::vector<Epoch> epochs;
        XYZ refECEF{0, 0, 0};
        std::atomic<bool> initializedRefECEF{false};

        View view;

        std::atomic<bool> hasNav{false};
        std::atomic<bool> noEphSolve{false};

        std::vector<RealtimeEndpoint> rtEndpoints;

        bool waitNavReady();

        bool nextObs(Observer::Item &out);

        void commitEpoch(Epoch &&item,
                         const std::function<void(int, Epoch &)> &onStored = nullptr);
        std::pair<int, int> readEpochView();

    private:
        mutable std::mutex errorMutex;
        std::atomic<bool> hasError{false};
        std::string errorMsg;
    };
}
