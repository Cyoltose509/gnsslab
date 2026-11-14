#pragma once

#include <atomic>
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
#include "core/geom/Ephemeris.h"
#include "core/obs/ObsStruct.h"
#include "core/equ/Result.h"
#include "core/SatID.h"
#include "Observer.h"
#include "QCProcessor.h"
#include "product/sp3/Reader.h"
#include "util/Clock.h"

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

    /// 统一参考真值：所有模式共用同一套「恒值 / 逐历元 SP3」数据面。
    /// File 模式下 reader 在解算线程启动时读一次，commitEpoch 逐历元插值。
    struct Truth {
        RefForm form = RefForm::None;
        Eigen::Vector3d fixed = Eigen::Vector3d::Zero();
        std::string sp3Path;
        SatID sat = SatID('L', 49);  // 真值 SP3 实际选中的星：loadTruth 取文件里第一颗 L 星
        bool fileReady = false;
        SP3::Reader reader;
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
        /// 逐历元参考坐标：挂真值文件时每格对应自己的基准，换 ENU 基准时据此重算。
        std::vector<Eigen::Vector3d> refs;
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

        /// 倒序滤波或融合覆盖主解后按 epochs 整段重建，推进 gen 让界面整段重拷。
        void replaceAll(const std::vector<Epoch> &epochs, const XYZ &refECEF, Mode mode);

        /// 与整份拷贝等价，但只复制 `src` 第 `from` 格之后的部分。
        void appendFrom(const PlotData &src, std::size_t from);

        void refreshENU(const XYZ &refECEF);

        /// 指定历元、指定卫星的残差：卫星表逐历元直读主表，不等图表副本的帧同步。
        [[nodiscard]] double codeResidual(SatID sat, int idx) const;
        [[nodiscard]] double phaseResidual(SatID sat, int idx) const;

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

        /// E-N 平面图的散点：只留已出解且有限的点，半径一并算好，界面每帧直接画。
        std::vector<double> planE, planN;
        double planR = 1e-6;
        /// 动态解：这张图是轨迹而不是 ENU 误差，坐标相对 planOrigin，拖拽中心只挪它。
        bool planTrack = false;
        Eigen::Vector3d planOrigin{0, 0, 0};
        bool planOriginSet = false;

        bool hasResidual = false;

        bool hasResRange = false, hasResPhaseRange = false;
        double resYlo = -8.0, resYhi = 8.0;
        double resPhaseYlo = -0.5, resPhaseYhi = 0.5;

        void sync(const PlotData &pp);

        /// 动态解切轨迹口径（静态解用 ENU 误差散点）。
        void setPlanTrack(bool track);

        /// 拖拽中心：只改这张图的显示原点，逐历元真值对比与 CSV 不动。
        void setPlanOrigin(const Eigen::Vector3d &origin);
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
        Clock::Point qcDirtyAt{};
        int qcDirtyTick = 0;
        std::string qcTabLabel = "质量分析";
        std::vector<std::shared_ptr<QcTab> > extraQcTabs;

        std::shared_ptr<void> uiTicket = std::make_shared<int>(0);

        std::map<SatID, std::vector<SkyPoint> > skyTracks;
        int skyTracksBuilt = 0;

        void ensureSkyTracks(const std::vector<Epoch> &epochs);

        /// 主解被倒序/融合覆盖后按 epochs 重画 LEO 轨迹与 RTN 曲线。
        void rebuildTruthTrails(const std::vector<Epoch> &epochs);

        int selectedEpoch = -1;
        /// 上一历元的解算游标：选中历元若仍停在它上面（用户没拖走），就随游标一起移动。
        int lastSolveCursor = -1;
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
        /// 观测文件的格式：读数、算不算自带星历、界面显示都问它。
        Observer::Format obsFormat = Observer::Format::Unknown;
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

        enum class SolvePhase { Forward, Scan, Backward };
        /// 当前正在处理的第几个历元（1 起）；前向递增、后向递减、扫描递增。
        std::atomic<int> solveEpoch{0};
        std::atomic<SolvePhase> solvePhase{SolvePhase::Forward};

        std::vector<Epoch> epochs;
        XYZ refECEF{0, 0, 0};
        std::atomic<bool> initializedRefECEF{false};
        Truth truth;

        /// 解算所用精密轨道（SP3 头的 #cP）的参考框架。解继承产品的框架，标错就是白算。
        /// `hasOrbitFrame = false` 表示产品没声明框架（广播星历解也算这一类），导出按 IGS14 写。
        Frame::Ref orbitFrame = Frame::Ref::IGS14;
        bool hasOrbitFrame = false;

        View view;

        /// 按解算配置灌入真值形式；CLI 与 makeTask 走这里，GUI 直接改 truth。
        void setupTruth(const SolverOptions &cfg);

        /// 观测路径 → 运行态派生（filePath / fileName / obsFormat）与输入框缓冲 view.obsPathBuf。
        /// CLI 的 applyConfig 与 GUI 的 beginFileSolve 共用这一个出口，改派生规则只改这里；
        /// 派生形态特殊的模式（RTK 双站名）先调它再覆写 fileName。
        void applyObsPath(const std::string &obsPath);

        /// 解算线程启动前读一次 SP3 并选真值星；失败只记日志，不影响解算。
        bool loadTruth();

        /// 产品框架与真值框架不一致时警告一次：RTN/ENU 里含两个实现之间的系统差，不能算进模型误差。
        void warnFrameMismatch() const;

        /// 逐历元真值对比：恒值设 ENU 基准，文件插值给 Epoch::refECEF，
        /// 动态模式共用参考轨迹，LEO 另给 RTN。
        void applyTruthAtEpoch(Epoch &ep, int index);

        /// 倒序趟结果写回后按同一参考轨道重算本历元真值派生量。
        void refreshTruthAtEpoch(Epoch &ep, const CommonTime &epoch);

        /// E-N 平面图口径：静态解画 ENU 误差散点，动态解画轨迹（中心可拖、只挪显示）。
        [[nodiscard]] bool enPlanStatic() const {
            return mode == Mode::PPPStatic || truth.form == RefForm::Fixed;
        }

        /// 真值基准是否就绪（恒值坐标有效或真值 SP3 已载入）：ENU / RTN / 3dRMS 全靠它才算得出误差。
        [[nodiscard]] bool truthReady() const {
            if (truth.form == RefForm::Fixed) return truth.fixed.squaredNorm() > 1.0;
            return truth.form == RefForm::File && truth.fileReady;
        }

        std::atomic<bool> hasNav{false};
        std::atomic<bool> noEphSolve{false};

        /// 实时流端点（ip/port 字符串形式）：运行态唯一真相。GUI 实时页直接编辑它，
        /// CLI 由 applyConfig 从 opts 的 ip/port 换算灌入；解算只认这里，不再回读 opts。
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
