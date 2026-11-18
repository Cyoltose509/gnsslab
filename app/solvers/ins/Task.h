#pragma once

#include <atomic>
#include <cstddef>
#include <deque>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "core/ins/InsStruct.h"
#include "solver/ins/INS.h"
#include "ImuScaleProbe.h"
#include "solvers/common/Task.h"

namespace Solvers::INS {

    struct AlignInfo {
        bool used = false;
        double spanSec = 0.0;
        double rollDeg = 0.0, pitchDeg = 0.0, headingDeg = 0.0;
        double earthRateErrDeg = 0.0;
        double gyroSigmaRad = 0.0;
        double yawSpreadDeg = 0.0;
        bool headingReliable = false;
    };

    /// 一次纯惯导解算的结果。解算线程边算边往里追加，界面按历元数增量同步。
    struct Result {
        std::vector<NavState> nav;
        std::vector<NavState> navZupt;
        std::vector<unsigned char> zuptApplied;
        /// 与 nav 一一对应的机体系增量的当次采样值（比力与角速度），只供导出 SBET / CSV 用。
        std::vector<InsEpoch> imu;
        std::size_t zuptCount = 0;
        std::string scaleNote; // 文件页自动检测的结论（型号/反推/失败原因）
        ImuScale scale;
        AlignInfo align;
        /// 静置段估出的加计零偏（机体系，m/s²），已在解算前从比力增量中扣除。
        Eigen::Vector3d accelBias = Eigen::Vector3d::Zero();
        bool accelBiasApplied = false;
    };

    /// 图表曲线（按 kMaxPts 抽取后的点集）
    struct Curves {
        std::vector<double> epoch, e, n, h;
        std::vector<double> vN, vE, vD, vNorm;
        std::vector<double> roll, pitch, heading;
    };

    /// 曲线增量缓存：只把上次之后新增的抽取点补进来，抽取率变化时才整体重建。
    struct CurveCache {
        std::size_t stride = 0;
        std::size_t next = 0;
        Curves freeSol, zuptSol;

        void sync(const Result &r, double lat0, double lon0);
    };

    struct Task : Solvers::Task {
        /// 解算线程改写、界面加锁读；不是不可变快照。
        std::shared_ptr<Result> result;
        mutable std::mutex resMutex;

        /// 读线程 → 解算线程的 IMU 计数样本；队列深度按采样率的二十秒算。
        std::deque<ImuSample> imuQueue;

        /// 出第一个解之前的进度：已收到的头段样本数与是否在做粗对准。
        std::atomic<int> headSamples{0};
        std::atomic<bool> aligning{false};

        CurveCache curves;

        /// 结果页当前查看的解（0 自由解 / 1 零速修正）。导出主列跟随它，Zupt* 附加列恒为零速修正解。
        int shownSolution = 0;
        /// 结果页历元选择是否跟随解算尾部；用户拖走游标后停住，属本任务的界面状态。
        bool followTail = true;

        /// 文件页「自动检测」的进行状态与结论，随结果一起显示在结果页。
        ScaleDetect scaleDet;
        /// 起算那一刻的当量说明，读线程只读它（界面状态不进后台线程）。
        std::string scaleText;

        /// 首帧 IMU 的 GPS 周：毫秒级结算里只存周内秒，导出日历/SP3 时靠它补周。
        int navWeek = 0;

        Task() {
            mode = Mode::INS;
            queueDepth = 4096;
        }

        [[nodiscard]] std::shared_ptr<const Result> snapshot() const;

        /// GUI 入口：Panel.cpp 的配置页与 Result.cpp 的结果页。
        void loadFromMemory();
        void renderConfigPanel() override;
        void renderResult() override;

        [[nodiscard]] std::size_t exportCount() const override;

        [[nodiscard]] Export::Stamp exportStamp(std::size_t i) const override;

        void exportRow(std::size_t i, Export::Row &row) const override;

        [[nodiscard]] std::vector<Export::Col> exportFields(Export::Format fmt) const override;

        [[nodiscard]] std::vector<Export::Col> exportDefaults(Export::Format fmt) const override;

        void writeCSVParams(CSV::Meta &m) const override;

        [[nodiscard]] std::size_t resultEpochCount() const override;

    public:
        /// 读线程：边解码边压队列，队列满就等解算侧腾位。
        void readBody() override;
        void solveBody() override;
        void onReset() override;

    private:
        /// 自由解与零速修正解：从同一初始状态出发，逐历元各推一路，主列与对照列都靠这一对。
        ::INS freeIns;
        ::INS zuptIns;

        /// 读线程解出的当量，解算侧按它换算采样率与判静窗口。
        ImuScale scale;

        /// 初始位置 (rad, rad, m) 与正常重力：onReset 里算一次，两线程只读。
        Eigen::Vector3d llh0 = Eigen::Vector3d::Zero();
        double gravity = 0.0;
        std::string initialError;

        /// 粗对准与静置标定用的头段；攒完除起算样本外回填队列继续推进。
        ImuLog alignHead;

        /// 导航起算样本下标：对准段末端，段内的上电沉降不算载体运动。
        std::size_t solveFrom = 0;

        /// 限速喂队列；返回 false 表示已被取消或解算已结束。
        bool feedImu(const ImuSample &s);

        /// 从队列取一个样本；队列空且读线程收尾时返回 false。
        bool nextImu(ImuSample &out);

        /// 攒粗对准与静置标定的头段；返回 false 表示已置错。
        bool prepareAlignHead(const INSConfig &c);

        /// 粗对准（或读手填姿态）→ 两路初始化 → 静置段标定，失败时已置错。
        bool prepareInitialState(const INSConfig &c);

        /// 逐历元推进两路解算，按批发布给界面。
        void runSolveLoop();
    };

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
