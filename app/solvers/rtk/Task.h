#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "core/geom/EphemerisTable.h"
#include "core/obs/ObsStruct.h"
#include "solver/rtk/RTK.h"
#include "solver/rtk/RTKConfig.h"
#include "solvers/common/Task.h"

namespace Solvers::RTK {
    /// RTK 双站解算：文件走配对管线，实时走双流泵；界面在 Panel.cpp。
    struct Task : Solvers::Task {
        /// 基准站观测（流动站观测就是基类的 view.obsPathBuf，与其它模式同源）。
        std::string baseObsPathBuf;
        std::string navPathBuf;

        /// 界面里单独填的 BRDC 优先于产品池里的广播星历
        std::string navPath() const override;

        bool useBaseCoord = false;
        Eigen::Vector3d baseXyz{0, 0, 0};

        ::RTK rtk{};

        struct RTKPair {
            ObsData base;
            ObsData rover;
            std::shared_ptr<EphemerisTable> eph;
        };

        std::queue<RTKPair> pairQueue;
        std::mutex pairMtx;
        std::condition_variable pairCv;
        /// 配对队列腾出空位时通知读取端。
        std::condition_variable pairSpaceCv;

        /// 一趟解算的配置快照：解算期间界面还能改 opts，两趟都必须按开始时这一份跑。
        SolverOptions solveOpts;

        /// 要跑倒序趟时前向顺带留下的配对；纯前向档不存，一帧一过。
        std::vector<RTKPair> replay;

        void applyConfig(const SolverOptions &cfg);


        /// 解一个配对历元并提交；失败也提交空解，保证历元下标与文件严格对齐（倒序趟按下标回填）。
        bool solvePair(const ObsData &base, const ObsData &rover, EphemerisTable &eph);

        /// 一个配对历元的解与本历元已有结果合并，按下标覆盖。
        static void mergeEpoch(Epoch &dst, const Epoch &bwd);

        void loadFromMemory();

        void renderConfigPanel() override;

        /// 浮点解位置来自基线 + 基准站坐标，需要覆盖基类的通用行走法。
        void exportRow(std::size_t i, Export::Row &row) const override;

        /// 基线列（XYZ/ENU/长度）相对基准站，与真值无关，恒可写出；
        /// REF/偏差/3dRMS 列走基类判据（挂真值才有），无真值时整体剔除。

        [[nodiscard]] std::vector<Export::Col> exportFields(Export::Format fmt) const override;

        /// 双站导出前缀用「流动站-基准站」，单看基准站名分不清是哪个任务。
        [[nodiscard]] std::string exportStem() const override;

        void writeCSVParams(CSV::Meta &m) const override;

        Task() {
            mode = Mode::RTK;
            opts.cutoffDeg = RTKConfig{}.cutoffElevRad * RAD_TO_DEG;
        }

        void readBody() override;

        void solveBody() override;

        /// 前向趟：逐配对历元解算并提交；纯后向档只占位，配对留给倒序趟重放。
        void runForward();

        /// 倒序趟：另建解算器从末历元重放，与正序解合并（整周解优先，都浮点才按 σ 融合）。
        void runBackward();

        void onReset() override;

        void wakeQueues() override;

        std::unique_ptr<Realtime::Session> makeSession() override;
    };

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
