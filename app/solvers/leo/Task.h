#pragma once

#include <memory>
#include <string>
#include <vector>

#include "GnssStruct.h"
#include "product/SP3OrbitReader.h"
#include "solvers/common/Core.h"
#include "solvers/ppp_kinematic/Task.h"

namespace Solvers::LEO {
    /// LEO 定轨：配置与结果在任务自身（Core），界面在 Panel.cpp。
    /// 继承 PPPKinematic::Task：结果表与单站循环复用，仅补参考轨道/RTN 与专用产品加载。
    struct Task : Solvers::PPPKinematic::Task {
        std::string refSp3PathBuf;

        std::vector<std::string> auxFiles;
        int refPrn = 49;
        /// 参考轨道 SP3 是否加载成功：只有它为真才有 RTN 评估、参考轨迹与结果表里的 R/T/N 列。
        bool hasRefOrbit = false;

        SP3OrbitReader refRdr;
        SatID refSat;

        void applyConfig(const SolverOptions &cfg) override;

        void loadFromMemory();

        void renderConfigPanel() override;

        bool initSolver(::SPP &solver) override;

        bool afterSolveEpoch(::SPP &solver, Solvers::Epoch &data,
                             Observer::Item &item, int index) override;

        /// LEO 结果表：PPP 的列 + 参考轨道 R/T/N（仅有参考轨道时）；参考坐标按历元用自己的轨道。
        void exportCSV(std::ostream &out) const override;

        Task() {
            mode = Mode::LEO;
            refSat = SatID('L', refPrn);
        }

    protected:
        std::unique_ptr<::SPP> makeSolver() override;
    };

    /// LEO 的参数默认值：动态滤波初值，关掉周跳与固体潮，运动阈值放宽到 100 m。
    inline SolverOptions defaultOptions() {
        SolverOptions o{};
        o.spp.sigmaPhase = 0.003;
        o.spp.firstEpochPosVar = 1.0;
        o.spp.ambNoise = 1.0E-7;
        o.spp.minSatsForUpdate = 4;
        o.spp.useCycleSlip = false;
        o.spp.applySolidTide = false;
        o.motionThresh = 100.0;
        return o;
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg);
}
