#pragma once

#include <vector>

#include "Core.h"
#include "SPP.h"
#include "RTK.h"
#include "Observer.h"
#include "EpochDecimator.h"

// 逐历元引擎原语：驱动求解器、提交 Solvers::Epoch。
// 观测源的打开与对齐见 app/io/ObsSource.h；实时泵在 spp/rtk 各自的 Realtime.h。
namespace Solvers::Driver {
    /// RTK 解出一个历元后构造 Solvers::Epoch 并提交到 core。
    /// 首解时用基准站坐标初始化 core->refECEF（GUI 的 ENU 基准）。
    inline void commitRtk(const std::shared_ptr<Core> &core, const ::RTK &rtk,
                          const ObsData &rover) {
        Epoch data;
        data.getFromObs(rover);
        data.getFromRTK(rtk);
        const Eigen::Vector3d baseXyz = rtk.mBaseXyz;
        commitEpoch(core, std::move(data), [&](int, Epoch &) {
            if (!core->initializedRefECEF) {
                core->refECEF = XYZ(baseXyz.x(), baseXyz.y(), baseXyz.z());
                core->initializedRefECEF = true;
            }
        });
    }

    /// 逐历元跑单站解算器，把观测喂给求解器并把结果填进 Solvers::Epoch。
    /// Solver 限于 SPP 继承链（SPP / PPPStatic / PPPKinematic / LEO），故结果统一由
    /// Epoch::getFromSPP 填；RTK 是双站，走各自的 commitRtk 路径。
    template<typename Solver, typename OnEpoch, typename BeforeSolve>
    void solveStream(Observer::Source &src, Solver &solver, OnEpoch onEpoch, BeforeSolve beforeSolve,
                     const double sampleInterval = 0.0) {
        Observer::Item item;
        const EpochDecimator dec(sampleInterval);
        while (src.next(item)) {
            if (!dec.accept(item.obs.weekSecond.sow)) continue;
            Epoch data;
            data.getFromObs(item.obs);
            beforeSolve(solver, item);
            bool ok = false;
            try { ok = solver.processEpoch(item.obs); } catch (...) { ok = false; }
            if (ok) data.getFromSPP(solver);
            else data.solved = false;
            if (!onEpoch(solver, item, data)) data.solved = false;
        }
    }

}
