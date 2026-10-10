#pragma once

// ---------------------------------------------------------------------------
// SolveDriver —— 解算编排共享层（CLI 与 GUI 共用，纯头文件、不依赖 ImGui）
//
// 之前 CLI 因为不链接 SolverTask.cpp（它随 GUI exe 编），被迫重写了一整套
// 「取历元 → fillFromObs → processEpoch → fillFromSPP → 收集」循环，且回填逻辑
// 与 EpochData::getFrom* 重复了两份。这里把差异收口：
//   * ObsSource        统一「取下一历元」的源头差异（RINEX / OEM7 / 实时 socket / GUI 队列）
//   * solveStationStream 唯一一处单站循环骨架，SPP/PPP/LEO 全部共用
//   * fillFromObs/fillFromSPP/feedRtk 唯一一份回填实现，GUI 的 EpochData::getFrom*
//     与 GuiRtkProcessor::feedRtkEpoch 都委托到这里 —— 消灭重复
//
// 设计要点：solveStationStream 只负责「迭代 + 铺观测骨架」，真正的 processEpoch /
// 回填 / 存储 留在 onEpoch 回调里（与各解算器既有 body 形态一致），因此 GUI 的
// RunSingleStationSolve 可以直接改走它而无需改动 body 契约。
// ---------------------------------------------------------------------------

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "SolverTask.h"      // GnssTask::EpochData / ObsItem / SolveTask
#include "SPP.h"             // SPP（ephTable / processEpoch / result / sat*Data）
#include "RTK.h"             // RTK（result / mUsedSats / mBaselineFloat）
#include "RinexObsReader.h"  // RinexObsReader::parseRinexObs
#include "OEM7Reader.h"      // OEM7Reader::getNextEpoch / ephSnapshot
#include "Exception.h"       // EndOfFile

namespace SolveDriver {

// 统一观测源：把「下一历元从哪来」的差异收口为一处。
// 所有单站解算器共用 solveStationStream，不再各自手搓循环。
struct ObsSource {
    virtual ~ObsSource() = default;
    // 取出下一历元（含其星历快照）；无更多历元返回 false（EOF/流结束）。
    // 异常由实现自行消化并返回 false，避免单层循环崩掉整次解算。
    virtual bool next(GnssTask::ObsItem &out) = 0;
};

// fillFromObs / fillFromSPP 是 EpochData::getFromObs / getFromSPP 的薄封装（见下方 solveStationStream 之前），
// 真正的 per-epoch 回填实现只存在一份，即 SolverTask.h 中 EpochData 的这两个 inline 成员。
// CLI（不链 SolverTask.cpp）经由头文件拿到定义；GUI 的 EpochData::getFrom* 与 Cli 的 SolveDriver::fillFrom* 共用之。

// RTK 解算结果回填（stat/ratio/基线/几何/排除标记），SPP/PPP/LEO 同形
inline void feedRtk(GnssTask::EpochData &data, const RTK &rtk) {
    const Result &r = rtk.result;
    if (r.numSats <= 0) {
        data.solved = false;
        return;
    }
    data.solved = true;
    data.sppResult = r;            // stat/ratio/numSats/sigmaP/sigmaXYZ/postRes/xyz 全带走
    data.rtkStat = r.stat;
    data.rtkRatio = r.ratio;
    data.rtkBaseFloat = rtk.mBaselineFloat;

    const int n = static_cast<int>(data.satIds.size());
    for (int i = 0; i < n; ++i) {
        const SatID &sat = data.satIds[i];
        auto itE = rtk.satElevData.find(sat);
        auto itA = rtk.satAzimData.find(sat);
        auto itP = rtk.satPVTRecTime.find(sat);
        if (itE != rtk.satElevData.end()) data.elevations[i] = itE->second;
        if (itA != rtk.satAzimData.end()) data.azimuths[i] = itA->second;
        if (itP != rtk.satPVTRecTime.end()) data.satPVTs[i] = itP->second;
    }
    data.numSatsResult = 0;
    for (int i = 0; i < n; ++i) {
        bool ok = rtk.mUsedSats.count(data.satIds[i]) > 0 &&
                  data.satPVTs[i].p.squaredNorm() > 1.0;
        data.rejected[i] = !ok;
        if (ok) ++data.numSatsResult;
    }
}

// 唯一一处单站循环骨架：SPP/PPP/LEO 全部共用。
// 本函数负责共性部分——迭代、铺观测骨架、processEpoch（异常在此消化）、结果回填；
// onEpoch 只写调用方特有的后续（LEO 的 RTN 标注、GUI 的 CommitEpoch），不必再重写样板。
// Solver 只需公有继承 SPP（PPPStatic/PPPKinematic/LEO 均是），processEpoch 虚分派。
// beforeSolve 在 processEpoch 之前触发，供 SPP 注入逐历元星历（GUI 实时/OEM7 路径需要）。
// onEpoch 返回 false 表示该历元作废（data.solved 会被置 false）。
// item 以非 const 引用传入：processEpoch 接收 ObsData&，需要可写别名。
template <typename Solver, typename OnEpoch, typename BeforeSolve>
void solveStream(ObsSource &src, Solver &solver, OnEpoch onEpoch, BeforeSolve beforeSolve) {
    GnssTask::ObsItem item;
    while (src.next(item)) {
        GnssTask::EpochData data;
        data.getFromObs(item.obs);   // 观测骨架（satIds/elevations/…），回填前必须先铺好
        beforeSolve(solver, item);
        bool ok = false;
        try { ok = solver.processEpoch(item.obs); } catch (...) { ok = false; }
        if (ok) data.getFromSPP(solver);
        else data.solved = false;
        if (!onEpoch(solver, item, data)) data.solved = false;
    }
}

// 无 pre/post 钩子的常见情形：迭代 + 解算 + 回填，onEpoch 收结果。
template <typename Solver, typename OnEpoch>
void solveStationStream(ObsSource &src, Solver &solver, OnEpoch onEpoch) {
    solveStream(src, solver, std::move(onEpoch),
                [](Solver &, GnssTask::ObsItem &) {});
}

// 收全程结果，零 lambda：SPP / PPP 的 CLI 主路径。
// 既然 solveStationStream 已含解算与回填，这里只需一个「收集」动作。
inline std::vector<GnssTask::EpochData> solveAll(ObsSource &src, SPP &solver) {
    std::vector<GnssTask::EpochData> epochs;
    solveStationStream(src, solver, [&](SPP &, GnssTask::ObsItem &, GnssTask::EpochData &data) {
        epochs.push_back(std::move(data));
        return true;
    });
    return epochs;
}

// 首个解出历元的 ECEF 作参考原点。CLI 的 ref 过去靠在 lambda 里维护 refInit 标志取，
// 改为收完再从成品 vector 里找，循环体内不再有可变状态。
// 返回 false = 全程无解算结果，ref 保持调用方给定的初值。
inline bool firstSolvedRef(const std::vector<GnssTask::EpochData> &epochs, Vector3d &ref) {
    for (const auto &e : epochs) {
        if (e.solved) { ref = e.sppResult.xyz; return true; }
    }
    return false;
}

// EpochData::getFromObs / getFromSPP 的 inline 定义收在 SolverTask.h（与 EpochData 同处），
// 这样 CLI（不链 SolverTask.cpp）也能拿到定义；本文件只提供它们的薄封装，逻辑不重复。
inline void fillFromObs(GnssTask::EpochData &data, const ObsData &obs) {
    data.getFromObs(obs);
}

inline void fillFromSPP(GnssTask::EpochData &data, const SPP &spp) {
    data.getFromSPP(spp);
}

// ---- RINEX 文件源 ----
struct RinexFileSource : ObsSource {
    std::fstream f;
    RinexObsReader rdr;
    bool open(const std::string &path) {
        f.open(path, std::ios::in);
        if (!f) return false;
        rdr.pFileStream = &f;
        return true;
    }
    bool next(GnssTask::ObsItem &out) override {
        try {
            out.obs = rdr.parseRinexObs();
        } catch (const EndOfFile &) {
            return false;
        } catch (const std::exception &e) {
            std::cerr << "RINEX 解析错误: " << e.what() << "\n";
            return false;
        } catch (...) {
            return false;
        }
        out.eph = nullptr;
        return true;
    }
};

// ---- OEM7 日志源（逐历元携带星历快照） ----
struct Oem7FileSource : ObsSource {
    OEM7Reader reader;
    bool open(const std::string &path) { return reader.open(path); }
    bool next(GnssTask::ObsItem &out) override {
        if (!reader.getNextEpoch(out.obs)) return false;
        out.eph = reader.ephSnapshot();
        return true;
    }
};

} // namespace SolveDriver
