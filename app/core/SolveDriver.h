#pragma once


#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "SolverTask.h"
#include "SPP.h"
#include "RTK.h"
#include "RinexObsReader.h"
#include "OEM7Reader.h"
#include "Exception.h"

namespace SolveDriver {
    struct ObsSource {
        virtual ~ObsSource() = default;
        virtual bool next(GnssTask::ObsItem &out) = 0;
    };

    inline void feedRtk(GnssTask::EpochData &data, const RTK &rtk) {
        const Result &r = rtk.result;
        if (r.numSats <= 0) {
            data.solved = false;
            return;
        }
        data.solved = true;
        data.sppResult = r;
        data.rtkStat = r.stat;
        data.rtkRatio = r.ratio;
        data.rtkBaseFloat = rtk.mBaselineFloat;
        data.rtkFloatXyz = rtk.mBaseXyz + rtk.mBaselineFloat; // 浮点解 ECEF，供 ENU 浮点对比线

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
            const bool ok = rtk.mUsedSats.count(data.satIds[i]) > 0 &&
                      data.satPVTs[i].p.squaredNorm() > 1.0;
            data.rejected[i] = !ok;
            if (ok) ++data.numSatsResult;
        }
    }

    template<typename Solver, typename OnEpoch, typename BeforeSolve>
    void solveStream(ObsSource &src, Solver &solver, OnEpoch onEpoch, BeforeSolve beforeSolve) {
        GnssTask::ObsItem item;
        while (src.next(item)) {
            GnssTask::EpochData data;
            data.getFromObs(item.obs); // 观测骨架（satIds/elevations/…），回填前必须先铺好
            beforeSolve(solver, item);
            bool ok = false;
            try { ok = solver.processEpoch(item.obs); } catch (...) { ok = false; }
            if (ok) data.getFromSPP(solver);
            else data.solved = false;
            if (!onEpoch(solver, item, data)) data.solved = false;
        }
    }

    // OEM7 日志内嵌星历快照注入：RINEX 模式 item.eph 为 nullptr（星历来自外部
    // BRDC/SP3 文件），不动作；OEM7 模式逐历元把快照写入 solver.ephTable。
    // 与 GUI 侧 SppTask/PppTask/LeoTask::beforeSolveEpoch 同一语义。
    template<typename Solver>
    void injectEmbeddedEph(Solver &solver, GnssTask::ObsItem &item) {
        if (item.eph && !(item.eph->gps.empty() && item.eph->bds.empty()))
            solver.ephTable = *item.eph;
    }

    template<typename Solver, typename OnEpoch>
    void solveStationStream(ObsSource &src, Solver &solver, OnEpoch onEpoch) {
        solveStream(src, solver, std::move(onEpoch),
                    [](Solver &s, GnssTask::ObsItem &item) { injectEmbeddedEph(s, item); });
    }

    inline std::vector<GnssTask::EpochData> solveAll(ObsSource &src, SPP &solver) {
        std::vector<GnssTask::EpochData> epochs;
        solveStationStream(src, solver, [&](SPP &, GnssTask::ObsItem &, GnssTask::EpochData &data) {
            epochs.push_back(std::move(data));
            return true;
        });
        return epochs;
    }
    inline bool firstSolvedRef(const std::vector<GnssTask::EpochData> &epochs, Eigen::Vector3d &ref) {
        for (const auto &e: epochs) {
            if (e.solved) {
                ref = e.sppResult.xyz;
                return true;
            }
        }
        return false;
    }

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
}
