#pragma once


#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "SolverTask.h"
#include "SPP.h"
#include "RTK.h"
#include "RinexObsReader.h"
#include "RinexNavStore.h"
#include "OEM7Reader.h"
#include "ObsFormat.h"
#include "Exception.h"
#include "core/DualObsPairing.h"
#include <chrono>
#include <thread>

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
        if (r.stat == 1) data.rtkFixedBaseline = rtk.mBaselineFixed; // 固定基线（相对 mBaseXyz，仅固定解有效）
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

    inline DualObsPairing::Puller makeObsPull(const bool rinex, OEM7Reader &oem, RinexObsReader &rnx,
                                              std::fstream &fs, const std::string &path, const char *who,
                                              ObsData &cache, bool &cached,
                                              std::shared_ptr<EphemerisTable> &cacheEph, std::string &err) {
        if (!rinex) {
            if (!oem.open(path)) {
                err = std::string("无法打开 ") + who + " OEM7 日志";
                return nullptr;
            }
            if (oem.getNextEpoch(cache)) {
                cached = true;
                cacheEph = oem.ephSnapshot();
            }
            return [&oem, &cache, &cached, &cacheEph](ObsData &out, std::shared_ptr<EphemerisTable> &eph) -> bool {
                if (cached) {
                    out = cache;
                    cached = false;
                    eph = cacheEph;
                    return true;
                }
                if (!oem.getNextEpoch(out)) return false;
                eph = oem.ephSnapshot();
                return true;
            };
        }
        fs.open(path, std::ios::in);
        if (!fs.is_open()) {
            err = std::string("无法打开 ") + who + " RINEX 观测文件";
            return nullptr;
        }
        rnx.pFileStream = &fs;
        try { rnx.parseRinexObs(); }
        catch (...) {
            err = std::string(who) + " RINEX 解析失败";
            return nullptr;
        }
        return [&rnx](ObsData &out, std::shared_ptr<EphemerisTable> &eph) -> bool {
            try { out = rnx.parseRinexObs(); }
            catch (const EndOfFile &) { return false; } catch (...) { return false; }
            eph = nullptr;
            return true;
        };
    }

    // RINEX 首历元探测（首跳快进对齐用）：单独开流解析第一历元，不动目标 reader。
    inline CommonTime probeRinexFirstEpoch(const std::string &path, bool &ok) {
        ok = false;
        try {
            RinexObsReader pr;
            std::fstream pf(path, std::ios::in);
            if (!pf.is_open()) return CommonTime();
            pr.pFileStream = &pf;
            const ObsData o = pr.parseRinexObs();
            ok = true;
            return o.epoch;
        } catch (...) { return CommonTime(); }
    }

    // 调用方持有（生命周期须覆盖配对全程）：两路 reader + 流 + OEM7 首历元缓存。
    struct DualObsHandles {
        OEM7Reader oemBase, oemRover;
        RinexObsReader rnxBase, rnxRover;
        std::fstream fsBase, fsRover;
        ObsData baseCache, roverCache;
        bool baseCached = false, roverCached = false;
        std::shared_ptr<EphemerisTable> baseCacheEph, roverCacheEph;
    };

    inline bool openDualObsSources(const bool baseRinex, const bool roverRinex,
                                   const std::string &basePath, const std::string &roverPath,
                                   DualObsHandles &h,
                                   DualObsPairing::Puller &nextBase, DualObsPairing::Puller &nextRover,
                                   std::string &err) {
        nextBase = makeObsPull(baseRinex, h.oemBase, h.rnxBase, h.fsBase, basePath, "基准站",
                               h.baseCache, h.baseCached, h.baseCacheEph, err);
        if (!nextBase) return false;
        nextRover = makeObsPull(roverRinex, h.oemRover, h.rnxRover, h.fsRover, roverPath, "流动站",
                                h.roverCache, h.roverCached, h.roverCacheEph, err);
        if (!nextRover) return false;
        if (baseRinex) {
            CommonTime t;
            bool ok = false;
            if (h.roverCached) { t = h.roverCache.epoch; ok = true; }   // rover OEM7 探测缓存
            else if (roverRinex) t = probeRinexFirstEpoch(roverPath, ok);
            if (ok) h.rnxBase.fastForwardToEpoch(t);
        }
        if (roverRinex) {
            CommonTime t;
            bool ok = false;
            if (h.baseCached) { t = h.baseCache.epoch; ok = true; }     // base OEM7 探测缓存
            else if (baseRinex) t = probeRinexFirstEpoch(basePath, ok);
            if (ok) h.rnxRover.fastForwardToEpoch(t);
        }
        return true;
    }

    // RTK 文件模式输入校验（GUI 红字与 CLI stderr 共用同一份判定；空串=通过）。
    inline std::string rtkFileInputError(const std::string &basePath, const std::string &roverPath,
                                         const std::string &navPath) {
        if (basePath.empty() || roverPath.empty()) return "请先选择基准站与流动站观测文件";
        const auto fb = GnssTask::classifyObsPath(basePath);
        const auto fr = GnssTask::classifyObsPath(roverPath);
        if (fb == GnssTask::ObsFormat::Unknown || fr == GnssTask::ObsFormat::Unknown)
            return "无法识别观测文件格式";
        if ((fb == GnssTask::ObsFormat::Rinex || fr == GnssTask::ObsFormat::Rinex) && navPath.empty())
            return "RINEX 模式需要广播星历文件（任一观测文件为 RINEX 即需要）";
        return {};
    }

    // 广播星历加载 + 空校验（任一侧 RINEX 时必需；OEM7 内嵌快照可自足，不强制 nav）。
    inline bool loadRtkNav(const std::string &navPath, EphemerisTable &navTable, std::string &err) {
        RinexNavStore nav;
        nav.loadFile(navPath, navTable);
        if (navTable.gps.empty() && navTable.bds.empty()) {
            err = "广播星历为空";
            return false;
        }
        return true;
    }

    template <class GetBase, class GetRover, class OnAttempt, class OnPair>
    bool pumpRealtimePair(DualObsPairing &pairing,
                          GetBase getBase, GetRover getRover,
                          const std::function<bool()> &shouldStop,
                          OnAttempt onAttempt, OnPair onPair) {

        // ⚠️ base/rover 及其星历快照必须与 haveBase/haveRover 同寿命（循环外）。
        // 若声明在循环内，则"本轮保留未重读的那一路"会在下一轮变成默认构造的空 ObsData
        // （epoch = (0,0)），判定时它永远比对面"更旧"→ 无限重读另一侧、一个历元都配不上。
        bool haveBase = false, haveRover = false;
        ObsData base, rover;
        std::shared_ptr<EphemerisTable> baseEph, roverEph;

        // 半开连接判定（配对器自有语义，不能用 socket 的"最后收字节时间"）：
        // 配对时会长时间"握住"一侧历元去追另一侧，被握住的那侧根本没被读取，
        // 若按字节静默判死就会误杀（实测 t=6s 即误判断线）。
        // 正确口径：只有在"正在主动轮询该侧、却连续 kIdleEpochMs 拿不到任何历元"时才判死。
        constexpr auto kIdleEpochMs = std::chrono::milliseconds(10000);
        bool basePolling = false, roverPolling = false;
        std::chrono::steady_clock::time_point basePollSince{}, roverPollSince{};

        while (!shouldStop()) {
            if (!haveBase) {
                if (!basePolling) { basePolling = true; basePollSince = std::chrono::steady_clock::now(); }
                const int r = getBase(base, baseEph);
                if (r < 0) return false;                        // 基准站断线
                if (r > 0) {
                    pairing.observeBase(base.weekSecond.sow);
                    haveBase = true;
                    basePolling = false;
                } else if (std::chrono::steady_clock::now() - basePollSince > kIdleEpochMs) {
                    return false;                               // 轮询 10s 无历元：半开连接
                }
            }
            if (!haveRover) {
                if (!roverPolling) { roverPolling = true; roverPollSince = std::chrono::steady_clock::now(); }
                const int r = getRover(rover, roverEph);
                if (r < 0) return false;                        // 流动站断线
                if (r > 0) {
                    pairing.observeRover(rover.weekSecond.sow);
                    haveRover = true;
                    roverPolling = false;
                } else if (std::chrono::steady_clock::now() - roverPollSince > kIdleEpochMs) {
                    return false;                               // 轮询 10s 无历元：半开连接
                }
            }
            if (!haveBase || !haveRover) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            if (!onAttempt()) return true;                     // 调用方要求停止（如历元到限）
            const double tol = pairing.pairWindow();
            if (const auto act = DualObsPairing::classify(base.epoch, rover.epoch, tol); act == DualObsPairing::Action::Pair) {
                if (!onPair(base, rover, DualObsPairing::selectEph(baseEph, roverEph))) return true;
                haveBase = haveRover = false;
            } else if (act == DualObsPairing::Action::AdvanceBase) {
                haveBase = false;                               // 基准站历元过时，丢弃重读
            } else {
                haveRover = false;                              // 流动站历元过时，丢弃重读
            }
        }
        return true;
    }
}
