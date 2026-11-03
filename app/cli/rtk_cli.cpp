#include <windows.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "Const.h"
#include "GnssStruct.h"
#include "solvers/common/Core.h"
#include "RTK.h"
#include "novatel/LogReader.h"
#include "Observer.h"
#include "novatel/SocketReader.h"
#include "Log.h"
#include "SolverOptions.h"
#include "CliCommon.h"
#include "solvers/common/Driver.h"
#include "solvers/rtk/Realtime.h"
#include "solvers/rtk/Pipeline.h"
#include "io/ObsSource.h"
#include "DualPair.h"
#include "core/IniFile.h"

namespace {
    void runRtk(const SolverOptions &cfg) {
        std::vector<Solvers::Epoch> epochs;

        RTK rtk;
        rtk.applyOptions(cfg);

        if (cfg.basePath.empty() || cfg.roverPath.empty()) {
            std::cerr << "RTK 需要基准站与流动站 [input] base / rover\n";
            Cli::dumpEmpty(Solvers::Mode::RTK, cfg);
            return;
        }

        // ini 里显式填的基准坐标优先于观测文件里的概略坐标（RINEX 表头 APPROX POSITION
        // 常是米级单点解，直接当基准会让双差残差劣化到无法固定）。GUI 两条通路都会设，
        // 只漏了本 CLI 文件模式，等于 ini 的 base_coord_x/y/z 长期被忽略。
        if (Eigen::Vector3d(cfg.baseCoordX, cfg.baseCoordY, cfg.baseCoordZ).norm() > 1e3)
            rtk.mBaseXyzUser = Eigen::Vector3d(cfg.baseCoordX, cfg.baseCoordY, cfg.baseCoordZ);

        EphemerisTable navTable;
        bool refInit = false;
        Eigen::Vector3d ref(0, 0, 0);
        auto processOne = [&](const ObsData &base, const ObsData &rover, EphemerisTable *eph) {
            bool ok = false;
            try { ok = rtk.processEpoch(base, rover, *eph); } catch (...) { ok = false; }
            if (!ok) return;
            if (const Result &r = rtk.result; r.numSats <= 0) return;
            Solvers::Epoch data;
            data.getFromObs(rover);
            data.getFromRTK(rtk);
            if (!refInit && rtk.mBaseXyz.squaredNorm() > 1e6) {
                ref = rtk.mBaseXyz;
                refInit = true;
            }
            epochs.push_back(data);
        };

        const EpochDecimator dec(cfg.sampleInterval);
        std::string pipeErr;
        const int nPair = Solvers::RTK::pairFiles(
            cfg.basePath, cfg.roverPath, cfg.navPath, cfg.sampleInterval, navTable,
            [] { return false; },
            [&](const ObsData &base, const ObsData &rover,
                const std::shared_ptr<EphemerisTable> &eph, EphemerisTable *nav) {
                if (!dec.accept(rover.weekSecond.sow)) return;
                processOne(base, rover, eph ? eph.get() : nav);
            }, pipeErr);
        if (nPair < 0) {
            std::cerr << "RTK " << pipeErr << "\n";
            Cli::dumpEmpty(Solvers::Mode::RTK, cfg);
            return;
        }

        if (cfg.hasRef) ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
        Cli::dumpCSV(epochs, Solvers::Mode::RTK, ref, false, cfg);
    }

    void runRtkRealtime(const SolverOptions &cfg) {
        std::vector<Solvers::Epoch> epochs;

        RTK rtk;
        rtk.applyOptions(cfg);

        if (Eigen::Vector3d(cfg.baseCoordX, cfg.baseCoordY, cfg.baseCoordZ).norm() > 1e3)
            rtk.mBaseXyzUser = Eigen::Vector3d(cfg.baseCoordX, cfg.baseCoordY, cfg.baseCoordZ);

        const std::string &bip = cfg.baseIp.empty() ? "127.0.0.1" : cfg.baseIp;
        const std::string &rip = cfg.roverIp.empty() ? "127.0.0.1" : cfg.roverIp;
        const int bport = cfg.basePort > 0 ? cfg.basePort : 7190;
        const int rport = cfg.roverPort > 0 ? cfg.roverPort : 7201;

        NovAtel::SocketReader baseRdr, roverRdr;
        if (!baseRdr.connect(bip, static_cast<unsigned short>(bport)) ||
            !roverRdr.connect(rip, static_cast<unsigned short>(rport))) {
            std::cerr << "[RTK实时] 连接失败 base=" << bip << ":" << bport
                    << " rover=" << rip << ":" << rport << "\n";
            Cli::dumpEmpty(Solvers::Mode::RTK, cfg);
            return;
        }
        std::cerr << "[RTK实时] 已连接 base=" << bip << ":" << bport
                << " rover=" << rip << ":" << rport << "\n";
        baseRdr.setReceiveTimeout(200);
        roverRdr.setReceiveTimeout(200);


        const auto tStart = std::chrono::steady_clock::now();
        auto timedOut = [&] {
            return cfg.realtimeSec > 0 &&
                   std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - tStart).count() >= cfg.realtimeSec;
        };

        Observer::DualPair pairing;
        pairing.setInterval(cfg.sampleInterval);
        EpochDecimator dec(cfg.sampleInterval);
        long long nProc = 0, nOk = 0, nAligned = 0;
        long long nBaseEp = 0, nRoverEp = 0, nIdle = 0;
        auto elapsedSec = [&] {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - tStart).count();
        };
        auto heartbeat = [&](const char *side) {
            static auto lastBeat = std::chrono::steady_clock::now();
            const auto now = std::chrono::steady_clock::now();
            if (now - lastBeat < std::chrono::seconds(10)) return;
            lastBeat = now;
            std::cerr << "[RTK实时] t=" << static_cast<int>(elapsedSec()) << "s base历元=" << nBaseEp
                    << " rover历元=" << nRoverEp << " 空轮询=" << nIdle
                    << " 配对=" << nAligned << " 解算=" << nProc << " (poll=" << side << ")\n";
        };
        auto getBase = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> int {
            heartbeat("base");
            if (!baseRdr.getNextEpoch(o, false)) {
                if (!baseRdr.isConnected()) {
                    std::cerr << "[RTK实时] base 连接断开 (t=" << static_cast<int>(elapsedSec()) << "s)\n";
                    return -1;
                }
                ++nIdle;
                return 0;
            }
            ++nBaseEp;
            e = baseRdr.ephSnapshot();
            return 1;
        };
        auto getRover = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> int {
            heartbeat("rover");
            if (!roverRdr.getNextEpoch(o, false)) {
                if (!roverRdr.isConnected()) {
                    std::cerr << "[RTK实时] rover 连接断开 (t=" << static_cast<int>(elapsedSec()) << "s)\n";
                    return -1;
                }
                ++nIdle;
                return 0;
            }
            ++nRoverEp;
            e = roverRdr.ephSnapshot();
            return 1;
        };
        Solvers::RTK::pump(
            pairing, getBase, getRover, [&] { return timedOut(); },
            [&] {
                if (cfg.maxEpochs > 0 && static_cast<int>(epochs.size()) >= cfg.maxEpochs) return false;
                ++nAligned;
                return true;
            },
            [&](const ObsData &b, const ObsData &r, EphemerisTable *eph) -> bool {
                if (!dec.accept(r.weekSecond.sow)) return true;
                ++nProc;
                bool ok = false;
                try { ok = rtk.processEpoch(b, r, *eph); } catch (const std::exception &e) {
                    std::cerr << "[RTK实时] processEpoch 异常: " << e.what() << "\n";
                    ok = false;
                } catch (...) { ok = false; }
                if (ok) {
                    const Result &res = rtk.result;
                    if (res.numSats > 0) {
                        Solvers::Epoch data;
                        data.getFromObs(r);
                        data.getFromRTK(rtk);
                        epochs.push_back(data);
                        ++nOk;
                    }
                    if (nProc % 20 == 0) {
                        std::cerr << "[RTK实时] #" << nProc
                                << " stat=" << (res.stat == 1 ? "FIX" : res.stat == 2 ? "FLOAT" : "INV")
                                << " ratio=" << res.ratio
                                << " sats=" << res.numSats
                                << " sigF=" << res.sigmaPFloat
                                << " tol=" << pairing.pairWindow() << "s\n";
                    }
                }
                return true;
            });
        std::cerr << "[RTK实时] 对齐尝试=" << nAligned << " process=" << nProc << " 有效历元=" << epochs.size()
                << " (base/rover 连接=" << bip << ":" << bport << "/" << rip << ":" << rport << ")\n";

        Eigen::Vector3d ref(0, 0, 0);
        if (cfg.hasRef) ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
        else Solvers::Driver::firstSolvedRef(epochs, ref);
        Cli::dumpCSV(epochs, Solvers::Mode::RTK, ref, false, cfg);
    }
}

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "rtk.ini";
    IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolverOptions cfg = loadConfig(f, Solvers::Mode::RTK);
    Log::init("cli_gnsslab.log");
    std::cout << "[RTK] base=" << cfg.basePath << " rover=" << cfg.roverPath
            << " nav=" << (cfg.navPath.empty() ? "(无)" : cfg.navPath) << "\n";
    if (cfg.source == "realtime") runRtkRealtime(cfg);
    else runRtk(cfg);
    return 0;
}
