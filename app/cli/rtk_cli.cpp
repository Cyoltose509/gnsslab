#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
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
#include "SolverTask.h"
#include "RTK.h"
#include "OEM7Reader.h"
#include "ObsFormat.h"
#include "OEM7SocketReader.h"
#include "Log.h"
#include "SolverOptions.h"
#include "CliCommon.h"
#include "SolveDriver.h"
#include "core/DualObsPairing.h"
#include "core/IniFile.h"

namespace {
    void runRtk(const SolverOptions &cfg) {
        std::vector<GnssTask::EpochData> epochs;

        RTK rtk;
        rtk.applyOptions(cfg);

        if (cfg.basePath.empty() || cfg.roverPath.empty()) {
            std::cerr << "RTK 需要基准站与流动站 [input] base / rover\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
            return;
        }

        EphemerisTable navTable;
        const GnssTask::ObsFormat fmtBase = GnssTask::classifyObsPath(cfg.basePath);
        const GnssTask::ObsFormat fmtRover = GnssTask::classifyObsPath(cfg.roverPath);
        if (fmtBase == GnssTask::ObsFormat::Unknown || fmtRover == GnssTask::ObsFormat::Unknown) {
            std::cerr << "RTK 无法识别观测文件格式\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
            return;
        }
        const bool baseRinex = fmtBase == GnssTask::ObsFormat::Rinex;
        const bool roverRinex = fmtRover == GnssTask::ObsFormat::Rinex;
        const bool anyRinex = baseRinex || roverRinex;
        bool refInit = false;
        Eigen::Vector3d ref(0, 0, 0);

        auto processOne = [&](const ObsData &base, const ObsData &rover, EphemerisTable *eph) {
            bool ok = false;
            try { ok = rtk.processEpoch(base, rover, *eph); } catch (...) { ok = false; }
            if (!ok) return;
            if (const Result &r = rtk.result; r.numSats <= 0) return;
            GnssTask::EpochData data;
            SolveDriver::fillFromObs(data, rover);
            SolveDriver::feedRtk(data, rtk);
            if (!refInit && rtk.mBaseXyz.squaredNorm() > 1e6) {
                ref = rtk.mBaseXyz;
                refInit = true;
            }
            epochs.push_back(data);
        };


        if (anyRinex) {
            if (cfg.navPath.empty()) {
                std::cerr << "RTK RINEX 模式需要广播星历 [input] nav（任一观测文件为 RINEX 即需要）\n";
                Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
                return;
            }
            if (std::string navErr; !SolveDriver::loadRtkNav(cfg.navPath, navTable, navErr)) {
                std::cerr << "RTK " << navErr << "\n";
                Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
                return;
            }
        }

        SolveDriver::DualObsHandles h; //NOLINT
        SolveDriver::DualObsPairing::Puller nextBase, nextRover;
        std::string openErr; //NOLINT
        if (!SolveDriver::openDualObsSources(baseRinex, roverRinex, cfg.basePath, cfg.roverPath,
                                             h, nextBase, nextRover, openErr)) {
            std::cerr << "RTK " << openErr << "\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
            return;
        }

        SolveDriver::DualObsPairing p(nextBase, nextRover);
        p.setInterval(cfg.sampleInterval); // 声明的标称采样间隔：配对容差免窗口学习期
        SolveDriver::EpochDecimator dec(cfg.sampleInterval); // 解算抽取：只解算网格历元
        SolveDriver::DualObsPairing::Pair pr;
        while (p.next(pr)) {
            if (!dec.accept(pr.rover.weekSecond.sow)) continue;
            EphemerisTable *eph = pr.baseEph
                                      ? pr.baseEph.get()
                                      : pr.roverEph
                                            ? pr.roverEph.get()
                                            : &navTable;
            processOne(pr.base, pr.rover, eph);
        }

        if (cfg.hasRef) ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
        Cli::dumpCsv(epochs, GnssTask::SolverMode::Rtk, ref, false, cfg);
    }

    void runRtkRealtime(const SolverOptions &cfg) {
        std::vector<GnssTask::EpochData> epochs;

        RTK rtk;
        rtk.applyOptions(cfg);

        if (Eigen::Vector3d(cfg.baseCoordX, cfg.baseCoordY, cfg.baseCoordZ).norm() > 1e3)
            rtk.mBaseXyzUser = Eigen::Vector3d(cfg.baseCoordX, cfg.baseCoordY, cfg.baseCoordZ);

        const std::string &bip = cfg.baseIp.empty() ? "127.0.0.1" : cfg.baseIp;
        const std::string &rip = cfg.roverIp.empty() ? "127.0.0.1" : cfg.roverIp;
        const int bport = cfg.basePort > 0 ? cfg.basePort : 7190;
        const int rport = cfg.roverPort > 0 ? cfg.roverPort : 7201;

        OEM7SocketReader baseRdr, roverRdr;
        if (!baseRdr.connect(bip, static_cast<unsigned short>(bport)) ||
            !roverRdr.connect(rip, static_cast<unsigned short>(rport))) {
            std::cerr << "[RTK实时] 连接失败 base=" << bip << ":" << bport
                    << " rover=" << rip << ":" << rport << "\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
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

        SolveDriver::DualObsPairing pairing;
        pairing.setInterval(cfg.sampleInterval);
        SolveDriver::EpochDecimator dec(cfg.sampleInterval); // 解算抽取：只解算网格历元
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
            if (!baseRdr.getNextEpoch(o, false)) {   // 非阻塞：仅取已缓冲历元，由泵循环并发轮询 base/rover
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
            if (!roverRdr.getNextEpoch(o, false)) {   // 非阻塞：与 base 并发轮询，避免串行等待把 1Hz 压到 0.4Hz
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
        SolveDriver::pumpRealtimePair(
            pairing, getBase, getRover, [&] { return timedOut(); },
            [&] {
                if (cfg.maxEpochs > 0 && static_cast<int>(epochs.size()) >= cfg.maxEpochs) return false;
                ++nAligned;
                return true;
            },
            [&](const ObsData &b, const ObsData &r, EphemerisTable *eph) -> bool {
                if (!dec.accept(r.weekSecond.sow)) return true; // 抽取：非网格历元不解算
                ++nProc;
                bool ok = false;
                try { ok = rtk.processEpoch(b, r, *eph); } catch (const std::exception &e) {
                    std::cerr << "[RTK实时] processEpoch 异常: " << e.what() << "\n";
                    ok = false;
                } catch (...) { ok = false; }
                if (ok) {
                    const Result &res = rtk.result;
                    if (res.numSats > 0) {
                        GnssTask::EpochData data;
                        SolveDriver::fillFromObs(data, r);
                        SolveDriver::feedRtk(data, rtk);
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
        else SolveDriver::firstSolvedRef(epochs, ref);
        Cli::dumpCsv(epochs, GnssTask::SolverMode::Rtk, ref, false, cfg);
    }
}

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "rtk.ini";
    AppConfig::IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolverOptions cfg = loadConfig(f);
    Log::init("cli_gnsslab.log");
    std::cout << "[RTK] base=" << cfg.basePath << " rover=" << cfg.roverPath
            << " nav=" << (cfg.navPath.empty() ? "(无)" : cfg.navPath) << "\n";
    if (cfg.source == "realtime") runRtkRealtime(cfg);
    else runRtk(cfg);
    return 0;
}
