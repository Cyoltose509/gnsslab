// RTK 无 GUI 命令行解算（短基线双差非组合）。用法：rtk_cli [ini文件]
// 默认读取同目录 rtk.ini。
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>   // 必须先于 <vector>/OEM7SocketReader 引入，避免 std::byte 与 Windows byte 歧义
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "Const.h"              // PI
#include "Exception.h"         // EndOfFile / SyncException
#include "GnssStruct.h"        // ObsData / EphemerisTable
#include "SolverTask.h"        // GnssTask::EpochData / SolverMode
#include "RTK.h"
#include "RinexObsReader.h"
#include "RinexNavStore.h"
#include "OEM7Reader.h"
#include "ObsFormat.h"      // classifyObsPath：双站格式一致性与分发判别
#include "OEM7SocketReader.h"
#include "Log.h"
#include "core/SolveConfig.h"
#include "CliCommon.h"
#include "SolveDriver.h"        // 共享解算编排：fillFromObs / feedRtk（CLI 与 GUI 共用）
#include "core/DualObsPairing.h" // 双站历元配对统一抽象（CLI 与 GUI 共用）
#include "core/IniFile.h"

namespace {

void runRtk(const SolveConfig &cfg) {
    std::vector<GnssTask::EpochData> epochs;

    RTK rtk;
    RTK::Config rc;
    rc.cutoffElevRad = cfg.cutoffDeg * DEG_TO_RAD;
    rc.enabledSystems = cfg.systems;
    rc.useAR = cfg.useAR != 0;
    rc.useCycleSlip = cfg.useCycleSlip != 0;
    rc.useGfCycleSlip = cfg.useGfCycleSlip != 0;
    rc.fullCovariance = cfg.fullCovariance != 0;
    rtk.setConfig(rc);

    if (cfg.basePath.empty() || cfg.roverPath.empty()) {
        std::cerr << "RTK 需要基准站与流动站 [input] base / rover\n";
        Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
        return;
    }

    EphemerisTable navTable;
    // 双站格式独立判别：混合格式（一 RINEX 一 OEM7）合法——两路 reader 各自构造、
    // 星历按侧分发（OEM7 内嵌快照优先，纯 RINEX 回落 navTable）。
    // 早期"全局 oem7 标志 + 成对 reader"实现曾强制两站同格式一刀切报错，
    // 但配对接口（DualObsPairing::Puller / selectEph）本就与格式无关，此处放开。
    const GnssTask::ObsFormat fmtBase = GnssTask::classifyObsPath(cfg.basePath);
    const GnssTask::ObsFormat fmtRover = GnssTask::classifyObsPath(cfg.roverPath);
    if (fmtBase == GnssTask::ObsFormat::Unknown || fmtRover == GnssTask::ObsFormat::Unknown) {
        std::cerr << "RTK 无法识别观测文件格式\n";
        Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
        return;
    }
    const bool baseRinex = fmtBase == GnssTask::ObsFormat::Rinex;
    const bool roverRinex = fmtRover == GnssTask::ObsFormat::Rinex;
    const bool anyRinex = baseRinex || roverRinex; // 任一侧 RINEX 即需外部广播星历
    bool refInit = false;
    Eigen::Vector3d ref(0, 0, 0);

    auto processOne = [&](const ObsData &base, const ObsData &rover, EphemerisTable *eph) {
        bool ok = false;
        try { ok = rtk.processEpoch(base, rover, *eph); } catch (...) { ok = false; }
        if (!ok) return;
        const Result &r = rtk.result;
        if (r.numSats <= 0) return;
        GnssTask::EpochData data;
        SolveDriver::fillFromObs(data, rover);
        SolveDriver::feedRtk(data, rtk);
        if (!refInit && rtk.mBaseXyz.squaredNorm() > 1e6) { ref = rtk.mBaseXyz; refInit = true; }
        epochs.push_back(data);
    };

    // 任一侧为 RINEX 即加载外部广播星历（OEM7 侧有内嵌快照可自足，不强制 nav）
    if (anyRinex) {
        if (cfg.navPath.empty()) {
            std::cerr << "RTK RINEX 模式需要广播星历 [input] nav（任一观测文件为 RINEX 即需要）\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
            return;
        }
        std::string navErr;
        if (!SolveDriver::loadRtkNav(cfg.navPath, navTable, navErr)) {
            std::cerr << "RTK " << navErr << "\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
            return;
        }
    }

    // ---- per-side reader + 首跳快进对齐 + 配对器：与 GUI ReaderThread 共用同一套
    // （原两端各写一份 makePull/probeRinexFirst/快进，近 90 行逐字相同）----
    SolveDriver::DualObsHandles h;
    SolveDriver::DualObsPairing::Puller nextBase, nextRover;
    std::string openErr;
    if (!SolveDriver::openDualObsSources(baseRinex, roverRinex, cfg.basePath, cfg.roverPath,
                                         h, nextBase, nextRover, openErr)) {
        std::cerr << "RTK " << openErr << "\n";
        Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
        return;
    }

    // ---- 配对解算：星历按侧分发（base 快照优先 → rover 快照 → navTable，与 selectEph 语义一致）----
    // 双站历元配对统一走 SolveDriver::DualObsPairing：配对窗口自适应采样间隔
    // （= 半个采样间隔，1Hz/5Hz/10Hz 通吃；样本不足时 tol=0 只推进不配对，绝不错配相邻历元）。
    SolveDriver::DualObsPairing p(nextBase, nextRover);
    SolveDriver::DualObsPairing::Pair pr;
    while (p.next(pr)) {
        EphemerisTable *eph = pr.baseEph ? pr.baseEph.get()
                          : pr.roverEph ? pr.roverEph.get()
                          : &navTable;
        processOne(pr.base, pr.rover, eph);
    }

    if (cfg.hasRef) ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
    Cli::dumpCsv(epochs, GnssTask::SolverMode::Rtk, ref, false, cfg);
}

// ---------------------------------------------------------------------------
// RTK（实时流模式：基准站 + 流动站两路 OEM7 socket）
// 与 GUI Solvers::RtkProcessor::SolveRealtimeThread 共用同一套双流对齐 + 双差逻辑，
// 只是这里把结果收进 vector 后一次性 dump（CLI 无增量渲染需求）。
// 两路按历元时间对齐：|Δ|≤tol 才配对解算，否则丢弃较旧的一路重读（与文件模式一致）。
//
// ⚠️ tol 由 DualObsPairing 内部的采样间隔估计自适应给出（= 半个采样间隔），不要写死成常量。两个方向都翻过车：
// 太小（1ms）→ 有恒定钟差的真实双流一个历元都解不出（实测对齐 7345 次、process=0）；
// 太大（1Hz 取 1.0s）→ 相隔整整一个采样间隔的相邻历元被错配，Ratio 恒为 0、
// 固定率 98%→8%。自适应后 1Hz/5Hz/10Hz 通吃，无需用户告知码率。
// ---------------------------------------------------------------------------
void runRtkRealtime(const SolveConfig &cfg) {
    std::vector<GnssTask::EpochData> epochs;

    RTK rtk;
    RTK::Config rc;
    rc.cutoffElevRad = cfg.cutoffDeg * DEG_TO_RAD;
    rc.enabledSystems = cfg.systems;
    rc.useAR = cfg.useAR != 0;
    rc.useCycleSlip = cfg.useCycleSlip != 0;
    rc.useGfCycleSlip = cfg.useGfCycleSlip != 0;
    rc.fullCovariance = cfg.fullCovariance != 0;
    rtk.setConfig(rc);

    // 与 GUI 一致：显式基站坐标（norm>1e3 才生效，语义同 RTK::processEpoch 的 mBaseXyzUser）
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

    // realtime_sec>0 计时停止（SolveConfig 注释承诺的行为；此前 CLI 实现漏掉，
    // 只认 max_epochs，导致 realtime_sec 单独设置时永不停止）。
    const auto tStart = std::chrono::steady_clock::now();
    auto timedOut = [&] {
        return cfg.realtimeSec > 0 &&
               std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - tStart).count() >= cfg.realtimeSec;
    };

    // 配对窗口自适应采样间隔（原理见 app/core/DualObsPairing.h 的内部采样间隔估计）：
    // 1Hz/5Hz/10Hz 通吃，未就绪时返回 0（只推进不配对），绝不会错配相邻历元。
    // 实时路径只持有一个 DualObsPairing 实例，通过 observeBase/observeRover/pairWindow 完成配对判定，
    // 不再出现 IntervalEstimator 类型名。
    // 双流轮询 + 配对核心与 GUI SolveRealtimeThread 共用（pumpRealtimePair）；
    // CLI 差异仅在连接失败即退出、历元上限与结果收集（vector + 一次性 dump）。
    SolveDriver::DualObsPairing pairing;
    long long nProc = 0, nOk = 0, nAligned = 0;
    // 实时诊断计数（现场判断"是没数据 / 是判死断线 / 是配对不上"三者的唯一依据）
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
        if (!baseRdr.getNextEpoch(o)) {
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
        if (!roverRdr.getNextEpoch(o)) {
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
            ++nProc;
            bool ok = false;
            try { ok = rtk.processEpoch(b, r, *eph); }
            catch (const std::exception &e) { std::cerr << "[RTK实时] processEpoch 异常: " << e.what() << "\n"; ok = false; }
            catch (...) { ok = false; }
            if (ok) {
                const Result &res = rtk.result;
                if (res.numSats > 0) {
                    GnssTask::EpochData data;
                    SolveDriver::fillFromObs(data, r);
                    SolveDriver::feedRtk(data, rtk);
                    epochs.push_back(data);
                    ++nOk;
                }
                // 诊断进度：每 20 历元打一行状态/ratio/卫星数/σ/配对窗口，
                // 用于现场判断"float 收敛中 / 永不固定 / 历元饿死"。
                if (nProc % 20 == 0)
                    std::cerr << "[RTK实时] #" << nProc
                              << " stat=" << (res.stat == 1 ? "FIX" : res.stat == 2 ? "FLOAT" : "INV")
                              << " ratio=" << res.ratio
                              << " sats=" << res.numSats
                              << " sigF=" << res.sigmaPFloat
                              << " tol=" << pairing.pairWindow() << "s\n";
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

} // namespace

int main(int argc, char **argv) {
    std::string ini = (argc > 1) ? argv[1] : "rtk.ini";
    AppConfig::IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    SolveConfig cfg = loadConfig(f);
    Log::init("cli_gnsslab.log");
    std::cout << "[RTK] base=" << cfg.basePath << " rover=" << cfg.roverPath
              << " nav=" << (cfg.navPath.empty() ? "(无)" : cfg.navPath) << "\n";
    if (cfg.source == "realtime") runRtkRealtime(cfg);
    else runRtk(cfg);
    return 0;
}
