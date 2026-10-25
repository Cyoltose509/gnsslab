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
    // 双站格式必须一致：混合格式（RINEX+OEM7）此前会整队落进 OEM7 分支必败，
    // 且错误信息指不到真正原因，这里显式报错
    const GnssTask::ObsFormat fmtBase = GnssTask::classifyObsPath(cfg.basePath);
    const GnssTask::ObsFormat fmtRover = GnssTask::classifyObsPath(cfg.roverPath);
    if (fmtBase == GnssTask::ObsFormat::Unknown || fmtRover == GnssTask::ObsFormat::Unknown) {
        std::cerr << "RTK 无法识别观测文件格式（支持 RINEX .??O/.rnx/.obs 与 OEM7 .log/.oem7）\n";
        Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
        return;
    }
    if (fmtBase != fmtRover) {
        std::cerr << "RTK 基准站与流动站文件格式不一致（需同为 RINEX 或同为 OEM7）\n";
        Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
        return;
    }
    const bool oem7 = (fmtBase == GnssTask::ObsFormat::Oem7);
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

    if (oem7) {
        OEM7Reader br, rr;
        if (!br.open(cfg.basePath) || !rr.open(cfg.roverPath)) {
            std::cerr << "RTK 无法打开 OEM7 日志\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
            return;
        }
        // 双站历元配对统一走 SolveDriver::DualObsPairing：配对窗口自适应采样间隔
        // （= 半个采样间隔，1Hz/5Hz/10Hz 通吃；样本不足时 tol=0 只推进不配对，绝不错配相邻历元）。
        auto nextBase = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> bool {
            if (!br.getNextEpoch(o)) return false;
            e = br.ephSnapshot();
            return true;
        };
        auto nextRover = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> bool {
            if (!rr.getNextEpoch(o)) return false;
            e = rr.ephSnapshot();
            return true;
        };
        SolveDriver::DualObsPairing p(nextBase, nextRover);
        SolveDriver::DualObsPairing::Pair pr;
        while (p.next(pr)) {
            EphemerisTable *eph = SolveDriver::DualObsPairing::selectEph(pr.baseEph, pr.roverEph);
            processOne(pr.base, pr.rover, eph);
        }
    } else {
        if (cfg.navPath.empty()) {
            std::cerr << "RTK RINEX 模式需要广播星历 [input] nav\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
            return;
        }
        RinexNavStore nav;
        nav.loadFile(cfg.navPath, navTable);
        if (navTable.gps.empty() && navTable.bds.empty()) {
            std::cerr << "RTK 广播星历为空\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
            return;
        }
        std::fstream baseFile(cfg.basePath, std::ios::in);
        std::fstream roverFile(cfg.roverPath, std::ios::in);
        if (!baseFile.is_open() || !roverFile.is_open()) {
            std::cerr << "RTK 无法打开 RINEX 观测文件\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg);
            return;
        }
        RinexObsReader baseRdr, roverRdr;
        baseRdr.pFileStream = &baseFile;
        roverRdr.pFileStream = &roverFile;

        // 失败快出：基准站首历元解析（同时校验文件可用性）
        try { baseRdr.parseRinexObs(); }
        catch (...) { std::cerr << "RTK 基准站 RINEX 解析失败\n"; Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg); return; }

        // 把基准站流跳到流动站首历元附近（与 GUI 一致的对齐加速；保留首跳优化）
        try {
            RinexObsReader probeRdr;
            std::fstream probeFile(cfg.roverPath, std::ios::in);
            if (probeFile.is_open()) {
                probeRdr.pFileStream = &probeFile;
                const ObsData rover0 = probeRdr.parseRinexObs();
                baseRdr.fastForwardToEpoch(rover0.epoch);
            }
        } catch (...) { /* 探测失败退回逐历元 */ }

        // 双站配对统一走 SolveDriver::DualObsPairing：两路各自独立取下一历元（无参
        // parseRinexObs），由配对器按时间对齐；首跳加速已在上面完成。RINEX 用全量 navTable。
        auto nextBase = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> bool {
            try { o = baseRdr.parseRinexObs(); }
            catch (const EndOfFile &) { return false; } catch (...) { return false; }
            e = nullptr;
            return true;
        };
        auto nextRover = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> bool {
            try { o = roverRdr.parseRinexObs(); }
            catch (const EndOfFile &) { return false; } catch (...) { return false; }
            e = nullptr;
            return true;
        };
        SolveDriver::DualObsPairing p(nextBase, nextRover);
        SolveDriver::DualObsPairing::Pair pr;
        while (p.next(pr)) {
            processOne(pr.base, pr.rover, &navTable);
        }
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

    // 配对窗口自适应采样间隔（原理见 app/core/DualObsPairing.h 的内部采样间隔估计）：
    // 1Hz/5Hz/10Hz 通吃，未就绪时返回 0（只推进不配对），绝不会错配相邻历元。
    // 实时路径只持有一个 DualObsPairing 实例，通过 observeBase/observeRover/pairWindow 完成配对判定，
    // 不再出现 IntervalEstimator 类型名。
    SolveDriver::DualObsPairing pairing;
    ObsData base, rover;
    std::shared_ptr<EphemerisTable> baseEph, roverEph;
    bool haveBase = false, haveRover = false;
    long long nProc = 0, nOk = 0, nAligned = 0;
    while (true) {
        if (cfg.maxEpochs > 0 && static_cast<int>(epochs.size()) >= cfg.maxEpochs) break;
        if (!haveBase) {
            ObsData b;
            if (baseRdr.getNextEpoch(b)) { base = std::move(b); baseEph = baseRdr.ephSnapshot(); pairing.observeBase(base.weekSecond.sow); haveBase = true; }
            else if (!baseRdr.isConnected()) break;
        }
        if (!haveRover) {
            ObsData r;
            if (roverRdr.getNextEpoch(r)) { rover = std::move(r); roverEph = roverRdr.ephSnapshot(); pairing.observeRover(rover.weekSecond.sow); haveRover = true; }
            else if (!roverRdr.isConnected()) break;
        }
        if (!haveBase || !haveRover) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        ++nAligned;
        // 配对判定与星历选择统一走 SolveDriver::DualObsPairing（与文件模式同一条逻辑）。
        // 实时路径是"轮询+超时+断连重连"，IO 形态与文件不同，故保留自身 haveBase/haveRover
        // 轮询结构，仅复用配对器暴露的 classify / selectEph。
        const double tol = pairing.pairWindow();
        const auto act = SolveDriver::DualObsPairing::classify(base.epoch, rover.epoch, tol);
        if (act == SolveDriver::DualObsPairing::Action::Pair) {
            EphemerisTable *eph = SolveDriver::DualObsPairing::selectEph(baseEph, roverEph);
            ++nProc;
            bool ok = false;
            try { ok = rtk.processEpoch(base, rover, *eph); }
            catch (const std::exception &e) { std::cerr << "[RTK实时] processEpoch 异常: " << e.what() << "\n"; ok = false; }
            catch (...) { ok = false; }
            if (ok) {
                const Result &res = rtk.result;
                if (res.numSats > 0) {
                    GnssTask::EpochData data;
                    SolveDriver::fillFromObs(data, rover);
                    SolveDriver::feedRtk(data, rtk);
                    epochs.push_back(data);
                    ++nOk;
                }
            }
            haveBase = haveRover = false;
        } else if (act == SolveDriver::DualObsPairing::Action::AdvanceBase) {
            haveBase = false;    // 基准站历元过时，丢弃重读
        } else {
            haveRover = false;   // 流动站历元过时，丢弃重读
        }
    }
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
