// RTK 无 GUI 命令行解算（短基线双差非组合）。用法：rtk_cli [ini文件]
// 默认读取同目录 rtk.ini。
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Const.h"              // PI
#include "Exception.h"         // EndOfFile / SyncException
#include "GnssStruct.h"        // ObsData / EphemerisTable
#include "SolverTask.h"        // GnssTask::EpochData / SolverMode
#include "RTK.h"
#include "RinexObsReader.h"
#include "RinexNavStore.h"
#include "OEM7Reader.h"
#include "Log.h"
#include "core/SolveConfig.h"
#include "CliCommon.h"
#include "SolveDriver.h"        // 共享解算编排：fillFromObs / feedRtk（CLI 与 GUI 共用）
#include "core/IniFile.h"

namespace {

void runRtk(const SolveConfig &cfg) {
    std::vector<GnssTask::EpochData> epochs;

    RTK rtk;
    RTK::Config rc;
    rc.cutoffElevRad = cfg.cutoffDeg * PI / 180.0;
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
    const bool oem7 = Cli::isOem7Path(cfg.basePath) || Cli::isOem7Path(cfg.roverPath);
    bool refInit = false;
    Vector3d ref(0, 0, 0);

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
        auto nextBase = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> bool {
            if (!br.getNextEpoch(o)) return false;
            e = br.ephSnapshot();
            return true;
        };
        auto nextRover = [&](ObsData &o) -> bool { return rr.getNextEpoch(o); };

        const double tol = 0.001;
        ObsData base, rover;
        std::shared_ptr<EphemerisTable> baseEph;
        if (!nextBase(base, baseEph)) { std::cerr << "RTK base 无历元\n"; Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg); return; }
        if (!nextRover(rover))        { std::cerr << "RTK rover 无历元\n"; Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg); return; }
        bool roverExhausted = false;
        while (true) {
            while (!roverExhausted && rover.epoch < base.epoch) {
                if (!nextRover(rover)) { roverExhausted = true; break; }
            }
            if (roverExhausted) break;
            if (rover.epoch > base.epoch + tol) {
                if (!nextBase(base, baseEph)) break;
                continue;
            }
            EphemerisTable *eph = baseEph ? baseEph.get() : &navTable;
            processOne(base, rover, eph);
            if (!nextBase(base, baseEph)) break;
            if (!nextRover(rover)) { roverExhausted = true; break; }
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

        ObsData base;
        try { base = baseRdr.parseRinexObs(); }
        catch (...) { std::cerr << "RTK 基准站 RINEX 解析失败\n"; Cli::dumpEmpty(GnssTask::SolverMode::Rtk, cfg); return; }

        // 把基准站流跳到流动站首历元附近（与 GUI 一致的对齐加速）
        try {
            RinexObsReader probeRdr;
            std::fstream probeFile(cfg.roverPath, std::ios::in);
            if (probeFile.is_open()) {
                probeRdr.pFileStream = &probeFile;
                const ObsData rover0 = probeRdr.parseRinexObs();
                baseRdr.fastForwardToEpoch(rover0.epoch);
            }
        } catch (...) { /* 探测失败退回逐历元 */ }

        while (true) {
            ObsData rover;
            bool got = false;
            try { rover = roverRdr.parseRinexObs(base.epoch); got = true; }
            catch (const SyncException &) {
                try { base = baseRdr.parseRinexObs(); }
                catch (const EndOfFile &) { break; } catch (...) { break; }
                continue;
            } catch (const EndOfFile &) { break; } catch (...) { break; }
            if (got) {
                processOne(base, rover, &navTable);
                try { base = baseRdr.parseRinexObs(); }
                catch (const EndOfFile &) { break; } catch (...) { break; }
            }
        }
    }

    if (cfg.hasRef) ref = Vector3d(cfg.refX, cfg.refY, cfg.refZ);
    Cli::dumpCsv(epochs, GnssTask::SolverMode::Rtk, ref, false, cfg);
}

} // namespace

int main(int argc, char **argv) {
    std::string ini = (argc > 1) ? argv[1] : "rtk.ini";
    IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    SolveConfig cfg = loadConfig(f);
    Log::init("cli_gnsslab.log");
    std::cout << "[RTK] base=" << cfg.basePath << " rover=" << cfg.roverPath
              << " nav=" << (cfg.navPath.empty() ? "(无)" : cfg.navPath) << "\n";
    runRtk(cfg);
    return 0;
}
