// LEO 无 GUI 命令行定轨（运动学；可选参考轨道评估 RTN）。用法：leo_cli [ini文件]
// 默认读取同目录 leo.ini。
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Exception.h"         // EndOfFile
#include "GnssStruct.h"        // ObsData / PVT / SatID / Vector3d
#include "CoordConvert.h"      // computeRTN
#include "SolverTask.h"        // GnssTask::EpochData / SolverMode
#include "SPP.h"
#include "LEO.h"
#include "RinexObsReader.h"
#include "Sp3OrbitReader.h"
#include "Log.h"
#include "core/SolveConfig.h"
#include "CliCommon.h"
#include "SolveDriver.h"        // 共享解算编排：ObsSource / solveStationStream / fillFrom*
#include "core/IniFile.h"

namespace {

void runLeo(const SolveConfig &cfg) {
    std::vector<GnssTask::EpochData> epochs;

    if (cfg.obsPath.empty()) {
        std::cerr << "LEO 需要观测文件 [input] obs\n";
        Cli::dumpEmpty(GnssTask::SolverMode::Leo, cfg);
        return;
    }
    SolveDriver::RinexFileSource src;
    if (!src.open(cfg.obsPath)) {
        std::cerr << "LEO 无法打开 RINEX: " << cfg.obsPath << "\n";
        Cli::dumpEmpty(GnssTask::SolverMode::Leo, cfg);
        return;
    }

    // 参考轨道读取器（可选）
    Sp3OrbitReader refRdr;
    bool hasRefOrbit = false;
    SatID refSat('L', cfg.refPrn);
    int refPrn = cfg.refPrn;
    if (!cfg.refSp3.empty()) {
        refRdr.read(cfg.refSp3);
        bool refFound = false;
        for (const auto &[sat, _] : refRdr.getData())
            if (sat.system == 'L') { refPrn = sat.id; refFound = true; break; }
        if (!refFound && !refRdr.getData().empty()) refPrn = refRdr.getData().begin()->first.id;
        refSat = SatID('L', refPrn);
        hasRefOrbit = true;
    }

    LEO leo;
    leo.setCutoffElevDeg(cfg.cutoffDeg);
    leo.setEnabledSystems(cfg.systems);
    leo.mApplySatPCO = cfg.satPCO != 0;
    leo.mApplyPcv   = cfg.satPCV != 0;
    leo.mApplySolidTide = cfg.solidTide != 0;
    leo.setMeasNoise(cfg.sigIFCode, cfg.sigPhase);
    leo.mMotionThresh = cfg.motionThresh;
    leo.mSppVsPredThresh = cfg.sppVsPredThresh;

    if (!cfg.gnssSp3.empty())  leo.loadSp3(cfg.gnssSp3);
    if (!cfg.gnssClk.empty())  leo.loadClk(cfg.gnssClk, cfg.systems);
    if (!cfg.osb.empty())      leo.loadOsb(cfg.osb);
    if (!cfg.atx.empty())      leo.loadAtx(cfg.atx);
    if (!cfg.gnssBrdc.empty()) leo.loadBrdc(cfg.gnssBrdc);
    if (hasRefOrbit) {
        if (!leo.loadLeoReference(cfg.refSp3, refSat)) {
            std::cerr << "LEO loadLeoReference 失败\n";
            hasRefOrbit = false;
        }
    }
    if (!cfg.aux.empty()) leo.loadAuxFile(cfg.aux);

    bool refInit = false;
    Vector3d ref(0, 0, 0);
    bool leoRefInit = false;
    Vector3d leoRef(0, 0, 0);

    // LEO 特有：位置合理性校验 + 参考轨道 RTN 标注。通用的解算与回填已由 solveStationStream 完成。
    SolveDriver::solveStationStream(src, leo,
        [&](LEO &, GnssTask::ObsItem &item, GnssTask::EpochData &data) {
            const Vector3d leoPos = data.sppResult.xyz;
            if (!data.solved || leoPos.squaredNorm() <= 1e12) {  // 未解出 或 非合理 ECEF 位置
                data.solved = false;
                epochs.push_back(std::move(data));
                return true;
            }
            if (hasRefOrbit) {
                const PVT refPVT = refRdr.getPVT(refSat, item.obs.epoch);
                // 与 GUI(GuiLeoProcessor)一致：参考轨道位置合理(|p|>1e6m)时采用，否则退化为 LEO 解作参考
                const Vector3d refPos = refPVT.p.squaredNorm() > 1e12 ? refPVT.p : leoPos;
                const RtnComponents rtn = computeRTN(leoPos, refPos, refPVT.v);
                data.refECEF = refPos;
                data.rtnR = rtn.R;
                data.rtnT = rtn.T;
                data.rtnN = rtn.N;
                if (!leoRefInit) { leoRef = refPos; leoRefInit = true; }
            }
            if (!refInit) { ref = leoPos; refInit = true; }
            epochs.push_back(std::move(data));
            return true;
        });

    if (cfg.hasRef) { ref = Vector3d(cfg.refX, cfg.refY, cfg.refZ); leoRef = ref; }
    Cli::dumpCsv(epochs, GnssTask::SolverMode::Leo, hasRefOrbit ? leoRef : ref, hasRefOrbit, cfg);
}

} // namespace

int main(int argc, char **argv) {
    std::string ini = (argc > 1) ? argv[1] : "leo.ini";
    IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    SolveConfig cfg = loadConfig(f);
    Log::init("cli_gnsslab.log");
    std::cout << "[LEO] obs=" << cfg.obsPath
              << " 参考轨道=" << (cfg.refSp3.empty() ? "(无)" : cfg.refSp3) << "\n";
    runLeo(cfg);
    return 0;
}
