
#include <iostream>
#include <string>
#include <vector>

#include "Exception.h"
#include "GnssStruct.h"
#include "CoordConvert.h"
#include "SolverTask.h"
#include "SPP.h"
#include "LEO.h"
#include "ObsFormat.h"      // classifyObsPath：RINEX/OEM7 三值判别（Unknown 显式报错）

#include "Sp3OrbitReader.h"
#include "Log.h"
#include "core/SolveConfig.h"
#include "CliCommon.h"
#include "SolveDriver.h"
#include "core/IniFile.h"

namespace {

void runLeo(const SolveConfig &cfg) {
    std::vector<GnssTask::EpochData> epochs;

    if (cfg.obsPath.empty()) {
        std::cerr << "LEO 需要观测文件 [input] obs\n";
        Cli::dumpEmpty(GnssTask::SolverMode::Leo, cfg);
        return;
    }
    // 观测文件格式三值判别（与 SPP/RTK/PPP 同一入口）：RINEX 或 OEM7 日志，
    // 无法识别时明确报错，不再默认落进 RINEX 读取器报"无法打开 RINEX"误导用户
    SolveDriver::RinexFileSource rsrc;
    SolveDriver::Oem7FileSource osrc;
    SolveDriver::ObsSource *src = nullptr;
    const auto fmt = GnssTask::classifyObsPath(cfg.obsPath);
    if (fmt == GnssTask::ObsFormat::Oem7) {
        if (!osrc.open(cfg.obsPath)) {
            std::cerr << "LEO 无法打开 OEM7: " << cfg.obsPath << "\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Leo, cfg);
            return;
        }
        src = &osrc;
    } else if (fmt == GnssTask::ObsFormat::Rinex) {
        if (!rsrc.open(cfg.obsPath)) {
            std::cerr << "LEO 无法打开 RINEX: " << cfg.obsPath << "\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Leo, cfg);
            return;
        }
        src = &rsrc;
    } else {
        std::cerr << "LEO 无法识别观测文件格式: "
                  << cfg.obsPath << "\n";
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
    leo.mApplySolidTide = cfg.solidTide != 0; // 卫星/接收机 PCO、PCV 现已由 ATX 文件自动决定
    leo.setMeasNoise(cfg.sigIFCode, cfg.sigPhase);
    // 位置过程噪声：此前 leo_cli 漏下发，ini 里 pos_noise 被静默忽略（A/B 全同值）。
    // SolveConfig 默认 100 与 PPPKinematic/LEO 的基类默认一致，故不写该键时行为不变。
    leo.setPosProcNoise(cfg.posNoise);
    leo.setKeepAmbMaxGap(cfg.keepAmbMaxGap);
    leo.setIsbProcNoise(cfg.isbProcNoise, cfg.isbInitVar); // ISB：非参考系统钟差当常数/随机游走估(对齐老师)
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
    Eigen::Vector3d ref(0, 0, 0);
    bool leoRefInit = false;
    Eigen::Vector3d leoRef(0, 0, 0);

    // LEO 特有：位置合理性校验 + 参考轨道 RTN 标注。通用的解算与回填已由 solveStationStream 完成。
    SolveDriver::solveStationStream(*src, leo,
        [&](LEO &, const GnssTask::ObsItem &item, GnssTask::EpochData &data) {
            const Eigen::Vector3d leoPos = data.sppResult.xyz;
            if (!data.solved || leoPos.squaredNorm() <= 1e12) {  // 未解出 或 非合理 ECEF 位置
                data.solved = false;
                epochs.push_back(std::move(data));
                return true;
            }
            if (hasRefOrbit) {
                const PVT refPVT = refRdr.getPVT(refSat, item.obs.epoch);
                const Eigen::Vector3d refPos = refPVT.p.squaredNorm() > 1e12 ? refPVT.p : leoPos;
                const Eigen::Vector3d rtn = XYZToRTN(leoPos, refPos, refPVT.v);
                data.refECEF = refPos;
                data.rtnR = rtn(0);
                data.rtnT = rtn(1);
                data.rtnN = rtn(2);
                if (!leoRefInit) { leoRef = refPos; leoRefInit = true; }
            }
            if (!refInit) { ref = leoPos; refInit = true; }
            epochs.push_back(std::move(data));
            return true;
        });

    if (cfg.hasRef) { ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ); leoRef = ref; }
    Cli::dumpCsv(epochs, GnssTask::SolverMode::Leo, hasRefOrbit ? leoRef : ref, hasRefOrbit, cfg);
}

} // namespace

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "leo.ini";
    AppConfig::IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolveConfig cfg = loadConfig(f);
    Log::init("cli_gnsslab.log");
    std::cout << "[LEO] obs=" << cfg.obsPath
              << " 参考轨道=" << (cfg.refSp3.empty() ? "(无)" : cfg.refSp3) << "\n";
    runLeo(cfg);
    return 0;
}
