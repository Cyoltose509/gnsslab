#include <iostream>
#include <string>
#include <vector>


#include "SolverTask.h"
#include "SPP.h"
#include "PPPStatic.h"
#include "PPPKinematic.h"
#include "ObsFormat.h"
#include "Log.h"
#include "core/SolveConfig.h"
#include "CliCommon.h"
#include "SolveDriver.h"
#include "core/IniFile.h"

namespace {
    void runPpp(const SolveConfig &cfg) {
        std::vector<GnssTask::EpochData> epochs;

        auto init = [&](auto &ppp) -> bool {
            if (!cfg.sp3.empty()) ppp.loadSp3(cfg.sp3);
            if (!cfg.clk.empty()) ppp.loadClk(cfg.clk, cfg.systems);
            if (!cfg.navPath.empty()) ppp.loadBrdc(cfg.navPath);
            if (!cfg.atx.empty()) ppp.loadAtx(cfg.atx);
            if (!cfg.osb.empty()) ppp.loadOsb(cfg.osb);
            if (!cfg.erp.empty()) ppp.loadErp(cfg.erp);
            if (!cfg.tro.empty()) ppp.loadTro(cfg.tro);
            if (!cfg.ionex.empty()) ppp.loadIonex(cfg.ionex);

            ppp.mApplySolidTide = cfg.solidTide != 0;
            ppp.setMeasNoise(cfg.sigIFCode, cfg.sigPhase);
            ppp.setFirstEpochPosVar(cfg.firstEpochPosVar);
            ppp.setTropo(true, cfg.ztdInitValue, cfg.ztdInitVar, cfg.ztdQ);
            ppp.setAmbInitVar(cfg.ambInitVar);
            ppp.setZtdProcNoise(cfg.ztdQ);
            ppp.setZtdPriorSigma(cfg.ztdPrior);
            if (cfg.kinematic) ppp.setPosProcNoise(cfg.posNoise);
            else ppp.setPosProcNoise(cfg.staticPosNoise);
            ppp.setPosVarFloor(cfg.posVarFloor);
            ppp.setEnabledSystems(cfg.systems);
            ppp.setCutoffElevDeg(cfg.cutoffDeg);
            ppp.setAmbProcNoise(cfg.ambQ);
            ppp.setTroZtdSigma(cfg.troZtdSigma);
            ppp.mUseAR = cfg.useAR != 0;
            ppp.mUseCycleSlip = cfg.useCycleSlip != 0;
            ppp.mUseTroGrad = cfg.useTroGrad != 0;
            ppp.mTropoMap = cfg.tropoMap;
            ppp.mApplyWindup = cfg.applyWindup != 0;
            ppp.setCodeBiasProcNoise(cfg.codeBiasProcNoise, cfg.codeBiasInitVar);
            ppp.setGloIfcbProcNoise(cfg.gloIfcbProcNoise, cfg.gloIfcbInitVar);
            ppp.setIsbProcNoise(cfg.isbProcNoise, cfg.isbInitVar);
            ppp.setKeepAmbMaxGap(cfg.keepAmbMaxGap);
            return true;
        };

        Eigen::Vector3d ref(0, 0, 0);

        if (cfg.obsPath.empty()) {
            std::cerr << "PPP 需要观测文件 [input] obs\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Ppp, cfg);
            return;
        }
        SolveDriver::RinexFileSource rsrc;
        SolveDriver::Oem7FileSource osrc;
        SolveDriver::ObsSource *src = nullptr;
        const auto fmt = GnssTask::classifyObsPath(cfg.obsPath);
        if (fmt == GnssTask::ObsFormat::Oem7) {
            if (!osrc.open(cfg.obsPath)) {
                std::cerr << "PPP 无法打开 OEM7: " << cfg.obsPath << "\n";
                Cli::dumpEmpty(GnssTask::SolverMode::Ppp, cfg);
                return;
            }
            src = &osrc;
        } else if (fmt == GnssTask::ObsFormat::Rinex) {
            if (!rsrc.open(cfg.obsPath)) {
                std::cerr << "PPP 无法打开 RINEX: " << cfg.obsPath << "\n";
                Cli::dumpEmpty(GnssTask::SolverMode::Ppp, cfg);
                return;
            }
            src = &rsrc;
        } else {
            std::cerr << "PPP 无法识别观测文件格式: "
                      << cfg.obsPath << "\n";
            Cli::dumpEmpty(GnssTask::SolverMode::Ppp, cfg);
            return;
        }

        if (cfg.kinematic) {
            PPPKinematic k;
            init(k);
            epochs = SolveDriver::solveAll(*src, k);
        } else {
            PPPStatic s;
            init(s);
            epochs = SolveDriver::solveAll(*src, s);
        }

        if (cfg.hasRef) ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
        else SolveDriver::firstSolvedRef(epochs, ref);
        Cli::dumpCsv(epochs, GnssTask::SolverMode::Ppp, ref, false, cfg);
    }
}

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "ppp.ini";
    AppConfig::IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolveConfig cfg = loadConfig(f);
    Log::init("cli_gnsslab.log");
    std::cout << "[PPP] obs=" << cfg.obsPath
            << " 运动学=" << (cfg.kinematic ? "是" : "否") << "\n";
    runPpp(cfg);
    return 0;
}
