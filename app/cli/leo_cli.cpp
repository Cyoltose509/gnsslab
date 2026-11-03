#include <iostream>
#include <string>
#include <vector>

#include "Exception.h"
#include "GnssStruct.h"
#include "CoordConvert.h"
#include "solvers/common/Core.h"
#include "SPP.h"
#include "LEO.h"
#include "Observer.h"

#include "product/SP3OrbitReader.h"
#include "Log.h"
#include "SolverOptions.h"
#include "CliCommon.h"
#include "solvers/common/Driver.h"
#include "io/ProductLoad.h"
#include "core/IniFile.h"

namespace {
    void runLeo(const SolverOptions &cfg) {
        std::vector<Solvers::Epoch> epochs;

        Observer::RINEXFile rsrc;
        Observer::NovAtelFile osrc;
        Observer::Source *src = Cli::openObsSource(cfg.obsPath, "LEO", rsrc, osrc);
        if (!src) {
            Cli::dumpEmpty(Solvers::Mode::LEO, cfg);
            return;
        }


        SP3OrbitReader refRdr;
        bool hasRefOrbit = false;
        SatID refSat('L', cfg.refPrn);
        int refPrn = cfg.refPrn;
        if (!cfg.refSp3.empty()) {
            refRdr.read(cfg.refSp3);
            bool refFound = false;
            for (const auto &[sat, _]: refRdr.getData())
                if (sat.system == 'L') {
                    refPrn = sat.id;
                    refFound = true;
                    break;
                }
            if (!refFound && !refRdr.getData().empty()) refPrn = refRdr.getData().begin()->first.id;
            refSat = SatID('L', refPrn);
            hasRefOrbit = true;
        }

        LEO leo;
        leo.applyOptions(cfg);

        if (!ProductLoad::requireSatellite("LEO", cfg.gnssSp3, cfg.gnssBrdc)) {
            Cli::dumpEmpty(Solvers::Mode::LEO, cfg);
            return;
        }
        std::vector<std::future<bool> > jobs;
        const ProductLoad::Paths paths{cfg.gnssSp3, cfg.gnssClk, cfg.gnssBrdc, cfg.osb, cfg.att, cfg.erp, cfg.atx};
        ProductLoad::core(leo, jobs, "LEO", paths, cfg.systems);
        bool loadOk = true;
        for (auto &j: jobs)
            if (!j.get()) loadOk = false;
        if (!loadOk) {
            std::cerr << "LEO 产品加载失败\n";
            Cli::dumpEmpty(Solvers::Mode::LEO, cfg);
            return;
        }
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

        Solvers::Driver::solveStationStream(*src, leo,
                                        [&](LEO &, const Observer::Item &item, Solvers::Epoch &data) {
                                            const Eigen::Vector3d leoPos = data.sppResult.xyz;
                                            if (!data.solved || leoPos.squaredNorm() <= 1e12) {
                                                data.solved = false;
                                                epochs.push_back(std::move(data));
                                                return true;
                                            }
                                            if (hasRefOrbit) {
                                                const PVT refPVT = refRdr.getPVT(refSat, item.obs.epoch);
                                                const Eigen::Vector3d refPos = refPVT.p.squaredNorm() > 1e12 ? refPVT.p : leoPos;
                                                const Eigen::Vector3d rtn = Coord::XYZToRTN(leoPos, refPos, refPVT.v);
                                                data.refECEF = refPos;
                                                data.rtnR = rtn(0);
                                                data.rtnT = rtn(1);
                                                data.rtnN = rtn(2);
                                                if (!leoRefInit) {
                                                    leoRef = refPos;
                                                    leoRefInit = true;
                                                }
                                            }
                                            if (!refInit) {
                                                ref = leoPos;
                                                refInit = true;
                                            }
                                            epochs.push_back(std::move(data));
                                            return true;
                                        }, cfg.sampleInterval);

        if (cfg.hasRef) {
            ref = Eigen::Vector3d(cfg.refX, cfg.refY, cfg.refZ);
            leoRef = ref;
        }
        Cli::dumpCSV(epochs, Solvers::Mode::LEO, hasRefOrbit ? leoRef : ref, hasRefOrbit, cfg);
    }
}

int main(const int argc, char **argv) {
    const std::string ini = argc > 1 ? argv[1] : "leo.ini";
    IniFile f;
    if (!f.load(ini)) {
        std::cerr << "无法读取 ini: " << ini << "\n";
        return 1;
    }
    const SolverOptions cfg = loadConfig(f, Solvers::Mode::LEO);
    Log::init("cli_gnsslab.log");
    std::cout << "[LEO] obs=" << cfg.obsPath
            << " 参考轨道=" << (cfg.refSp3.empty() ? "(无)" : cfg.refSp3) << "\n";
    runLeo(cfg);
    return 0;
}
