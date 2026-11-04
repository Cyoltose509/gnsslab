#include "Task.h"

#include <ostream>
#include <string>

#include "Log.h"
#include "Observer.h"
#include "SPP.h"
#include "StringUtils.h"
#include "io/ObsSource.h"
#include "io/ProductLoad.h"
#include "solvers/common/CSV.h"
#include "solvers/spp/Realtime.h"

namespace Solvers::SPP {
    void Task::applyConfig(const SolverOptions &cfg) {
        core->mode = Mode::SPP;
        core->opts = cfg;
        core->filePath = cfg.obsPath;
        core->fileName = Str::baseName(cfg.obsPath);
        core->brdcPathBuf = cfg.navPath;
        core->cutoffDeg = cfg.cutoffDeg;
        core->enabledSystems = cfg.systems;
        core->sampleInterval = cfg.sampleInterval;
        core->isRINEX = Observer::isRINEX(Observer::classifyObsPath(cfg.obsPath));
        core->view.obsPathBuf = cfg.obsPath;
        core->rtEndpoints.resize(1);
        core->rtEndpoints[0].ip = cfg.ip;
        core->rtEndpoints[0].port = std::to_string(cfg.port);
    }

    bool Task::initSolver(void *solver) {
        ::SPP &spp = *static_cast<::SPP *>(solver);
        std::vector<std::string> navs;
        if (!core->brdcPathBuf.empty()) navs.push_back(core->brdcPathBuf);
        else navs = ObsSource::scanNavFiles(core->filePath);
        for (const auto &nav: navs) {
            try {
                spp.loadBRDC(nav);
                core->hasNav = true;
                LOG_INFO << "SPP 读取广播星历: " << nav;
            } catch (const std::exception &e) {
                LOG_ERROR << "SPP 读取广播星历失败 " << nav << ": " << e.what();
            }
        }
        const auto products = ProductLoad::resolve(core->opts, modeKey(Mode::SPP));
        ProductLoad::loadSPPGrids(spp, products.vmf, products.gpt3);
        ProductLoad::warnTropoFallback("SPP", core->opts.spp.tropoMap, spp.tropoGridReady());
        return true;
    }

    void Task::beforeSolveEpoch(void *solver, Observer::Item *item) {
        ::SPP &spp = *static_cast<::SPP *>(solver);
        const auto &[obs, eph] = *item;
        spp.ephTable.mergeEmbedded(eph, obs.epoch);
        if (!core->isRINEX && eph) core->hasNav = true;
        if (core->isRINEX && !core->hasNav) core->noEphSolve = true;
    }

    ::SPP *Task::makeSolver() {
        return new ::SPP;
    }

    std::unique_ptr<Realtime::Session> Task::makeSession() {
        return std::make_unique<Realtime::SPPSession>(
            std::static_pointer_cast<Task>(shared_from_this()), core->rtEndpoints[0]);
    }

    void Task::exportCSV(std::ostream &out) const {
        CSV::Writer w;
        w.row("Wk", "SOW", "ECEF-X/m", "ECEF-Y/m", "ECEF-Z/m",
              "REF-X/m", "REF-Y/m", "REF-Z/m", "EAST/m", "NORTH/m", "UP/m",
              "B/deg", "L/deg", "H/m", "VX/m", "VY/m", "VZ/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "SigmaP", "SigmaV",
              "SatCount", "3dRMS/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: core->epochs) {
            if (!r.solved) {
                csvInvalidRow(w, r, cols);
                continue;
            }
            const auto &res = r.sppResult;
            const ENU enu = Coord::XYZToENU(res.xyz, core->refECEF);
            const double rms3d = std::sqrt(enu.E() * enu.E() + enu.N() * enu.N() + enu.U() * enu.U());
            w.row(r.week, CSV::fx(r.sow, 3),
                  CSV::f(res.xyz[0], 11), CSV::f(res.xyz[1], 11), CSV::f(res.xyz[2], 11),
                  CSV::f(core->refECEF.x(), 11), CSV::f(core->refECEF.y(), 11), CSV::f(core->refECEF.z(), 11),
                  CSV::f(enu.E(), 6), CSV::f(enu.N(), 6), CSV::f(enu.U(), 6),
                  CSV::f(res.blh[0] * RAD_TO_DEG, 10), CSV::f(res.blh[1] * RAD_TO_DEG, 10),
                  CSV::f(res.blh[2], 4),
                  CSV::f(res.vel[0], 6), CSV::f(res.vel[1], 6), CSV::f(res.vel[2], 6),
                  CSV::f(res.pdop, 4), CSV::f(res.gdop, 4), CSV::f(res.hdop, 4),
                  CSV::f(res.vdop, 4), CSV::f(res.tdop, 4),
                  CSV::f(res.sigmaP, 4), CSV::f(res.sigmaV, 4),
                  r.numSatsResult,
                  CSV::f(rms3d, 6));
        }
        out << w.str();
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
