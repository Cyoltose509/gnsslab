#include "Task.h"

#include <chrono>
#include <memory>
#include <ostream>
#include <string>

#include "Decimator.h"
#include "Log.h"
#include "Observer.h"
#include "SPP.h"
#include "StringUtils.h"
#include "io/ObsSource.h"
#include "io/ProductLoad.h"
#include "solvers/common/CSV.h"
#include "solvers/common/Core.h"
#include "solvers/spp/Realtime.h"

namespace Solvers::SPP {
    void Task::applyConfig(const SolverOptions &cfg) {
        opts = cfg;
        filePath = cfg.obsPath;
        fileName = Str::baseName(cfg.obsPath);
        isRINEX = Observer::isRINEX(Observer::classifyObsPath(cfg.obsPath));
        navPathBuf = cfg.navPath;
        rtEndpoints.resize(1);
        rtEndpoints[0].ip = cfg.ip;
        rtEndpoints[0].port = std::to_string(cfg.port);
    }

    bool Task::initSolver(::SPP &solver) {
        std::vector<std::string> navs;
        if (!navPathBuf.empty()) navs.push_back(navPathBuf);
        else navs = ObsSource::scanNavFiles(filePath);
        for (const auto &nav: navs) {
            try {
                solver.loadBRDC(nav);
                hasNav = true;
                LOG_INFO << "SPP 读取广播星历: " << nav;
            } catch (const std::exception &e) {
                LOG_ERROR << "SPP 读取广播星历失败 " << nav << ": " << e.what();
            }
        }
        const auto products = ProductLoad::resolve(opts, modeKey(Mode::SPP));
        ProductLoad::loadSPPGrids(solver, products.vmf, products.gpt3);
        ProductLoad::warnTropoFallback("SPP", opts.spp.tropoMap, solver.tropoGridReady());
        return true;
    }

    void Task::beforeSolveEpoch(::SPP &solver, Observer::Item &item) {
        solver.ephTable.mergeEmbedded(item.eph, item.obs.epoch);
        if (const auto &eph = item.eph; !isRINEX && eph) hasNav = true;
        if (isRINEX && !hasNav) noEphSolve = true;
    }

    bool Task::afterSolveEpoch(::SPP &, Epoch &data, Observer::Item &, int) {
        if (!data.solved) return false;
        std::lock_guard lk(mutex);
        if (!initializedRefECEF) {
            // 勾选参考真值且已填有效坐标时，以真值为 ENU 参考点（{0,0,0} 视为未填，退回首历元解）
            Eigen::Vector3d truth;
            {
                std::lock_guard<std::mutex> flk(refFetchMtx);
                truth = refXyz;
            }
            refECEF = (useRefTruth && truth.squaredNorm() > 1.0) ? truth : data.sppResult.xyz;
            initializedRefECEF = true;
        }
        return true;
    }

    std::unique_ptr<::SPP> Task::makeSolver() {
        return std::make_unique<::SPP>();
    }

    std::unique_ptr<Realtime::Session> Task::makeSession() {
        return std::make_unique<Realtime::SPPSession>(
            std::static_pointer_cast<Task>(shared_from_this()), rtEndpoints[0]);
    }

    void Task::exportCSV(std::ostream &out) const {
        CSV::Writer w;
        w.row("Wk", "SOW", "ECEF-X/m", "ECEF-Y/m", "ECEF-Z/m",
              "REF-X/m", "REF-Y/m", "REF-Z/m", "EAST/m", "NORTH/m", "UP/m",
              "B/deg", "L/deg", "H/m", "VX/m", "VY/m", "VZ/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "SigmaP", "SigmaV",
              "SatCount", "3dRMS/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: epochs) {
            if (!r.solved) {
                csvInvalidRow(w, r, cols);
                continue;
            }
            const auto &res = r.sppResult;
            const ENU enu = Coord::XYZToENU(res.xyz, refECEF);
            const double rms3d = std::sqrt(enu.E() * enu.E() + enu.N() * enu.N() + enu.U() * enu.U());
            w.row(r.week, CSV::fx(r.sow, 3),
                  CSV::f(res.xyz[0], 11), CSV::f(res.xyz[1], 11), CSV::f(res.xyz[2], 11),
                  CSV::f(refECEF.x(), 11), CSV::f(refECEF.y(), 11), CSV::f(refECEF.z(), 11),
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

    namespace {
        struct TaskQueueSource : Observer::Source {
            Task &owner;

            explicit TaskQueueSource(Task &o) : owner(o) {
            }

            bool next(Observer::Item &out) override {
                if (owner.epochLimitReached()) return false;
                return owner.nextObs(out);
            }
        };
    }

    void Task::solveBody() {
        const std::unique_ptr<::SPP> solver = makeSolver();
        if (!solver) {
            setError("解算器未就绪");
            finishSolving();
            return;
        }
        const auto tSolve0 = std::chrono::steady_clock::now();
        try {
            if (!waitNavReady()) {
                LOG_WARN << "[解算] 任务被取消(stop 已置位)：解算线程未启动，0 历元";
                finishSolving();
                return;
            }

            std::string sys;
            for (const char c: opts.systems) sys += c;
            LOG_INFO << "[解算] 开始: 截止高度角 " << opts.cutoffDeg << "°, 系统="
                    << (sys.empty() ? std::string("全部") : sys)
                    << (opts.sampleInterval > 0.0
                            ? ", 采样间隔 " + std::to_string(opts.sampleInterval) + "s"
                            : ", 采样间隔按文件");
            const SolverOptions o = opts;
            solver->applyOptions(o);
            if (!initSolver(*solver)) {
                LOG_ERROR << "[解算] 解算器初始化失败（产品加载返回 false）";
                finishSolving();
                return;
            }

            TaskQueueSource src(*this);
            const Observer::Decimator dec(opts.sampleInterval);
            Observer::Item item;
            while (src.next(item)) {
                Epoch data;
                try {
                    if (!dec.accept(item.obs.weekSecond.sow)) continue;
                    data.getFromObs(item.obs);
                    beforeSolveEpoch(*solver, item);
                    if (solver->processEpoch(item.obs)) data.getFromSPP(*solver);
                    else data.solved = false;
                    int idx = 0;
                    {
                        std::lock_guard lk(mutex);
                        idx = static_cast<int>(epochs.size());
                    }
                    if (!afterSolveEpoch(*solver, data, item, idx)) data.solved = false;
                } catch (const std::exception &e) {
                    LOG_ERROR << "[解算] 历元异常 (sod=" << item.obs.epoch.m_sod << "): " << e.what();
                    data.solved = false;
                } catch (...) {
                    LOG_ERROR << "[解算] 历元未知异常 (sod=" << item.obs.epoch.m_sod << ")";
                    data.solved = false;
                }
                commitEpoch(std::move(data));
            }
        } catch (const std::exception &e) {
            LOG_ERROR << "[解算] 解算线程异常: " << e.what();
            setError(e.what());
        } catch (...) {
            LOG_ERROR << "[解算] 解算线程未知异常";
            setError("解算线程未知异常");
        }
        std::size_t total = 0;
        {
            std::lock_guard lk(mutex);
            total = epochs.size();
        }
        LOG_INFO << "[解算] 结束: " << total << " 历元, 耗时 "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - tSolve0).count() << " ms";
        finishSolving();
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
