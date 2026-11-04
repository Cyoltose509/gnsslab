#include "Task.h"

#include <chrono>
#include <ostream>
#include <string>

#include "EpochDecimator.h"
#include "Log.h"
#include "Observer.h"
#include "StringUtils.h"
#include "solvers/common/CSV.h"
#include "solvers/rtk/Pipeline.h"
#include "solvers/rtk/Realtime.h"

namespace Solvers::RTK {
    namespace {
        void failRead(const std::shared_ptr<Task> &task, const std::string &msg) {
            task->core->setError(msg);
            task->readDone = true;
            task->core->readDone = true;
            task->core->loading = false;
            task->core->queueCv.notify_all();
        }
    }

    void Task::applyConfig(const SolverOptions &cfg) {
        core->mode = Mode::RTK;
        core->opts = cfg;
        core->filePath = cfg.roverPath;
        core->fileName = Str::baseName(cfg.roverPath) + " / " + Str::baseName(cfg.basePath);
        core->isRINEX = Observer::isRINEX(Observer::classifyObsPath(cfg.roverPath));
        baseObsPathBuf = cfg.basePath;
        roverObsPathBuf = cfg.roverPath;
        navPathBuf = cfg.navPath;
        cutoffDeg = cfg.cutoffDeg;
        enabledSystems = cfg.systems;
        sampleInterval = cfg.sampleInterval;
        core->cutoffDeg = cfg.cutoffDeg;
        core->enabledSystems = cfg.systems;
        core->sampleInterval = cfg.sampleInterval;
        // ini 里显式填的基准坐标优先于观测文件里的概略坐标（RINEX 表头 APPROX POSITION
        // 常是米级单点解，直接当基准会让双差残差劣化到无法固定）。
        if (Eigen::Vector3d(cfg.baseCoordX, cfg.baseCoordY, cfg.baseCoordZ).norm() > 1e3) {
            useBaseCoord = true;
            baseXyz = Eigen::Vector3d(cfg.baseCoordX, cfg.baseCoordY, cfg.baseCoordZ);
        }
        core->rtEndpoints.resize(2);
        core->rtEndpoints[0].ip = cfg.baseIp;
        core->rtEndpoints[0].port = std::to_string(cfg.basePort);
        core->rtEndpoints[1].ip = cfg.roverIp;
        core->rtEndpoints[1].port = std::to_string(cfg.roverPort);
    }

    void Task::commitRtkEpoch(const ObsData &rover) {
        Epoch data;
        data.getFromObs(rover);
        data.getFromRTK(rtk);
        const Eigen::Vector3d baseXyz = rtk.mBaseXyz;
        commitEpoch(core, std::move(data), [&](int, Epoch &) {
            if (!core->initializedRefECEF) {
                core->refECEF = XYZ(baseXyz.x(), baseXyz.y(), baseXyz.z());
                core->initializedRefECEF = true;
            }
        });
    }

    void Task::readBody() {
        auto &task = *this;
        try {
            {
                std::lock_guard lk(task.core->view.qcInputMutex);
                task.core->view.qcInput.clear();
            }
            task.core->view.extraQcTabs.clear();
            task.core->view.qcTabLabel = "质量分析[流动站]";
            std::vector<QC::QCObsEpoch> baseQcInput;
            auto baseQc = std::make_shared<Solvers::Core>();
            baseQc->mode = Mode::RTK;
            baseQc->filePath = task.baseObsPathBuf;
            baseQc->fileName = Str::baseName(task.baseObsPathBuf);
            baseQc->sampleInterval = task.sampleInterval;

            const EpochDecimator dec(task.sampleInterval);
            std::string pipeErr;
            const int nPair = pairFiles(
                task.baseObsPathBuf, task.roverObsPathBuf, task.navPathBuf,
                task.sampleInterval, task.navTable,
                [&] { return task.shouldStop(); },
                [&](const ObsData &base, const ObsData &rover,
                    const std::shared_ptr<EphemerisTable> &eph, EphemerisTable *) {
                    {
                        std::lock_guard lk(task.core->view.qcInputMutex);
                        task.core->view.qcInput.push_back(QC::makeQCObsEpoch(rover, rover.weekSecond.sow));
                    }
                    baseQcInput.push_back(QC::makeQCObsEpoch(base, base.weekSecond.sow));
                    if (!dec.accept(rover.weekSecond.sow)) return;
                    {
                        std::lock_guard lk(task.pairMtx);
                        task.pairQueue.push({base, rover, eph});
                    }
                    task.pairCv.notify_one();
                }, pipeErr);
            if (nPair < 0) {
                failRead(std::static_pointer_cast<Task>(shared_from_this()), pipeErr);
                return;
            }
            task.totalEpochs = nPair;

            task.readDone = true;
            task.core->readDone = true;
            task.core->queueCv.notify_all();
            task.pairCv.notify_all();
            {
                std::lock_guard lk(task.core->view.qcInputMutex);
                if (task.core->view.qcInput.empty()) task.core->view.qcReady = true;
                else task.core->view.qcDirty = true;
            }
            if (!baseQcInput.empty()) {
                baseQc->view.qcInput = std::move(baseQcInput);
                baseQc->view.qcDirty = true;
            }
            baseQc->readDone = true;
            baseQc->done = true;
            {
                std::lock_guard lk(task.core->mutex);
                task.core->view.extraQcTabs.emplace_back("质量分析[基准站]", baseQc);
            }
        } catch (const std::exception &e) {
            task.core->setError(std::string("RTK 读取错误: ") + e.what());
            task.readDone = true;
            task.core->readDone = true;
            task.core->queueCv.notify_all();
            task.pairCv.notify_all();
        }
        task.core->loading = false;
    }

    void Task::solveBody() {
        const auto tRtk0 = std::chrono::steady_clock::now();

        SolverOptions opts = core->opts;
        opts.cutoffDeg = cutoffDeg;
        opts.systems = enabledSystems;
        opts.sampleInterval = sampleInterval;
        rtk.applyOptions(opts);
        rtk.mBaseXyzUser = useBaseCoord ? baseXyz : Eigen::Vector3d::Zero();

        try {
            while (true) {
                RTKPair item;
                {
                    std::unique_lock lk(pairMtx);
                    pairCv.wait(lk, [&] {
                        return !pairQueue.empty() || readDone.load() || stop.load();
                    });
                    if (pairQueue.empty() && (readDone.load() || stop.load())) break;
                    if (pairQueue.empty()) continue;
                    item = std::move(pairQueue.front());
                    pairQueue.pop();
                }
                if (stop) break;

                EphemerisTable *eph = item.eph ? item.eph.get() : &navTable;
                bool ok = false;
                try { ok = rtk.processEpoch(item.base, item.rover, *eph); } catch (const std::exception &e) {
                    LOG_ERROR << "[RTK] processEpoch 异常: " << e.what();
                    ok = false;
                } catch (...) {
                    LOG_ERROR << "[RTK] processEpoch 未知异常";
                    ok = false;
                }

                if (!ok) continue;
                if (const Result &r = rtk.result; r.numSats <= 0) continue;
                commitRtkEpoch(item.rover);
            }
        } catch (const std::exception &e) {
            core->setError(std::string("RTK 解算错误: ") + e.what());
        }
        solvingDone = true;
        core->solvingDone = true;

        std::size_t rtkEpochs = 0;
        {
            std::lock_guard lk(core->mutex);
            rtkEpochs = core->epochs.size();
        }
        LOG_INFO << "[RTK] 结束: " << rtkEpochs << " 历元, 耗时 "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - tRtk0).count() << " ms";
    }

    std::unique_ptr<Realtime::Session> Task::makeSession() {
        return std::make_unique<Realtime::RTKSession>(
            std::static_pointer_cast<Task>(shared_from_this()),
            core->rtEndpoints[0], core->rtEndpoints[1]);
    }

    void Task::exportCSV(std::ostream &out) const {
        const Eigen::Vector3d &ref = core->refECEF;
        CSV::Writer w;
        w.row("Wk", "SOW", "Status",
              "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m",
              "Float-3dRMS/m", "Float-SigmaP/m",
              "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m",
              "Fix-3dRMS/m", "Fix-SigmaP/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "NSats",
              "REF-X/m", "REF-Y/m", "REF-Z/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: core->epochs) {
            if (!r.solved) {
                csvInvalidRow(w, r, cols);
                continue;
            }
            const auto &res = r.sppResult;
            const bool fixed = res.stat == 1;
            const Eigen::Vector3d base(ref.x(), ref.y(), ref.z());
            const Eigen::Vector3d floatE = r.rtkFloatXyz.squaredNorm() > 1.0
                                               ? r.rtkFloatXyz
                                               : base + r.rtkBaseFloat;
            const Eigen::Vector3d fixE = fixed ? res.xyz : Eigen::Vector3d::Zero();
            w.cell(r.week).cell(CSV::fx(r.sow, 3)).cell(csvStatStr(res.stat));
            csvWriteSolBlock(w, floatE, ref, res.sigmaPFloat, false);
            csvWriteSolBlock(w, fixE, ref, res.sigmaPFixed, !fixed);
            w.cell(CSV::f(res.pdop, 4)).cell(CSV::f(res.gdop, 4)).cell(CSV::f(res.hdop, 4))
                    .cell(CSV::f(res.vdop, 4)).cell(CSV::f(res.tdop, 4))
                    .cell(r.rtkRatio).cell(r.numSatsResult);
            w.cell(CSV::f(ref.x(), 11)).cell(CSV::f(ref.y(), 11)).cell(CSV::f(ref.z(), 11));
            w.endRow();
        }
        out << w.str();
    }

    std::shared_ptr<Task> makeTask(const SolverOptions &cfg) {
        auto task = std::make_shared<Task>();
        task->applyConfig(cfg);
        return task;
    }
}
