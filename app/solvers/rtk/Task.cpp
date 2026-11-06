#include "Task.h"

#include <chrono>
#include <memory>
#include <ostream>
#include <string>

#include "Decimator.h"
#include "Log.h"
#include "Observer.h"
#include "StringUtils.h"
#include "solvers/common/CSV.h"
#include "solvers/rtk/Pipeline.h"
#include "solvers/rtk/Realtime.h"

namespace Solvers::RTK {
    void Task::applyConfig(const SolverOptions &cfg) {
        mode = Mode::RTK;
        opts = cfg;
        filePath = cfg.roverPath;
        fileName = Str::baseName(cfg.roverPath) + " / " + Str::baseName(cfg.basePath);
        isRINEX = Observer::isRINEX(Observer::classifyObsPath(cfg.roverPath));
        baseObsPathBuf = cfg.basePath;
        roverObsPathBuf = cfg.roverPath;
        navPathBuf = cfg.navPath;
        // ini 中的基准坐标优先于 RINEX 表头概略坐标。
        const Eigen::Vector3d cfgBase(cfg.baseCoordX, cfg.baseCoordY, cfg.baseCoordZ);
        useBaseCoord = cfgBase.norm() > 1e3;
        baseXyz = useBaseCoord ? cfgBase : Eigen::Vector3d::Zero();
        rtEndpoints.resize(2);
        rtEndpoints[0].ip = cfg.baseIp;
        rtEndpoints[0].port = std::to_string(cfg.basePort);
        rtEndpoints[1].ip = cfg.roverIp;
        rtEndpoints[1].port = std::to_string(cfg.roverPort);
    }

    bool Task::solvePair(const ObsData &base, const ObsData &rover, EphemerisTable &eph) {
        try {
            if (!rtk.processEpoch(base, rover, eph) || rtk.result.numSats <= 0) return false;
            Epoch data;
            data.getFromObs(rover);
            data.getFromRTK(rtk);
            const Eigen::Vector3d b = rtk.mBaseXyz;
            commitEpoch(std::move(data), [&](int, Epoch &) {
                if (!initializedRefECEF) {
                    refECEF = XYZ(b.x(), b.y(), b.z());
                    initializedRefECEF = true;
                }
            });
            return true;
        } catch (const std::exception &e) {
            LOG_ERROR << "[RTK] 历元异常: " << e.what();
            return false;
        } catch (...) {
            LOG_ERROR << "[RTK] 历元未知异常";
            return false;
        }
    }

    void Task::onReset() {
        {
            std::lock_guard lk(pairMtx);
            std::queue<RTKPair> empty;
            std::swap(pairQueue, empty);
        }
        rtk = ::RTK{};
        navTable = EphemerisTable{};
    }

    void Task::wakeQueues() {
        Solvers::Task::wakeQueues();
        /// 与配对队列的等待侧共用一把锁后再通知。
        std::lock_guard lk(pairMtx);
        pairCv.notify_all();
        pairSpaceCv.notify_all();
    }

    void Task::readBody() {
        try {
            view.qcTabLabel = "质量分析[流动站]";
            auto baseTab = std::make_shared<QcTab>();
            baseTab->label = "质量分析[基准站]";
            baseTab->fileName = Str::baseName(baseObsPathBuf);
            baseTab->view.clearQCInput(opts.sampleInterval);

            const Observer::Decimator dec(opts.sampleInterval);
            std::string pipeErr;
            const int nPair = pairFiles(
                baseObsPathBuf, roverObsPathBuf, navPathBuf,
                opts.sampleInterval, navTable,
                [&] { return shouldStop(); },
                [&](const ObsData &base, const ObsData &rover,
                    const std::shared_ptr<EphemerisTable> &eph, EphemerisTable *) {
                    view.pushQCInput(rover, rover.weekSecond.sow);
                    baseTab->view.pushQCInput(base, base.weekSecond.sow);
                    view.markQcDirty();
                    baseTab->view.markQcDirty();
                    if (!dec.accept(rover.weekSecond.sow)) return;
                    {
                        std::unique_lock lk(pairMtx);
                        pairSpaceCv.wait(lk, [&] {
                            return pairQueue.size() < queueDepth || stop.load() || solvingDone.load();
                        });
                        if (stop.load() || solvingDone.load()) return;
                        pairQueue.push({base, rover, eph});
                    }
                    pairCv.notify_one();
                }, pipeErr);
            if (nPair < 0) {
                failRead(pipeErr);
                loading = false;
                return;
            }
            {
                std::lock_guard lk(pairMtx);
                readDone = true;
            }
            wakeQueues();
            view.endQCInput();
            baseTab->view.endQCInput();
            {
                std::lock_guard lk(mutex);
                view.extraQcTabs.push_back(std::move(baseTab));
            }
        } catch (const std::exception &e) {
            failRead("RTK 读取错误: " + std::string(e.what()));
        } catch (...) {
            failRead("RTK 读取未知异常");
        }
        loading = false;
    }

    void Task::solveBody() {
        const auto tRtk0 = std::chrono::steady_clock::now();

        const SolverOptions o = opts;
        rtk.applyOptions(o);
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
                pairSpaceCv.notify_one();
                if (stop || epochLimitReached()) break;

                EphemerisTable *eph = item.eph ? item.eph.get() : &navTable;
                solvePair(item.base, item.rover, *eph);
            }
        } catch (const std::exception &e) {
            setError(std::string("RTK 解算错误: ") + e.what());
        } catch (...) {
            setError("RTK 解算未知异常");
        }
        finishSolving();

        std::size_t nSolved = 0;
        {
            std::lock_guard lk(mutex);
            nSolved = epochs.size();
        }
        LOG_INFO << "[RTK] 结束: " << nSolved << " 历元, 耗时 "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - tRtk0).count() << " ms";
    }

    std::unique_ptr<Realtime::Session> Task::makeSession() {
        return std::make_unique<Realtime::RTKSession>(
            std::static_pointer_cast<Task>(shared_from_this()),
            rtEndpoints[0], rtEndpoints[1]);
    }

    void Task::exportCSV(std::ostream &out) const {
        const Eigen::Vector3d &ref = refECEF;
        CSV::Writer w;
        w.row("Wk", "SOW", "Status",
              "Float-X/m", "Float-Y/m", "Float-Z/m", "Float-E/m", "Float-N/m", "Float-U/m",
              "Float-3dRMS/m", "Float-SigmaP/m",
              "Fix-X/m", "Fix-Y/m", "Fix-Z/m", "Fix-E/m", "Fix-N/m", "Fix-U/m",
              "Fix-3dRMS/m", "Fix-SigmaP/m",
              "PDOP", "GDOP", "HDOP", "VDOP", "TDOP", "Ratio", "NSats",
              "REF-X/m", "REF-Y/m", "REF-Z/m").endRow();
        const int cols = w.colCount();
        for (const auto &r: epochs) {
            if (!r.solved) {
                csvInvalidRow(w, r, cols);
                continue;
            }
            const auto &res = r.sppResult;
            const bool fixed = r.isFixed(mode);
            const Eigen::Vector3d base(ref.x(), ref.y(), ref.z());
            const Eigen::Vector3d floatE = r.rtkFloatXyz.squaredNorm() > 1.0
                                               ? r.rtkFloatXyz
                                               : base + r.rtkBaseFloat;
            const Eigen::Vector3d fixE = fixed ? res.xyz : Eigen::Vector3d::Zero();
            w.cell(r.week).cell(CSV::fx(r.sow, 3)).cell(csvStatStr(res.stat));
            csvWriteSolBlock(w, floatE, ref, res.sigmaPFloat, false);
            csvWriteSolBlock(w, fixE, ref, res.sigmaPFixed, !fixed);
            csvDopCells(w, res).cell(r.rtkRatio).cell(r.numSatsResult);
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
