#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "SolverTask.h"

#include "RinexObsReader.h"
#include "OEM7Reader.h"
#include "PPPStatic.h"
#include "PPPKinematic.h"
#include "LEO.h"
#include "QualityControl.h"
#include "Log.h"
#include "SolveDriver.h"
#include "GuiSolverPanel.h"   // SolverPanel::LaunchQC：读取线程收尾触发质量分析
#include "GuiSppProcessor.h"  // RunSingleStationReader/RunDualThreadJob 显式实例化
#include "GuiPppProcessor.h"
#include "GuiLeoProcessor.h"

#include <chrono>
#include <fstream>
#include <limits>
#include <string>

namespace GnssTask {
    bool SolverTaskBase::errorFlag() const { return hasError.load(); }

    std::string SolverTaskBase::errorMessage() const {
        std::lock_guard lk(errorMutex);
        return errorMsg;
    }

    void SolverTaskBase::setError(std::string msg) {
        std::lock_guard lk(errorMutex);
        errorMsg = std::move(msg);
        hasError.store(true);
    }

    void SolverTaskBase::clearError() {
        std::lock_guard lk(errorMutex);
        errorMsg.clear();
        hasError.store(false);
    }

    void SolverTaskBase::syncErrorFrom(const SolverTaskBase &src) {
        bool h = false;
        std::string m;
        {
            std::lock_guard lk(src.errorMutex);
            h = src.hasError.load();
            m = src.errorMsg;
        }
        {
            std::lock_guard lk(errorMutex);
            errorMsg = std::move(m);
            hasError.store(h);
        }
    }


    bool WaitNavReady(const std::shared_ptr<SolveTask> &core) {
        std::unique_lock lk(core->queueMutex);
        core->queueCv.wait(lk, [&] { return core->navReady.load() || core->stop.load(); });
        return !core->stop.load();
    }


    bool NextObsQueue(const std::shared_ptr<SolveTask> &core, ObsItem &out) {
        std::unique_lock lk(core->queueMutex);
        for (;;) {
            core->queueCv.wait(lk, [&] {
                return !core->obsQueue.empty() || core->readDone.load() || core->stop.load();
            });
            if (!core->obsQueue.empty()) {
                out = std::move(core->obsQueue.front());
                core->obsQueue.pop();
                return !core->stop.load();
            }
            return false;
        }
    }

    void CommitEpoch(const std::shared_ptr<SolveTask> &core, EpochData &&item,
                     const std::function<void(int, EpochData &)> &onStored) {
        std::lock_guard lk(core->mutex);
        const int idx = static_cast<int>(core->epochs.size());
        core->epochs.push_back(std::move(item));
        if (onStored) onStored(idx, core->epochs.back());
        try {
            std::lock_guard plk(core->plotMutex);
            core->plotData.insert(idx, core->epochs.back(), core->refECEF);
        } catch (...) {
        }
        if (core->selectedEpoch == -1 || core->selectedEpoch == idx - 1) core->selectedEpoch = idx;
        core->solvingProgress = idx + 1;
        core->solvedCount = static_cast<int>(core->epochs.size());
    }

    void SolverTaskBase::beforeSolveEpoch(void *solver, ObsItem *item) {
        if (!item || !item->eph) return; // RINEX 路径 eph 恒空，天然跳过
        const EphemerisTable *tbl = item->eph.get();
        if (tbl->gps.empty() && tbl->bds.empty() && lastGoodEph) tbl = lastGoodEph.get();
        else if (!tbl->gps.empty() || !tbl->bds.empty()) lastGoodEph = item->eph;
        static_cast<SPP *>(solver)->ephTable = *tbl; // shared_ptr 元素深拷贝廉价
    }

    // 默认 afterSolveEpoch：首历元参考点初始化（SPP 原先无锁、PPP 有锁，统一加锁版）。
    bool SolverTaskBase::afterSolveEpoch(void *, EpochData *data, ObsItem *, int) {
        if (!data->solved) return false;
        if (const auto core = solveResult()) {
            std::lock_guard lk(core->mutex);
            if (!core->initializedRefECEF) {
                core->refECEF = data->sppResult.xyz;
                core->initializedRefECEF = true;
            }
        }
        return true;
    }

    void SyncTaskStatus(const std::shared_ptr<SolverTaskBase> &task, const std::shared_ptr<SolveTask> &core) {
        core->loading.store(task->loading.load());
        core->done.store(task->done.load());
        core->syncErrorFrom(*task);
    }

    struct TaskQueueSource : SolveDriver::ObsSource {
        std::shared_ptr<SolveTask> core;

        explicit TaskQueueSource(std::shared_ptr<SolveTask> c) : core(std::move(c)) {
        }

        bool next(ObsItem &out) override {
            return NextObsQueue(core, out);
        }
    };

    template<typename Solver>
    void RunSingleStationSolve(const std::shared_ptr<SolveTask> &core,
                               Solver &solver,
                               const std::shared_ptr<SolverTaskBase> &job) {
        const auto tSolve0 = std::chrono::steady_clock::now();
        try {
            if (!WaitNavReady(core)) {
                LOG_WARN << "[解算] 任务被取消(stop 已置位)：解算线程未启动，0 历元";
                core->solvingDone = true;
                return;
            }

            std::string sys;
            for (const char c: core->enabledSystems) sys += c;
            LOG_INFO << "[解算] 开始: 截止高度角 " << core->cutoffDeg << "°, 系统="
                    << (sys.empty() ? std::string("全部") : sys);

            solver.setCutoffElevDeg(core->cutoffDeg);
            solver.setEnabledSystems(core->enabledSystems);
            if (!job->initSolver(&solver)) {
                LOG_ERROR << "[解算] 解算器初始化失败（产品加载返回 false）";
                core->solvingDone = true;
                return;
            }

            TaskQueueSource src(core);
            SolveDriver::solveStream(src, solver,
                                     [core, job](Solver &s, ObsItem &item, EpochData &data) {
                                         int idx = 0;
                                         {
                                             std::lock_guard lk(core->mutex);
                                             idx = static_cast<int>(core->epochs.size());
                                         }
                                         bool ok = false;
                                         try {
                                             ok = job->afterSolveEpoch(&s, &data, &item, idx);
                                         } catch (const std::exception &e) {
                                             LOG_ERROR << "[解算] 历元异常 (sod=" << item.obs.epoch.m_sod << "): " << e.what();
                                         } catch (...) {
                                             LOG_ERROR << "[解算] 历元未知异常 (sod=" << item.obs.epoch.m_sod << ")";
                                         }
                                         if (!ok) data.solved = false;
                                         CommitEpoch(core, std::move(data));
                                         return true; // 已自行处理 solved 标记，交由外层原样收下
                                     },
                                     [job](Solver &s, ObsItem &item) {
                                         job->beforeSolveEpoch(&s, &item);
                                     });
        } catch (const std::exception &e) {
            LOG_ERROR << "[解算] 解算线程异常: " << e.what();
            core->setError(e.what());
        }
        size_t totalEpochs = 0;
        {
            std::lock_guard lk(core->mutex);
            totalEpochs = core->epochs.size();
        }
        LOG_INFO << "[解算] 结束: " << totalEpochs << " 历元, 耗时 "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - tSolve0).count() << " ms";
        core->solvingDone = true;
    }

    // 三个单站解算器各实例化一份，函数体仍留在 .cpp（不进头文件）。
    template void RunSingleStationSolve<SPP>(
        const std::shared_ptr<SolveTask> &, SPP &, const std::shared_ptr<SolverTaskBase> &);

    template void RunSingleStationSolve<PPPStatic>(
        const std::shared_ptr<SolveTask> &, PPPStatic &, const std::shared_ptr<SolverTaskBase> &);

    template void RunSingleStationSolve<PPPKinematic>(
        const std::shared_ptr<SolveTask> &, PPPKinematic &, const std::shared_ptr<SolverTaskBase> &);

    template void RunSingleStationSolve<LEO>(
        const std::shared_ptr<SolveTask> &, LEO &, const std::shared_ptr<SolverTaskBase> &);

    template <class Task>
    void RunSingleStationReader(const std::shared_ptr<Task> &task, const char *tag) {
        const auto core = task->solveResult();
        try {
            LOG_INFO << tag << " 读取文件: " << core->filePath;
            if (const int ok = GnssTask::ReadObsToQueue(core, core->stop); ok >= 0) {
                bool hasQcInput = false;
                {
                    std::lock_guard lk(core->qcInputMutex);
                    hasQcInput = !core->qcInput.empty();
                }
                if (hasQcInput) Solvers::SolverPanel::LaunchQC(core);
                else core->qcReady = true; // 无 QC 输入：直接置完成位
            }
        } catch (const std::exception &e) {
            core->setError(e.what());
        }
        core->loading = false;
    }

    template <class Task>
    void RunDualThreadJob(const std::shared_ptr<Task> &task, const char *tag,
                          void (*solveFn)(const std::shared_ptr<Task> &)) {
        std::thread thRead(RunSingleStationReader<Task>, task, tag);
        std::thread thSolve(solveFn, task);
        thRead.join();
        thSolve.join();
        task->state = SolverTaskBase::State::Done;
        task->done = true;
        task->loading = false;
        GnssTask::SyncTaskStatus(task, task->solveResult());
    }

    // 三个单站解算器各实例化一份，函数体留在 .cpp（理由同 RunSingleStationSolve）。
    template void RunSingleStationReader<Solvers::SppProcessor::SppTask>(
        const std::shared_ptr<Solvers::SppProcessor::SppTask> &, const char *);
    template void RunSingleStationReader<Solvers::PppProcessor::PppTask>(
        const std::shared_ptr<Solvers::PppProcessor::PppTask> &, const char *);
    template void RunSingleStationReader<Solvers::LeoProcessor::LeoTask>(
        const std::shared_ptr<Solvers::LeoProcessor::LeoTask> &, const char *);

    template void RunDualThreadJob<Solvers::SppProcessor::SppTask>(
        const std::shared_ptr<Solvers::SppProcessor::SppTask> &, const char *,
        void (*)(const std::shared_ptr<Solvers::SppProcessor::SppTask> &));
    template void RunDualThreadJob<Solvers::PppProcessor::PppTask>(
        const std::shared_ptr<Solvers::PppProcessor::PppTask> &, const char *,
        void (*)(const std::shared_ptr<Solvers::PppProcessor::PppTask> &));
    template void RunDualThreadJob<Solvers::LeoProcessor::LeoTask>(
        const std::shared_ptr<Solvers::LeoProcessor::LeoTask> &, const char *,
        void (*)(const std::shared_ptr<Solvers::LeoProcessor::LeoTask> &));

    void PlotData::insert(const int index, const EpochData &ep, const XYZ &refECEF) {
        times.push_back(index);
        const auto &result = ep.sppResult;
        const bool solved = ep.solved;
        constexpr double dopNaN = std::numeric_limits<double>::quiet_NaN();
        sigmaPs.push_back(solved ? result.sigmaP : 0.0);
        sigmaPFloatVals.push_back(solved && result.sigmaPFloat > 0 ? result.sigmaPFloat : dopNaN);
        sigmaPFixedVals.push_back(solved && result.sigmaPFixed > 0 ? result.sigmaPFixed : dopNaN);
        sigmaVs.push_back(solved ? result.sigmaV : 0.0);
        pdops.push_back(solved ? result.pdop : dopNaN);
        gdops.push_back(solved ? result.gdop : dopNaN);
        hdops.push_back(solved ? result.hdop : dopNaN);
        vdops.push_back(solved ? result.vdop : dopNaN);
        tdops.push_back(solved ? result.tdop : dopNaN);

        // ratio 序列：RTK 用 rtkRatio；PPP/LEO 用 sppResult.ratio；未解出用 NaN 制造断点
        if (ep.rtkStat != 0) ratioVals.push_back(ep.rtkRatio);
        else if (solved) ratioVals.push_back(result.ratio);
        else ratioVals.push_back(std::numeric_limits<double>::quiet_NaN());

        // 主曲线（固定解优先，否则浮点解）
        if (solved) {
            auto enu = XYZToENU(result.xyz, refECEF);
            enu_e.push_back(enu[0]);
            enu_n.push_back(enu[1]);
            enu_u.push_back(enu[2]);
        } else {
            enu_e.push_back(0);
            enu_n.push_back(0);
            enu_u.push_back(0);
        }

        // 浮点解曲线（与固定解对比；无独立浮点解时回退为 xyz；未解算用 NaN 断点）
        if (solved) {
            Eigen::Vector3d fxyz;
            if (ep.rtkStat != 0) fxyz = ep.rtkFloatXyz;
            else if (result.xyzFloat.squaredNorm() > 1.0) fxyz = result.xyzFloat;
            else fxyz = result.xyz;
            auto fenu = XYZToENU(fxyz, refECEF);
            float_e.push_back(fenu[0]);
            float_n.push_back(fenu[1]);
            float_u.push_back(fenu[2]);
        } else {
            constexpr double nan = std::numeric_limits<double>::quiet_NaN();
            float_e.push_back(nan);
            float_n.push_back(nan);
            float_u.push_back(nan);
        }

        if (solved) {
            for (auto &[satID, res]: result.postRes) {
                satResTimes[satID].push_back(index);
                satResVals[satID].push_back(res);
            }
            for (auto &[satID, res]: result.postResPhase) {
                satResPhaseTimes[satID].push_back(index);
                satResPhaseVals[satID].push_back(res);
            }
        }
        newed = true;
    }

    void PlotData::refreshENU(const std::vector<EpochData> &ep, const XYZ &refECEF) {
        for (int i = 0; i < static_cast<int>(enu_e.size()); i++) {
            const auto &r = ep[i].sppResult;
            if (ep[i].solved) {
                auto enu = XYZToENU(r.xyz, refECEF);
                enu_e[i] = enu[0];
                enu_n[i] = enu[1];
                enu_u[i] = enu[2];
            } else {
                enu_e[i] = 0;
                enu_n[i] = 0;
                enu_u[i] = 0;
            }
            // 浮点解曲线随参考点重算（未解算保持 NaN 断点，与 insert 一致）
            if (ep[i].solved) {
                Eigen::Vector3d fxyz;
                if (ep[i].rtkStat != 0) fxyz = ep[i].rtkFloatXyz;
                else if (r.xyzFloat.squaredNorm() > 1.0) fxyz = r.xyzFloat;
                else fxyz = r.xyz;
                auto fenu = XYZToENU(fxyz, refECEF);
                float_e[i] = fenu[0];
                float_n[i] = fenu[1];
                float_u[i] = fenu[2];
            } else {
                constexpr double nan = std::numeric_limits<double>::quiet_NaN();
                float_e[i] = nan;
                float_n[i] = nan;
                float_u[i] = nan;
            }
        }
    }

    void PlotData::clear() {
        times.clear();
        sigmaPs.clear();
        sigmaPFloatVals.clear();
        sigmaPFixedVals.clear();
        sigmaVs.clear();
        pdops.clear();
        gdops.clear();
        hdops.clear();
        vdops.clear();
        tdops.clear();
        enu_e.clear();
        enu_n.clear();
        enu_u.clear();
        ratioVals.clear();
        float_e.clear();
        float_n.clear();
        float_u.clear();
        satResTimes.clear();
        satResVals.clear();
        satResPhaseTimes.clear();
        satResPhaseVals.clear();
        resRangeReady = false;
        resYlo = -8.0;
        resYhi = 8.0;
        resPhaseRangeReady = false;
        resPhaseYlo = -0.5;
        resPhaseYhi = 0.5;
    }

    std::string AutoDetectNav(const std::string &obsPath) {
        const auto &cands = ScanNavFiles(obsPath);
        return cands.empty() ? std::string() : cands.front();
    }

    int ReadRinexObsToQueue(const std::shared_ptr<SolveTask> &core, const std::atomic<bool> &stopFlag) {
        const std::string &path = core->filePath;
        {
            std::lock_guard lk(core->qcInputMutex);
            core->qcInput.clear(); // 重新收集原始观测，供质量分析（与解算解耦）
        }

        {
            std::lock_guard lk(core->queueMutex);
            core->navReady = true;
        }
        core->queueCv.notify_all();

        RinexObsReader obsReader;
        std::fstream obsFile(path.c_str(), std::ios::in);
        if (!obsFile) {
            LOG_ERROR << "无法打开 RINEX: " << path;
            core->setError("无法打开 RINEX");
            core->readDone = true;
            core->queueCv.notify_all();
            return -1;
        }
        obsReader.pFileStream = &obsFile;
        int safety = 0, ok = 0;
        while (++safety < 100000) {
            if (stopFlag) break;
            try {
                ObsData o = obsReader.parseRinexObs();
                {
                    std::lock_guard lk(core->qcInputMutex);
                    core->qcInput.push_back(QC::makeQCObsEpoch(o, o.weekSecond.sow)); // 原始观测 → QC 输入
                }
                {
                    std::lock_guard lk(core->queueMutex);
                    core->obsQueue.push({std::move(o), nullptr});
                }
                core->queueCv.notify_one();
                ok++;
            } catch (const EndOfFile &) { break; } catch (const std::exception &e) {
                LOG_ERROR << "RINEX parse err (" << path << "): " << e.what();
                core->setError("RINEX parse err: " + std::string(e.what()));
                core->readDone = true;
                core->queueCv.notify_all();
                return -1;
            }
        }
        if (ok == 0) {
            LOG_ERROR << "RINEX 无历元: " << path << "（文件是否被取消/解压失败/选错文件？）";
            core->setError("RINEX 无历元");
        }
        core->totalEpochs = ok;
        core->readDone = true;
        core->queueCv.notify_all();
        return ok;
    }

    int ReadOem7ObsToQueue(const std::shared_ptr<SolveTask> &core, const std::atomic<bool> &stopFlag) {
        const std::string &path = core->filePath;
        {
            std::lock_guard lk(core->qcInputMutex);
            core->qcInput.clear(); // 重新收集原始观测，供质量分析（与解算解耦）
        }

        OEM7Reader oem7;
        if (!oem7.open(path)) {
            LOG_ERROR << "无法打开 OEM7: " << path;
            core->setError("无法打开 OEM7");
            core->readDone = true;
            core->queueCv.notify_all();
            return -1;
        }
        {
            std::lock_guard lk(core->queueMutex);
            core->navReady = true;
        }
        core->queueCv.notify_all();

        ObsData obs;
        int ok = 0;
        while (oem7.getNextEpoch(obs)) {
            if (stopFlag) break;
            // 与 RTK 流式路径统一：复用写时复制星历快照（toe 不变时各历元共享同一 shared_ptr，避免逐历元深拷贝整张表）
            auto eph = oem7.ephSnapshot();
            {
                std::lock_guard lk(core->qcInputMutex);
                core->qcInput.push_back(QC::makeQCObsEpoch(obs, obs.weekSecond.sow));
            }
            {
                std::lock_guard lk(core->queueMutex);
                core->obsQueue.push({std::move(obs), std::move(eph)});
            }
            core->queueCv.notify_one();
            ok++;
        }
        if (ok == 0) {
            LOG_ERROR << "OEM7 无历元: " << path;
            core->setError("OEM7 无历元");
        }
        core->totalEpochs = ok;
        core->hasNav = ok > 0; // OEM7 内含星历
        core->readDone = true;
        core->queueCv.notify_all();
        return ok;
    }

    int ReadObsToQueue(const std::shared_ptr<SolveTask> &core, const std::atomic<bool> &stopFlag) {
        if (core->isRinex) return ReadRinexObsToQueue(core, stopFlag);
        return ReadOem7ObsToQueue(core, stopFlag);
    }
}
