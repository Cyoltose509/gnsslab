#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "SolverTask.h"

#include "RinexObsReader.h"
#include "PPPStatic.h"        // RunSingleStationSolve<PPPStatic> / <PPPKinematic> 的显式实例化需要完整定义
#include "PPPKinematic.h"
#include "LEO.h"
#include "QualityControl.h"   // QC::makeQCObsEpoch
#include "Log.h"
#include "SolveDriver.h"      // 单站解算编排共享层：fillFrom* / solveStationStream（CLI 与 GUI 共用）

#include <chrono>
#include <fstream>
#include <string>

namespace GnssTask {
    // ----------------------------------------------------------
    // SolverTaskBase：错误状态的线程安全读写
    // ----------------------------------------------------------
    // hasError 是 atomic，渲染线程可无锁判断；errorMsg 是 std::string，必须加锁访问。
    // errorMutex 只保护 errorMsg，且始终是最内层锁（不在持有时去取别的锁）。
    bool SolverTaskBase::errorFlag() const { return hasError.load(); }

    std::string SolverTaskBase::errorMessage() const {
        std::lock_guard<std::mutex> lk(errorMutex);
        return errorMsg;
    }

    void SolverTaskBase::setError(std::string msg) {
        std::lock_guard<std::mutex> lk(errorMutex);
        errorMsg = std::move(msg);
        hasError.store(true); // 先写完消息再置标志，保证标志为真时消息已可见
    }

    void SolverTaskBase::clearError() {
        std::lock_guard<std::mutex> lk(errorMutex);
        errorMsg.clear();
        hasError.store(false);
    }

    void SolverTaskBase::syncErrorFrom(const SolverTaskBase &src) {
        // src 与 this 是不同对象：取完 src 的快照（原子+锁）后再写自身，
        // 不在持有 src.errorMutex 时去取 this->errorMutex，不引入任何锁序依赖。
        bool h = false;
        std::string m;
        {
            std::lock_guard<std::mutex> lk(src.errorMutex);
            h = src.hasError.load();
            m = src.errorMsg;
        }
        {
            std::lock_guard<std::mutex> lk(errorMutex);
            errorMsg = std::move(m);
            hasError.store(h);
        }
    }

    // EpochData::getFromObs / getFromSPP 已提升为 SolverTask.h 的 inline 成员（CLI 不链本 .cpp 亦可），
    // 故此处不再提供定义；SolveDriver 的 fillFromObs/fillFromSPP 薄封装转调它们。

    // ----------------------------------------------------------
    // 解算线程主循环的三段公共件（SPP / PPP / LEO / RTK 共用）
    // ----------------------------------------------------------
    // 等广播星历就绪。返回 false = 任务被取消。
    bool WaitNavReady(const std::shared_ptr<SolveTask> &core) {
        std::unique_lock lk(core->queueMutex);
        core->queueCv.wait(lk, [&] { return core->navReady.load() || core->stop.load(); });
        return !core->stop.load();
    }

    // 阻塞取下一个观测历元。false = 队列已读完或任务被取消（解算循环应结束）。
    // 注意「队列空但读取未结束」时不能就此收尾：读取线程置 navReady 早于首历元入队，
    // 此刻队列确实为空，必须继续等。
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
            return false;  // 队列空 ⇒ 上一条 wait 只可能因 readDone 或 stop 醒来
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
            // 绘图插入失败（例如残差 map 分配失败）不该让整次解算中断，历元已入列
        }
        if (core->selectedEpoch == -1 || core->selectedEpoch == idx - 1) core->selectedEpoch = idx;
        core->solvingProgress = idx + 1;
        core->solvedCount = static_cast<int>(core->epochs.size());
    }

    // 渲染侧同步：外层 Task 的 loading/done/错误态镜像到共享 core。
    void SyncTaskStatus(const std::shared_ptr<SolverTaskBase> &task, const std::shared_ptr<SolveTask> &core) {
        core->loading.store(task->loading.load());
        core->done.store(task->done.load());
        core->syncErrorFrom(*task);
    }

    // ----------------------------------------------------------
    // 单站解算器（SPP / PPP / LEO）的唯一解算线程主体
    // ----------------------------------------------------------
    // 解算本身由各解算器的 processEpoch() 承担（SPP::processEpoch / PPP::processEpoch / LEO::processEpoch
    // 三个 override 同名同签名），由 SolveDriver::solveStream 统一调用并回填；这里再包上
    // "等星历 → 入列 → 收尾"，三家 Processor 只剩 init/body/beforeSolve 三段差异。
    // GUI 侧把 SolveTask 的观测队列包装成 ObsSource，使单站解算与 CLI 共用同一条循环骨架。
    struct TaskQueueSource : SolveDriver::ObsSource {
        std::shared_ptr<SolveTask> core;
        explicit TaskQueueSource(std::shared_ptr<SolveTask> c) : core(std::move(c)) {}
        bool next(GnssTask::ObsItem &out) override {
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
            for (char c : core->enabledSystems) sys += c;
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
            // 钩子经 job 的虚函数转出：解算与回填已在 SolveDriver 内完成，
            // 这里只负责「调钩子 → 入列 → 记账」。下面两个 lambda 是本文件与 SolveDriver
            // 之间的固定转接层，各家差异全在 job 的虚函数实现里，故调用处无需组装回调。
            SolveDriver::solveStream(src, solver,
                [core, job, &solver](Solver &s, ObsItem &item, EpochData &data) {
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
                    return true;   // 已自行处理 solved 标记，交由外层原样收下
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

    void PlotData::insert(int index, const EpochData &ep, const XYZ &refECEF) {
        times.push_back(index);
        const auto &result = ep.sppResult;
        const bool solved = ep.solved;
        sigmaPs.push_back(solved ? result.sigmaP : 0.0);
        sigmaVs.push_back(solved ? result.sigmaV : 0.0);
        pdops.push_back(solved ? result.pdop : 0.0);
        if (solved) {
            auto enu = XYZtoENU(result.xyz, refECEF);
            enu_e.push_back(enu[0]);
            enu_n.push_back(enu[1]);
            enu_u.push_back(enu[2]);
        } else {
            enu_e.push_back(0);
            enu_n.push_back(0);
            enu_u.push_back(0);
        }
        if (solved)
            for (auto &[satID, res]: result.postRes) {
                satResTimes[satID].push_back(index);
                satResVals[satID].push_back(res);
            }
        newed = true;
    }

    void PlotData::refreshENU(const std::vector<EpochData> &ep, const XYZ &refECEF) {
        for (int i = 0; i < (int) enu_e.size(); i++) {
            if (ep[i].solved) {
                auto enu = XYZtoENU(ep[i].sppResult.xyz, refECEF);
                enu_e[i] = enu[0];
                enu_n[i] = enu[1];
                enu_u[i] = enu[2];
            } else {
                enu_e[i] = 0;
                enu_n[i] = 0;
                enu_u[i] = 0;
            }
        }
    }

    void PlotData::clear() {
        times.clear();
        sigmaPs.clear();
        sigmaVs.clear();
        pdops.clear();
        enu_e.clear();
        enu_n.clear();
        enu_u.clear();
        satResTimes.clear();
        satResVals.clear();
        resRangeReady = false;
        resYlo = -8.0;
        resYhi = 8.0;
    }

    // ----------------------------------------------------------
    // 伴生文件扫描
    // ----------------------------------------------------------
    std::vector<std::string> ScanNavFiles(const std::string &obsPath) {
        std::vector<std::string> result;
        if (obsPath.size() < 4) return result;
        const std::string base = obsPath.substr(0, obsPath.size() - 1);
        // RINEX 3: .??N, .??G, .??C, .??F 等
        for (auto &ext: {"N", "G", "C", "F", "E", "J", "I", "L"}) {
            std::string np = base + ext;
            std::ifstream t(np);
            if (t.good()) {
                t.close();
                result.push_back(np);
            }
        }
        return result;
    }

    // 把单历元原始观测转成 QC 输入由 QC::makeQCObsEpoch 统一完成（见 QCProcessor.h），三端共用。

    // ----------------------------------------------------------
    // RINEX 观测读取公共主体（SPP / PPP / LEO 的读取线程共用）
    // ----------------------------------------------------------
    // 三处原 ReaderThread 逐行雷同的部分：清 QC 输入 → 发 navReady 信号 → 打开 RINEX →
    // 逐历元 makeQCObsEpoch 并入 obsQueue → 置 totalEpochs / readDone / notify_all。
    //
    // 四点差异留在调用方（强行参数化只会让签名变绕）：
    //   1. 日志前缀不同（"读取文件" / "PPP 读取文件" / "LEO 读取文件"）；
    //   2. 停止标志不同：SPP 用 SolveTask 自身的 stop，PPP/LEO 用外层 Task 的 stop
    //      （core->stop 不会被 Application 置位）→ stopFlag 参数；
    //   3. hasNav 取值不同（PPP=true / LEO=有 BRDC / SPP 不在此处置位）；
    //   4. 无历元时是否立即返回、以及 QC 触发时机不同（PPP/LEO 空输入置 qcReady，SPP 无条件触发）。
    int ReadRinexObsToQueue(const std::shared_ptr<SolveTask> &core, const std::atomic<bool> &stopFlag) {
        const std::string &path = core->filePath;
        {
            std::lock_guard<std::mutex> lk(core->qcInputMutex);
            core->qcInput.clear(); // 重新收集原始观测，供质量分析（与解算解耦）
        }

        // 广播星历改由解算线程 SPP/PPP/LEO::loadBrdc 在类内加载（三者一致），读取线程仅发 navReady 信号
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
                    std::lock_guard<std::mutex> lk(core->qcInputMutex);
                    core->qcInput.push_back(QC::makeQCObsEpoch(o, o.weekSecond.sow)); // 原始观测 → QC 输入
                }
                {
                    std::lock_guard lk(core->queueMutex);
                    core->obsQueue.push({std::move(o), nullptr});
                }
                core->queueCv.notify_one();
                ok++;
            } catch (const EndOfFile &) { break;             } catch (const std::exception &e) {
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
} // namespace GnssTask
