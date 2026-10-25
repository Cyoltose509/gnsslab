#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
#include "GuiRtkProcessor.h"
#include "GuiHelpers.h"   // fileRow / renderCutoffAndSystems / inputTextStd / beginConfigBody / renderCancelButton
#include "core/PathMemory.h"
#include "core/DualObsPairing.h" // 双站历元配对统一抽象（CLI 与 GUI 共用）
#include "ui/Gui.h"
#include "imgui.h"
#include "RTK.h"               // 手写 RTK 双差非组合求解器（自举基准/流动站线性化初值）
#include "RinexObsReader.h"
#include "RinexNavStore.h"
#include "OEM7Reader.h"
#include "ObsFormat.h"      // classifyObsPath：双站格式一致性与分发判别
#include "CoordConvert.h"
#include "Exception.h"         // EndOfFile, SyncException
#include "QualityControl.h"    // QC::makeQCObsEpoch / QualityReport
#include "Log.h"
#include "SolveDriver.h"       // 解算编排共享层：feedRtk（CLI 与 GUI 共用回填实现）
#include "OEM7SocketReader.h"  // 实时流：基准站/流动站两路 OEM7 socket 读取
#include "GuiRealtimeProcessor.h" // ConnectionConfig（与 SPP 实时流共用）
#include "GuiSolverPanel.h"    // LaunchQC（实时 QC 节流）

#include <chrono>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <set>
#include <string>
#include <vector>
#include <cmath>
#include <cstdlib>

using Eigen::Vector3d;

namespace Solvers::RtkProcessor {
    using GnssTask::EpochData;
    using GnssTask::SolveTask;
    using GnssTask::SolverMode;

    static constexpr size_t kRtkRealtimeQcWindow = 7200;

    static void failRead(const std::shared_ptr<RtkTask> &task, const std::string &msg) {
        task->core->setError(msg);
        task->readDone = true;
        task->core->readDone = true;
        task->core->loading = false;
        task->core->queueCv.notify_all();
    }

    static void ReaderThread(const std::shared_ptr<RtkTask> &task) {
        try {
            // 双站格式必须一致：旧逻辑"任一端是 .log 就整队走 OEM7"会把混合格式
            // （RINEX 基准 + OEM7 流动）送进 OEM7Reader 必败，错误信息还指不到真正原因
            const GnssTask::ObsFormat fmtBase = GnssTask::classifyObsPath(task->baseObsPathBuf);
            const GnssTask::ObsFormat fmtRover = GnssTask::classifyObsPath(task->roverObsPathBuf);
            if (fmtBase == GnssTask::ObsFormat::Unknown || fmtRover == GnssTask::ObsFormat::Unknown) {
                failRead(task, "无法识别观测文件格式（支持 RINEX .??O/.rnx/.obs 与 OEM7 .log/.oem7）");
                return;
            }
            if (fmtBase != fmtRover) {
                failRead(task, "基准站与流动站文件格式不一致（需同为 RINEX 或同为 OEM7）");
                return;
            }
            const bool oem7 = (fmtBase == GnssTask::ObsFormat::Oem7);

            // 质量分析分两标签：主页=流动站，附加页=基准站。每页自带 qcInput/qcReport。
            {
                std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
                task->core->qcInput.clear();
            }
            task->core->extraQcTabs.clear();
            task->core->qcTabLabel = "质量分析[流动站]";
            task->core->qcLazy = true;   // 两份 QC 推迟到打开对应标签时算，加快首屏加载
            std::vector<QC::QCObsEpoch> baseQcInput;
            auto baseQc = std::make_shared<SolveTask>();
            baseQc->mode = SolverMode::Rtk;
            baseQc->filePath = task->baseObsPathBuf;
            baseQc->fileName = Solvers::Helpers::baseName(task->baseObsPathBuf);
            baseQc->qcLazy = true;

            if (oem7) {
                // ---- OEM7 流式路径（逐历元读取，避免 readAll 急切加载整文件；双站时间对齐见 app/core/DualObsPairing.h）----
                OEM7Reader br, rr;
                if (!br.open(task->baseObsPathBuf) || !rr.open(task->roverObsPathBuf)) {
                    failRead(task, "无法打开 OEM7 日志");
                    return;
                }

                // 双路配对窗口自适应采样间隔（原理见 app/core/DualObsPairing.h）：
                // tol = 半个采样间隔，1Hz/5Hz/10Hz 都不用改代码；未就绪时为 0，
                // 只推进不配对，绝不会把相邻历元错配。统一走 SolveDriver::DualObsPairing。
                auto nextBase = [&](ObsData &out, std::shared_ptr<EphemerisTable> &eph) -> bool {
                    if (!br.getNextEpoch(out)) return false;
                    eph = br.ephSnapshot();
                    return true;
                };
                auto nextRover = [&](ObsData &out, std::shared_ptr<EphemerisTable> &eph) -> bool {
                    if (!rr.getNextEpoch(out)) return false;
                    eph = rr.ephSnapshot();
                    return true;
                };

                int ok = 0;
                SolveDriver::DualObsPairing p(nextBase, nextRover);
                SolveDriver::DualObsPairing::Pair pr;
                while (true) {
                    if (task->stop) break;
                    if (!p.next(pr)) break;
                    // 匹配成功，入队解算（近似坐标由 RTK 内部自举）
                    {
                        std::lock_guard lk(task->pairMtx);
                        task->pairQueue.push({pr.base, pr.rover, pr.baseEph});
                    }
                    task->pairCv.notify_one();
                    {
                        std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
                        task->core->qcInput.push_back(QC::makeQCObsEpoch(pr.rover, pr.rover.weekSecond.sow));
                    }
                    baseQcInput.push_back(QC::makeQCObsEpoch(pr.base, pr.base.weekSecond.sow));
                    ++ok;
                }
                task->totalEpochs = ok;
            } else {
                // ---- RINEX 路径 ----
                if (task->navPathBuf.empty()) {
                    failRead(task, "RINEX 模式需要广播星历(导航)文件");
                    return;
                }
                {
                    RinexNavStore nav;
                    nav.loadFile(task->navPathBuf, task->navTable);
                }
                if (task->navTable.gps.empty() && task->navTable.bds.empty()) {
                    failRead(task, "广播星历无法加载");
                    return;
                }
                LOG_INFO << "RTK 读取广播星历: " << task->navPathBuf
                         << " gps=" << task->navTable.gps.size()
                         << " bds=" << task->navTable.bds.size();

                RinexObsReader baseRdr, roverRdr;
                std::fstream baseFile(task->baseObsPathBuf, std::ios::in);
                std::fstream roverFile(task->roverObsPathBuf, std::ios::in);
                if (!baseFile.is_open() || !roverFile.is_open()) {
                    failRead(task, "无法打开 RINEX 观测文件");
                    return;
                }
                baseRdr.pFileStream = &baseFile;
                roverRdr.pFileStream = &roverFile;

                // 失败快出：基准站首历元解析（同时校验文件可用性）
                try {
                    baseRdr.parseRinexObs();
                } catch (...) {
                    failRead(task, "基准站 RINEX 解析失败");
                    return;
                }
                // 把基准站流跳到流动站首历元附近（保留首跳加速优化）
                {
                    std::fstream probeFile(task->roverObsPathBuf, std::ios::in);
                    if (probeFile.is_open()) {
                        RinexObsReader probeRdr;
                        probeRdr.pFileStream = &probeFile;
                        try {
                            const ObsData rover0 = probeRdr.parseRinexObs();
                            baseRdr.fastForwardToEpoch(rover0.epoch);
                        } catch (const std::exception &) {
                            // 探测失败不致命：退回原始逐历元同步
                        }
                    }
                }

                // 流动站线性化初值由 RTK::processEpoch 内部自举（首历元 antennaPosition 或 SPP）。
                // 双站配对统一走 SolveDriver::DualObsPairing：两路各自独立取下一历元（无参
                // parseRinexObs），由配对器按时间对齐；首跳加速已在上面完成。RINEX 推入
                // pairQueue 的 eph 为 nullptr，由 SolverThread 回退到全量 navTable。
                auto nextBase = [&](ObsData &out, std::shared_ptr<EphemerisTable> &eph) -> bool {
                    try { out = baseRdr.parseRinexObs(); }
                    catch (const EndOfFile &) { return false; } catch (...) { return false; }
                    eph = nullptr;
                    return true;
                };
                auto nextRover = [&](ObsData &out, std::shared_ptr<EphemerisTable> &eph) -> bool {
                    try { out = roverRdr.parseRinexObs(); }
                    catch (const EndOfFile &) { return false; } catch (...) { return false; }
                    eph = nullptr;
                    return true;
                };
                int ok = 0;
                SolveDriver::DualObsPairing p(nextBase, nextRover);
                SolveDriver::DualObsPairing::Pair pr;
                while (true) {
                    if (task->stop) break;
                    if (!p.next(pr)) break;
                    {
                        std::lock_guard lk(task->pairMtx);
                        task->pairQueue.push({pr.base, pr.rover, nullptr});
                    }
                    task->pairCv.notify_one();
                    {
                        std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
                        task->core->qcInput.push_back(QC::makeQCObsEpoch(pr.rover, pr.rover.weekSecond.sow));
                    }
                    baseQcInput.push_back(QC::makeQCObsEpoch(pr.base, pr.base.weekSecond.sow));
                    ++ok;
                }
                task->totalEpochs = ok;
            }

            task->readDone = true;
            task->core->readDone = true;
            task->core->queueCv.notify_all();
            task->pairCv.notify_all();
            {
                std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
                if (task->core->qcInput.empty()) task->core->qcReady = true;
            }
            if (!baseQcInput.empty()) baseQc->qcInput = std::move(baseQcInput);
            baseQc->readDone = true;
            baseQc->done = true;
            {
                std::lock_guard lk(task->core->mutex);
                task->core->extraQcTabs.push_back({"质量分析[基准站]", baseQc});
            }
        } catch (const std::exception &e) {
            task->core->setError(std::string("RTK 读取错误: ") + e.what());
            task->readDone = true;
            task->core->readDone = true;
            task->core->queueCv.notify_all();
            task->pairCv.notify_all();
        }
        task->core->loading = false;
    }

    // 收口本历元 RTK 结果：构造 EpochData → SolveDriver::feedRtk 填充 result/几何/排除标记 → 提交；
    // 首次提交时把基准站坐标作为 refECEF 初值（与 SPP/PPP/LEO 同形，统一经 GnssTask::CommitEpoch）。
    // 文件模式与实时模式共用，消除两份逐字复制的提交代码。
    static void commitRtkEpoch(const std::shared_ptr<RtkTask> &task, const ObsData &rover) {
        EpochData data;
        data.getFromObs(rover);
        SolveDriver::feedRtk(data, task->rtk);
        const Vector3d baseXyz = task->rtk.mBaseXyz;
        GnssTask::CommitEpoch(task->core, std::move(data), [&](int, EpochData &) {
            if (!task->core->initializedRefECEF) {
                task->core->refECEF = XYZ(baseXyz.x(), baseXyz.y(), baseXyz.z());
                task->core->initializedRefECEF = true;
            }
        });
    }

    // 把 QC 样本压入带滑动窗口截断的 QC 输入队列（实时流防无限增长）。调用方须持有对应 qcInputMutex。
    static void pushQcWindow(std::vector<QC::QCObsEpoch> &dst,
                             const ObsData &epoch, double sow, size_t window) {
        dst.push_back(QC::makeQCObsEpoch(epoch, sow));
        if (dst.size() > window) dst.erase(dst.begin(), dst.begin() + (dst.size() - window));
    }

    static void SolverThread(const std::shared_ptr<RtkTask> &task) {
        try {
            while (true) {
                RtkTask::RtkPair item;
                {
                    std::unique_lock lk(task->pairMtx);
                    task->pairCv.wait(lk, [&] {
                        return !task->pairQueue.empty() || task->readDone.load() || task->stop.load();
                    });
                    if (task->pairQueue.empty() && (task->readDone.load() || task->stop.load())) break;
                    if (task->pairQueue.empty()) continue;
                    item = std::move(task->pairQueue.front());
                    task->pairQueue.pop();
                }
                if (task->stop) break;


                EphemerisTable *eph = item.eph ? item.eph.get() : &task->navTable;
                bool ok = false;
                try { ok = task->rtk.processEpoch(item.base, item.rover, *eph); }
                catch (const std::exception &e) { LOG_ERROR << "[RTK] processEpoch 异常: " << e.what(); ok = false; }
                catch (...) { LOG_ERROR << "[RTK] processEpoch 未知异常"; ok = false; }

                if (!ok) continue;

                const Result &r = task->rtk.result;
                if (r.numSats <= 0) continue;

                commitRtkEpoch(task, item.rover);
            }
        } catch (const std::exception &e) {
            task->core->setError(std::string("RTK 解算错误: ") + e.what());
        }
        task->solvingDone = true;
        task->core->solvingDone = true;
    }

    static void SolveThread(const std::shared_ptr<RtkTask> &task) {
        const auto tRtk0 = std::chrono::steady_clock::now();
        task->rtkCfg.cutoffElevRad = task->cutoffDeg * DEG_TO_RAD;
        task->rtkCfg.enabledSystems = task->enabledSystems;
        task->rtk.setConfig(task->rtkCfg);
        // 配置页显式填入的 surveyed 基站坐标（未勾选则为 0 → RTK 内部按 BESTPOS/SPP 自举）
        task->rtk.mBaseXyzUser = task->useBaseCoord ? task->baseXyz : Eigen::Vector3d::Zero();

        std::thread thRead(ReaderThread, task);
        std::thread thSolve(SolverThread, task);
        thRead.join();
        thSolve.join();

        size_t rtkEpochs = 0;
        {
            std::lock_guard lk(task->core->mutex);
            rtkEpochs = task->core->epochs.size();
        }
        LOG_INFO << "[RTK] 结束: " << rtkEpochs << " 历元, 耗时 "
                 << std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - tRtk0).count() << " ms";
        task->state = RtkTask::State::Done;
        task->done = true;
        task->loading = false;
        GnssTask::SyncTaskStatus(task, task->core);
    }

    static void SolveRealtimeThread(const std::shared_ptr<RtkTask> &task,
                                   const Solvers::RealtimeProcessor::ConnectionConfig &baseCfg,
                                   const Solvers::RealtimeProcessor::ConnectionConfig &roverCfg) {
        // 实时流不使用文件模式的全量广播星历表；星历由基准站（优先）/流动站 内嵌 OEM7 消息提供。
        task->rtkCfg.cutoffElevRad = task->cutoffDeg * DEG_TO_RAD;
        task->rtkCfg.enabledSystems = task->enabledSystems;
        task->rtk.setConfig(task->rtkCfg);
        // 配置页显式填入的 surveyed 基站坐标（未勾选则为 0 → RTK 内部按 BESTPOS/SPP 自举）
        task->rtk.mBaseXyzUser = task->useBaseCoord ? task->baseXyz : Eigen::Vector3d::Zero();

        // 实时质量分析：复用文件模式的双标签结构（主页=流动站 / 附加页=基准站）。
        // 实时流由后台线程定时触发 LaunchQC，不依赖懒加载（qcLazy 保持 false）。
        {
            std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
            task->core->qcInput.clear();
        }
        task->core->qcTabLabel = "质量分析[流动站]";
        auto baseQc = std::make_shared<GnssTask::SolveTask>();
        baseQc->mode = GnssTask::SolverMode::Rtk;
        baseQc->isRealtime = true;
        baseQc->filePath = task->core->rtEndpoints[0].ip;
        baseQc->fileName = Solvers::Helpers::baseName(task->core->rtEndpoints[0].ip);
        {
            std::lock_guard<std::mutex> lk(task->core->mutex);
            task->core->extraQcTabs.clear();
            task->core->extraQcTabs.push_back({"质量分析[基准站]", baseQc});
        }

        bool loggedConnectFail = false;
        while (!(task->stop || task->core->stop)) {
            try {
                OEM7SocketReader baseRdr, roverRdr;
                if (!baseRdr.connect(baseCfg.ip, static_cast<unsigned short>(baseCfg.port)) ||
                    !roverRdr.connect(roverCfg.ip, static_cast<unsigned short>(roverCfg.port))) {
                    if (!loggedConnectFail) {
                        LOG_WARN << "[RTK实时] 连接失败 base=" << baseCfg.ip << ":" << baseCfg.port
                                 << " rover=" << roverCfg.ip << ":" << roverCfg.port << "，每 2s 重试";
                        loggedConnectFail = true;
                    }
                    task->core->setError("正在尝试连接 base " + baseCfg.ip + ":" + std::to_string(baseCfg.port)
                                         + " / rover " + roverCfg.ip + ":" + std::to_string(roverCfg.port) + " ...");
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    continue;
                }
                task->core->clearError();
                if (loggedConnectFail) {
                    LOG_INFO << "[RTK实时] 已连接 base=" << baseCfg.ip << ":" << baseCfg.port
                             << " rover=" << roverCfg.ip << ":" << roverCfg.port;
                    loggedConnectFail = false;
                }
                // socket 接收超时设短，使双流对齐循环能及时在两路之间轮询，而非阻塞在单路读取上。
                baseRdr.setReceiveTimeout(100);
                roverRdr.setReceiveTimeout(100);

                // 双流时间对齐：|Δ|≤tol 才配对解算，否则丢弃较旧的一路重读（与文件模式对齐语义等价）。
                // 配对窗口自适应采样间隔（原理见 app/core/DualObsPairing.h 的内部采样间隔估计）：1Hz/5Hz/10Hz 通吃，
                // 未就绪时返回 0（只推进不配对），绝不会错配相邻历元。
                // 实时路径只持有一个 DualObsPairing 实例，通过 observeBase/observeRover/pairWindow 完成配对判定，
                // 不再出现 IntervalEstimator 类型名。
                SolveDriver::DualObsPairing pairing;
                ObsData base, rover;
                std::shared_ptr<EphemerisTable> baseEph, roverEph;
                bool haveBase = false, haveRover = false;
                std::chrono::steady_clock::time_point lastQcTime;

                while (!(task->stop || task->core->stop)) {
                    if (!haveBase) {
                        ObsData b;
                        if (baseRdr.getNextEpoch(b)) { base = std::move(b); baseEph = baseRdr.ephSnapshot(); pairing.observeBase(base.weekSecond.sow); haveBase = true; }
                        else if (!baseRdr.isConnected()) break;
                    }
                    if (!haveRover) {
                        ObsData r;
                        if (roverRdr.getNextEpoch(r)) { rover = std::move(r); roverEph = roverRdr.ephSnapshot(); pairing.observeRover(rover.weekSecond.sow); haveRover = true; }
                        else if (!roverRdr.isConnected()) break;
                    }
                    if (!haveBase || !haveRover) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));
                        continue;
                    }

                    // 两历元时间对齐（与文件模式同一条配对逻辑，见 app/core/DualObsPairing.h）。
                    // 实时路径是轮询模型（haveBase/haveRover + sleep），无法套用 DualObsPairing::next()，
                    // 故保留轮询结构，仅复用其 classify / selectEph，避免配对语义分叉。
                    const double tol = pairing.pairWindow();
                    const auto act = SolveDriver::DualObsPairing::classify(base.epoch, rover.epoch, tol);
                    if (act == SolveDriver::DualObsPairing::Action::Pair) {
                        EphemerisTable *eph = SolveDriver::DualObsPairing::selectEph(baseEph, roverEph);
                        bool ok = false;
                        try { ok = task->rtk.processEpoch(base, rover, *eph); }
                        catch (const std::exception &e) { LOG_ERROR << "[RTK实时] processEpoch 异常: " << e.what(); ok = false; }
                        catch (...) { ok = false; }

                        if (ok) {
                            const Result &res = task->rtk.result;
                            if (res.numSats > 0) {
                                commitRtkEpoch(task, rover);

                                // 实时质量分析：流动站入主标签 qcInput，基准站入附加标签 qcInput；
                                // 滑动窗口截断，避免 QC::compute 随历元数无限增长（增量式质量分析）。
                                {
                                    std::lock_guard<std::mutex> lk(task->core->qcInputMutex);
                                    pushQcWindow(task->core->qcInput, rover, rover.weekSecond.sow, kRtkRealtimeQcWindow);
                                }
                                {
                                    std::lock_guard<std::mutex> lk(baseQc->qcInputMutex);
                                    pushQcWindow(baseQc->qcInput, base, base.weekSecond.sow, kRtkRealtimeQcWindow);
                                }

                                const auto nowQc = std::chrono::steady_clock::now();
                                if (nowQc - lastQcTime > std::chrono::milliseconds(500)) {
                                    lastQcTime = nowQc;
                                    Solvers::SolverPanel::LaunchQC(task->core);
                                    Solvers::SolverPanel::LaunchQC(baseQc);
                                }
                            }
                        }
                        haveBase = haveRover = false;
                    } else if (act == SolveDriver::DualObsPairing::Action::AdvanceBase) {
                        haveBase = false;    // 基准站历元过时，丢弃重读
                    } else {
                        haveRover = false;   // 流动站历元过时，丢弃重读
                    }
                }
                baseRdr.close();
                roverRdr.close();
            } catch (const std::exception &e) {
                task->core->setError(std::string("RTK实时运行异常: ") + e.what());
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }

        task->loading = false;
        task->done = true;
        task->core->loading = false;
        task->core->done = true;
    }

    void RtkTask::renderConfigPanel() {
        Solvers::Helpers::beginConfigBody("RTK 处理配置###rtk_cfg_", *this);

        if (ImGui::BeginTabBar("##rtk_cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                // 与 PPP/LEO 完全一致：直接复用 GuiHelpers 预置过滤器（fObs 已含 *.??O 与 *.log，fRnx 含广播星历）
                Solvers::Helpers::fileRow("基准站观测文件", &this->baseObsPathBuf, "rtk_base", Solvers::Helpers::fObs, 3,
                                    [&](const std::string &p) {
                                        const std::string d = GnssTask::AutoDetectNav(p);
                                        if (!d.empty()) this->navPathBuf = d;
                                    });
                Solvers::Helpers::fileRow("流动站观测文件", &this->roverObsPathBuf, "rtk_rover", Solvers::Helpers::fObs, 3,
                                    [&](const std::string &p) {
                                        const std::string d = GnssTask::AutoDetectNav(p);
                                        if (!d.empty()) this->navPathBuf = d;
                                    });
                Solvers::Helpers::fileRow("广播星历 BRDC", &this->navPathBuf, "rtk_nav", Solvers::Helpers::fRnx, 2, {}, "rtk_base");
                ImGui::Separator();
                Solvers::Helpers::renderCutoffAndSystems(&this->cutoffDeg, &this->enabledSystems);

                ImGui::Separator();
                ImGui::Checkbox("手动指定基站坐标 (ECEF/m)", &this->useBaseCoord);
                if (this->useBaseCoord) {
                    ImGui::PushItemWidth(200);
                    ImGui::InputDouble("X##rtk_base_x", &this->baseXyz.x());
                    ImGui::SameLine();
                    ImGui::InputDouble("Y##rtk_base_y", &this->baseXyz.y());
                    ImGui::SameLine();
                    ImGui::InputDouble("Z##rtk_base_z", &this->baseXyz.z());
                    ImGui::PopItemWidth();
                    ImGui::SameLine();
                    if (ImGui::Button("从剪贴板获取##rtk_base_clip")) {
                        if (const char *c = ImGui::GetClipboardText()) {
                            double val[3] = {};
                            const char *p = c;
                            int cnt = 0;
                            while (*p && cnt < 3) {
                                while (*p && !(*p == '-' || (*p >= '0' && *p <= '9'))) p++;
                                if (!*p) break;
                                val[cnt] = strtod(p, const_cast<char **>(&p));
                                cnt++;
                            }
                            if (cnt >= 1) this->baseXyz.x() = val[0];
                            if (cnt >= 2) this->baseXyz.y() = val[1];
                            if (cnt >= 3) this->baseXyz.z() = val[2];
                        }
                    }
                }

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    // 记忆上次选择（项目外 ini）——无条件持久化（含清空），
                    // 确保"删掉广播星历再开始解算"后不会被旧值重新填回。
                    const auto &cfg = AppConfig::PathMemory::instance();
                    cfg.set("rtk_base", this->baseObsPathBuf);
                    cfg.set("rtk_rover", this->roverObsPathBuf);
                    cfg.set("rtk_nav", this->navPathBuf);
                    cfg.setNum("rtk_cutoff", this->cutoffDeg);
                    cfg.set("rtk_systems", sysSetToString(this->enabledSystems));
                    cfg.set("rtk_base_manual", this->useBaseCoord ? "1" : "0");
                    {
                        std::stringstream ss;
                        ss << this->baseXyz.x() << ',' << this->baseXyz.y() << ',' << this->baseXyz.z();
                        cfg.set("rtk_base_xyz", ss.str());
                    }
                    // 同步渲染层 SolveTask（卫星列表过滤、QC 标签等读取 core->enabledSystems/cutoffDeg）
                    this->core->enabledSystems = this->enabledSystems;
                    this->core->cutoffDeg = this->cutoffDeg;

                    if (this->baseObsPathBuf.empty() || this->roverObsPathBuf.empty()) {
                        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "请先选择基准站与流动站观测文件");
                    } else {
                        const GnssTask::ObsFormat fmtBase = GnssTask::classifyObsPath(this->baseObsPathBuf);
                        const GnssTask::ObsFormat fmtRover = GnssTask::classifyObsPath(this->roverObsPathBuf);
                        if (fmtBase == GnssTask::ObsFormat::Unknown || fmtRover == GnssTask::ObsFormat::Unknown) {
                            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1),
                                               "无法识别观测文件格式（支持 RINEX .??O/.rnx/.obs 与 OEM7 .log/.oem7）");
                        } else if (fmtBase != fmtRover) {
                            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1),
                                               "基准站与流动站文件格式不一致（需同为 RINEX 或同为 OEM7）");
                        } else if (fmtBase == GnssTask::ObsFormat::Rinex && this->navPathBuf.empty()) {
                            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "RINEX 模式需要广播星历文件");
                        } else {
                            this->core->filePath = this->roverObsPathBuf;
                            this->core->fileName = Solvers::Helpers::baseName(this->roverObsPathBuf) + " / "
                                                    + Solvers::Helpers::baseName(this->baseObsPathBuf);
                            LOG_INFO << "[RTK] 开始解算: 基准=" << this->baseObsPathBuf
                                     << ", 流动站=" << this->roverObsPathBuf;

                            this->state = State::Running;
                            this->loading = true;
                            this->core->loading = true;
                            this->worker = std::thread(SolveThread, std::static_pointer_cast<RtkTask>(shared_from_this()));
                        }
                    }
                }
                ImGui::SameLine();
                Solvers::Helpers::renderCancelButton(this);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("实时")) {
                const auto &cfg = AppConfig::PathMemory::instance();
                Solvers::Helpers::renderRealtimeConfig(
                    this, &this->core->rtEndpoints, {"基准站", "流动站"},
                    &this->cutoffDeg, &this->enabledSystems,
                    [&]() {
                        const std::string &bip = this->core->rtEndpoints[0].ip;
                        const std::string &bport = this->core->rtEndpoints[0].port;
                        const std::string &rip = this->core->rtEndpoints[1].ip;
                        const std::string &rport = this->core->rtEndpoints[1].port;
                        cfg.set("rtk_rt_base_ip", bip);
                        cfg.set("rtk_rt_base_port", bport);
                        cfg.set("rtk_rt_rover_ip", rip);
                        cfg.set("rtk_rt_rover_port", rport);
                        cfg.setNum("rtk_cutoff", this->cutoffDeg);
                        cfg.set("rtk_systems", sysSetToString(this->enabledSystems));
                        this->core->enabledSystems = this->enabledSystems;
                        this->core->cutoffDeg = this->cutoffDeg;

                        this->isRealtime = true;
                        this->core->isRealtime = true;
                        this->core->mode = GnssTask::SolverMode::Rtk;
                        this->core->fileName = bip + ":" + bport + " / " + rip + ":" + rport;
                        this->core->filePath = this->core->fileName;
                        LOG_INFO << "[RTK] 开始实时解算: base=" << bip << ":" << bport
                                 << ", rover=" << rip << ":" << rport;

                        Solvers::RealtimeProcessor::ConnectionConfig baseCfg;
                        baseCfg.ip = bip; baseCfg.port = std::atoi(bport.c_str());   //NOLINT
                        Solvers::RealtimeProcessor::ConnectionConfig roverCfg;
                        roverCfg.ip = rip; roverCfg.port = std::atoi(rport.c_str()); //NOLINT

                        this->state = State::Running;
                        this->loading = true;
                        this->core->loading = true;
                        if (this->worker.joinable()) this->worker.join();
                        this->stop = false;
                        this->core->stop = false;
                        this->worker = std::thread(SolveRealtimeThread,
                                                  std::static_pointer_cast<RtkTask>(shared_from_this()),
                                                  baseCfg, roverCfg);
                    });
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        Solvers::Helpers::endConfigBody();
    }

}
