#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "GuiRtkProcessor.h"
#include "GuiHelpers.h"
#include "core/PathMemory.h"
#include "core/DualObsPairing.h"
#include "imgui.h"
#include "RTK.h"
#include "RinexObsReader.h"
#include "RinexNavStore.h"
#include "OEM7Reader.h"
#include "ObsFormat.h"
#include "CoordConvert.h"
#include "Exception.h"
#include "QualityControl.h"
#include "Log.h"
#include "SolveDriver.h"
#include "OEM7SocketReader.h"
#include "GuiRealtimeProcessor.h"
#include "GuiSolverPanel.h"

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
#include <cstdlib>


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

    static void renderBaseCoordInput(RtkTask *task) {
        ImGui::Checkbox("手动指定基站坐标 (ECEF/m)", &task->useBaseCoord);
        if (!task->useBaseCoord) return;
        ImGui::PushItemWidth(200);
        ImGui::InputDouble("X##rtk_base_x", &task->baseXyz.x());
        ImGui::SameLine();
        ImGui::InputDouble("Y##rtk_base_y", &task->baseXyz.y());
        ImGui::SameLine();
        ImGui::InputDouble("Z##rtk_base_z", &task->baseXyz.z());
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
                if (cnt >= 1) task->baseXyz.x() = val[0];
                if (cnt >= 2) task->baseXyz.y() = val[1];
                if (cnt >= 3) task->baseXyz.z() = val[2];
            }
        }
    }

    static void persistBaseCoord(const RtkTask *task) {
        const auto &cfg = AppConfig::PathMemory::instance();
        cfg.set("rtk_base_manual", task->useBaseCoord ? "1" : "0");
        std::stringstream ss;
        ss << task->baseXyz.x() << ',' << task->baseXyz.y() << ',' << task->baseXyz.z();
        cfg.set("rtk_base_xyz", ss.str());
    }

    static void ReaderThread(const std::shared_ptr<RtkTask> &task) {
        try {
            const GnssTask::ObsFormat fmtBase = GnssTask::classifyObsPath(task->baseObsPathBuf);
            const GnssTask::ObsFormat fmtRover = GnssTask::classifyObsPath(task->roverObsPathBuf);
            if (fmtBase == GnssTask::ObsFormat::Unknown || fmtRover == GnssTask::ObsFormat::Unknown) {
                failRead(task, "无法识别观测文件格式");
                return;
            }
            const bool baseRinex = fmtBase == GnssTask::ObsFormat::Rinex;
            const bool roverRinex = fmtRover == GnssTask::ObsFormat::Rinex;
            const bool anyRinex = baseRinex || roverRinex; // 任一侧 RINEX 即需外部广播星历

            {
                std::lock_guard lk(task->core->qcInputMutex);
                task->core->qcInput.clear();
            }
            task->core->extraQcTabs.clear();
            task->core->qcTabLabel = "质量分析[流动站]";
            task->core->qcLazy = true; // 两份 QC 推迟到打开对应标签时算，加快首屏加载
            std::vector<QC::QCObsEpoch> baseQcInput;
            auto baseQc = std::make_shared<SolveTask>();
            baseQc->mode = SolverMode::Rtk;
            baseQc->filePath = task->baseObsPathBuf;
            baseQc->fileName = Helpers::baseName(task->baseObsPathBuf);
            baseQc->qcLazy = true;

            if (anyRinex) {
                if (task->navPathBuf.empty()) {
                    failRead(task, "RINEX 模式需要广播星历(导航)文件（任一观测文件为 RINEX 即需要）");
                    return;
                }
                std::string navErr;
                if (!SolveDriver::loadRtkNav(task->navPathBuf, task->navTable, navErr)) {
                    failRead(task, "广播星历无法加载");
                    return;
                }
                LOG_INFO << "RTK 读取广播星历: " << task->navPathBuf
                        << " gps=" << task->navTable.gps.size()
                        << " bds=" << task->navTable.bds.size();
            }

            // per-side reader + 首跳快进对齐 + 配对器：与 rtk_cli 共用同一套
            // （原两端各写一份 makePull/probeRinexFirst/快进，近 90 行逐字相同）。
            SolveDriver::DualObsHandles h;
            SolveDriver::DualObsPairing::Puller nextBase, nextRover;
            std::string openErr;
            if (!SolveDriver::openDualObsSources(baseRinex, roverRinex, task->baseObsPathBuf,
                                                 task->roverObsPathBuf, h, nextBase, nextRover, openErr)) {
                failRead(task, openErr);
                return;
            }

            // ---- 配对入队（原 OEM7/RINEX 两分支循环体相同，合并；差异仅在星历分发）----
            int ok = 0;
            SolveDriver::DualObsPairing p(nextBase, nextRover);
            SolveDriver::DualObsPairing::Pair pr;
            while (true) {
                if (task->stop) break;
                if (!p.next(pr)) break;
                // 匹配成功，入队解算（近似坐标由 RTK 内部自举）。
                // 星历分发：base 快照优先 → rover 快照（混合时唯一来源）→ nullptr（纯 RINEX 回落 navTable）。
                {
                    std::lock_guard lk(task->pairMtx);
                    task->pairQueue.push({pr.base, pr.rover, pr.baseEph ? pr.baseEph : pr.roverEph});
                }
                task->pairCv.notify_one();
                {
                    std::lock_guard lk(task->core->qcInputMutex);
                    task->core->qcInput.push_back(QC::makeQCObsEpoch(pr.rover, pr.rover.weekSecond.sow));
                }
                baseQcInput.push_back(QC::makeQCObsEpoch(pr.base, pr.base.weekSecond.sow));
                ++ok;
            }
            task->totalEpochs = ok;

            task->readDone = true;
            task->core->readDone = true;
            task->core->queueCv.notify_all();
            task->pairCv.notify_all();
            {
                std::lock_guard lk(task->core->qcInputMutex);
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

    static void commitRtkEpoch(const std::shared_ptr<RtkTask> &task, const ObsData &rover) {
        EpochData data;
        data.getFromObs(rover);
        SolveDriver::feedRtk(data, task->rtk);
        const Eigen::Vector3d baseXyz = task->rtk.mBaseXyz;
        GnssTask::CommitEpoch(task->core, std::move(data), [&](int, EpochData &) {
            if (!task->core->initializedRefECEF) {
                task->core->refECEF = XYZ(baseXyz.x(), baseXyz.y(), baseXyz.z());
                task->core->initializedRefECEF = true;
            }
        });
    }

    static void pushQcWindow(std::vector<QC::QCObsEpoch> &dst,
                             const ObsData &epoch, const double sow, const size_t window) {
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
                try { ok = task->rtk.processEpoch(item.base, item.rover, *eph); } catch (const std::exception &e) {
                    LOG_ERROR << "[RTK] processEpoch 异常: " << e.what();
                    ok = false;
                } catch (...) {
                    LOG_ERROR << "[RTK] processEpoch 未知异常";
                    ok = false;
                }

                if (!ok) continue;

                if (const Result &r = task->rtk.result; r.numSats <= 0) continue;

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
                                    const RealtimeProcessor::ConnectionConfig &baseCfg,
                                    const RealtimeProcessor::ConnectionConfig &roverCfg) {
        task->rtkCfg.cutoffElevRad = task->cutoffDeg * DEG_TO_RAD;
        task->rtkCfg.enabledSystems = task->enabledSystems;
        task->rtk.setConfig(task->rtkCfg);
        task->rtk.mBaseXyzUser = task->useBaseCoord ? task->baseXyz : Eigen::Vector3d::Zero();

        {
            std::lock_guard lk(task->core->qcInputMutex);
            task->core->qcInput.clear();
        }
        task->core->qcTabLabel = "质量分析[流动站]";
        auto baseQc = std::make_shared<SolveTask>();
        baseQc->mode = SolverMode::Rtk;
        baseQc->isRealtime = true;
        baseQc->filePath = task->core->rtEndpoints[0].ip;
        baseQc->fileName = Helpers::baseName(task->core->rtEndpoints[0].ip);
        {
            std::lock_guard lk(task->core->mutex);
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
                baseRdr.setReceiveTimeout(100);
                roverRdr.setReceiveTimeout(100);

                SolveDriver::DualObsPairing pairing;
                std::chrono::steady_clock::time_point lastQcTime;

                // 双流轮询 + 配对核心与 rtk_cli 共用（SolveDriver::pumpRealtimePair）；
                // GUI 侧差异仅在 stop 判定（task/core 双标志）、QC 窗口与 LaunchQC 节奏。
                // 注意：半开连接判死已收进 pumpRealtimePair（按"主动轮询却拿不到历元"计时），
                // 这里绝不能再按 socket 字节静默判死——配对器会长期握住一侧历元去追另一侧，
                // 被握住的那侧不读取、字节时间自然过期，会误杀正常连接。
                auto getBase = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> int {
                    if (!baseRdr.getNextEpoch(o)) return baseRdr.isConnected() ? 0 : -1;
                    e = baseRdr.ephSnapshot();
                    return 1;
                };
                auto getRover = [&](ObsData &o, std::shared_ptr<EphemerisTable> &e) -> int {
                    if (!roverRdr.getNextEpoch(o)) return roverRdr.isConnected() ? 0 : -1;
                    e = roverRdr.ephSnapshot();
                    return 1;
                };
                SolveDriver::pumpRealtimePair(
                    pairing, getBase, getRover,
                    [&] { return task->stop.load() || task->core->stop.load(); },
                    [] { return true; },
                    [&](const ObsData &b, const ObsData &r, EphemerisTable *eph) -> bool {
                        bool ok = false;
                        try { ok = task->rtk.processEpoch(b, r, *eph); } catch (const std::exception &e) {
                            LOG_ERROR << "[RTK实时] processEpoch 异常: " << e.what();
                            ok = false;
                        } catch (...) { ok = false; }

                        if (ok) {
                            if (const Result &res = task->rtk.result; res.numSats > 0) {
                                commitRtkEpoch(task, r);

                                {
                                    std::lock_guard lk(task->core->qcInputMutex);
                                    pushQcWindow(task->core->qcInput, r, r.weekSecond.sow, kRtkRealtimeQcWindow);
                                }
                                {
                                    std::lock_guard lk(baseQc->qcInputMutex);
                                    pushQcWindow(baseQc->qcInput, b, b.weekSecond.sow, kRtkRealtimeQcWindow);
                                }

                                if (const auto nowQc = std::chrono::steady_clock::now();
                                    nowQc - lastQcTime > std::chrono::milliseconds(500)) {
                                    lastQcTime = nowQc;
                                    SolverPanel::LaunchQC(task->core);
                                    SolverPanel::LaunchQC(baseQc);
                                }
                            }
                        }
                        return true;
                    });
                baseRdr.close();
                roverRdr.close();
                // pumpRealtimePair 返回只有两种含义：上游要求停止，或数据流中断
                // （对端关闭 / 半开连接被 10s 无历元判死）。中断时必须给用户可见反馈并退避，
                // 否则界面表现为"突然不再出历元且毫无提示"，且会立刻重连形成热循环。
                if (!(task->stop.load() || task->core->stop.load())) {
                    LOG_WARN << "[RTK实时] 数据流中断，1s 后重连 base=" << baseCfg.ip << ":" << baseCfg.port
                             << " rover=" << roverCfg.ip << ":" << roverCfg.port;
                    task->core->setError("实时数据流中断（连接关闭或长时间无历元），正在重连 ...");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
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
        Helpers::beginConfigBody("RTK 处理配置###rtk_cfg_", *this);

        if (ImGui::BeginTabBar("##rtk_cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                // 与 PPP/LEO 完全一致：直接复用 GuiHelpers 预置过滤器（fObs 已含 *.??O 与 *.log，fRnx 含广播星历）
                Helpers::fileRow("基准站观测文件", &this->baseObsPathBuf, "rtk_base", Helpers::fObs, 3,
                                 {}, nullptr, &this->navPathBuf);
                Helpers::fileRow("流动站观测文件", &this->roverObsPathBuf, "rtk_rover", Helpers::fObs, 3,
                                 {}, nullptr, &this->navPathBuf);
                Helpers::fileRow("广播星历 BRDC", &this->navPathBuf, "rtk_nav", Helpers::fRnx, 2, {}, "rtk_base");
                ImGui::Separator();
                Helpers::renderCutoffAndSystems(&this->cutoffDeg, &this->enabledSystems);

                ImGui::Separator();
                renderBaseCoordInput(this);

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {

                    const auto &cfg = AppConfig::PathMemory::instance();
                    cfg.set("rtk_base", this->baseObsPathBuf);
                    cfg.set("rtk_rover", this->roverObsPathBuf);
                    cfg.set("rtk_nav", this->navPathBuf);
                    cfg.setNum("rtk_cutoff", this->cutoffDeg);
                    cfg.set("rtk_systems", sysSetToString(this->enabledSystems));
                    persistBaseCoord(this);
                    this->core->enabledSystems = this->enabledSystems;
                    this->core->cutoffDeg = this->cutoffDeg;

                    // 输入校验与 ReaderThread / rtk_cli 共用同一份判定
                    if (const std::string inputErr = SolveDriver::rtkFileInputError(
                            this->baseObsPathBuf, this->roverObsPathBuf, this->navPathBuf);
                        !inputErr.empty()) {
                        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "%s", inputErr.c_str());
                    } else {
                        this->core->filePath = this->roverObsPathBuf;
                        this->core->fileName = Helpers::baseName(this->roverObsPathBuf) + " / "
                                               + Helpers::baseName(this->baseObsPathBuf);
                        LOG_INFO << "[RTK] 开始解算: 基准=" << this->baseObsPathBuf
                                << ", 流动站=" << this->roverObsPathBuf;

                        // 与单站三家（Helpers::beginFileSolve）一致：旧 worker 先 join
                        // （未 join 就赋新 thread 会 std::terminate），再清上一轮取消残留的
                        // stop（不清则本轮 0 历元且无报错）。
                        if (this->worker.joinable()) this->worker.join();
                        this->stop = false;
                        this->core->stop = false;
                        // 参考点状态随每次解算复位（同 beginFileSolve）：initializedRefECEF
                        // 残留会让本轮沿用上一轮锁存的参考点，基线/ENU/CSV 全部锚错。
                        this->core->initializedRefECEF = false;
                        this->core->refAutoConfirmed = false;

                        this->state = State::Running;
                        this->loading = true;
                        this->core->loading = true;
                        this->worker = std::thread(SolveThread, std::static_pointer_cast<RtkTask>(shared_from_this()));
                    }
                }
                ImGui::SameLine();
                Helpers::renderCancelButton(this);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("实时")) {
                const auto &cfg = AppConfig::PathMemory::instance();
                Helpers::renderRealtimeConfig(
                    this, &this->core->rtEndpoints, {"基准站", "流动站"},
                    &this->cutoffDeg, &this->enabledSystems,
                    [&] {
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
                        persistBaseCoord(this);
                        this->core->enabledSystems = this->enabledSystems;
                        this->core->cutoffDeg = this->cutoffDeg;

                        this->isRealtime = true;
                        this->core->isRealtime = true;
                        this->core->mode = SolverMode::Rtk;
                        this->core->fileName = bip + ":" + bport + " / " + rip + ":" + rport;
                        this->core->filePath = this->core->fileName;
                        LOG_INFO << "[RTK] 开始实时解算: base=" << bip << ":" << bport
                                << ", rover=" << rip << ":" << rport;

                        RealtimeProcessor::ConnectionConfig baseCfg;
                        baseCfg.ip = bip;
                        baseCfg.port = std::atoi(bport.c_str()); //NOLINT
                        RealtimeProcessor::ConnectionConfig roverCfg;
                        roverCfg.ip = rip;
                        roverCfg.port = std::atoi(rport.c_str()); //NOLINT

                        this->state = State::Running;
                        this->loading = true;
                        this->core->loading = true;
                        if (this->worker.joinable()) this->worker.join();
                        this->stop = false;
                        this->core->stop = false;
                        // 参考点状态随每次解算复位（同 beginFileSolve）：initializedRefECEF
                        // 残留会让本轮沿用上一轮锁存的参考点，基线/ENU/CSV 全部锚错。
                        this->core->initializedRefECEF = false;
                        this->core->refAutoConfirmed = false;
                        this->worker = std::thread(SolveRealtimeThread,
                                                   std::static_pointer_cast<RtkTask>(shared_from_this()),
                                                   baseCfg, roverCfg);
                    },
                    [&] { renderBaseCoordInput(this); });
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        Helpers::endConfigBody();
    }
}
