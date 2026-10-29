#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "GuiRtkProcessor.h"
#include "GuiHelpers.h"
#include "core/PathMemory.h"
#include "core/DualObsPairing.h"
#include "imgui.h"
#include "RTK.h"
#include "ObsFormat.h"
#include "QualityControl.h"
#include "Log.h"
#include "SolveDriver.h"
#include "GuiRealtime.h"

#include <chrono>
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
            baseQc->sampleInterval = task->sampleInterval; // 基准站 QC 用同一标称间隔
            baseQc->qcLazy = true;

            if (anyRinex) {
                if (task->navPathBuf.empty()) {
                    failRead(task, "RINEX 模式需要广播星历(导航)文件（任一观测文件为 RINEX 即需要）");
                    return;
                }
                if (std::string navErr; !SolveDriver::loadRtkNav(task->navPathBuf, task->navTable, navErr)) {
                    failRead(task, "广播星历无法加载");
                    return;
                }
                LOG_INFO << "RTK 读取广播星历: " << task->navPathBuf
                        << " gps=" << task->navTable.gps.size()
                        << " bds=" << task->navTable.bds.size();
            }

            SolveDriver::DualObsPairing::Puller nextBase, nextRover;
            std::string openErr;
            SolveDriver::DualObsHandles h;
            if (!SolveDriver::openDualObsSources(baseRinex, roverRinex, task->baseObsPathBuf,
                                                task->roverObsPathBuf, h, nextBase, nextRover, openErr)) {
                failRead(task, openErr);
                return;
            }

            int ok = 0;
            SolveDriver::DualObsPairing p(nextBase, nextRover);
            p.setInterval(task->sampleInterval); // 声明的标称采样间隔：配对容差免窗口学习期
            SolveDriver::EpochDecimator dec(task->sampleInterval); // 解算抽取：只解算网格历元
            SolveDriver::DualObsPairing::Pair pr;
            while (true) {
                if (task->stop) break;
                if (!p.next(pr)) break;
                if (dec.accept(pr.rover.weekSecond.sow)) {
                    {
                        std::lock_guard lk(task->pairMtx);
                        task->pairQueue.push({pr.base, pr.rover, pr.baseEph ? pr.baseEph : pr.roverEph});
                    }
                    task->pairCv.notify_one();
                }
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

                SolveDriver::commitRtk(task->core, task->rtk, item.rover);
            }
        } catch (const std::exception &e) {
            task->core->setError(std::string("RTK 解算错误: ") + e.what());
        }
        task->solvingDone = true;
        task->core->solvingDone = true;
    }

    static void SolveThread(const std::shared_ptr<RtkTask> &task) {
        const auto tRtk0 = std::chrono::steady_clock::now();
        // 统一参数注入：RTK GUI 仅暴露 截止角/星座，其余取 SolverOptions 默认（= RTK::Config 默认，与 CLI 一致）
        SolverOptions opts;
        opts.cutoffDeg = task->cutoffDeg;
        opts.systems = task->enabledSystems;
        opts.sampleInterval = task->sampleInterval;
        task->rtk.applyOptions(opts);
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

    void RtkTask::renderConfigPanel() {
        Helpers::beginConfigBody("RTK 处理配置###rtk_cfg_", *this);

        if (ImGui::BeginTabBar("##rtk_cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
           Helpers::fileRow("基准站观测文件", &this->baseObsPathBuf, "rtk_base", Helpers::fObs, 3,
                                 {}, nullptr, &this->navPathBuf);
                Helpers::fileRow("流动站观测文件", &this->roverObsPathBuf, "rtk_rover", Helpers::fObs, 3,
                                 {}, nullptr, &this->navPathBuf);
                Helpers::fileRow("广播星历 BRDC", &this->navPathBuf, "rtk_nav", Helpers::fRnx, 2, {}, "rtk_base");
                ImGui::Separator();
                Helpers::renderCutoffAndSystems(&this->cutoffDeg, &this->enabledSystems, &this->sampleInterval);

                ImGui::Separator();
                renderBaseCoordInput(this);

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    const auto &cfg = AppConfig::PathMemory::instance();
                    cfg.set("rtk_base", this->baseObsPathBuf);
                    cfg.set("rtk_rover", this->roverObsPathBuf);
                    cfg.set("rtk_nav", this->navPathBuf);
                    cfg.setNum("rtk_cutoff", this->cutoffDeg);
                    cfg.setNum("rtk_interval", this->sampleInterval);
                    cfg.set("rtk_systems", sysSetToString(this->enabledSystems));
                    persistBaseCoord(this);
                    this->core->enabledSystems = this->enabledSystems;
                    this->core->cutoffDeg = this->cutoffDeg;
                    this->core->sampleInterval = this->sampleInterval;

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

                        if (this->worker.joinable()) this->worker.join();
                        this->stop = false;
                        this->core->stop = false;
                        this->core->initializedRefECEF = false;

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
                    &this->cutoffDeg, &this->enabledSystems, &this->sampleInterval,
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
                        cfg.setNum("rtk_interval", this->sampleInterval);
                        cfg.set("rtk_systems", sysSetToString(this->enabledSystems));
                        persistBaseCoord(this);
                        this->core->enabledSystems = this->enabledSystems;
                        this->core->cutoffDeg = this->cutoffDeg;
                        this->core->sampleInterval = this->sampleInterval;

                        this->isRealtime = true;
                        this->core->isRealtime = true;
                        this->core->mode = SolverMode::Rtk;
                        this->core->fileName = bip + ":" + bport + " / " + rip + ":" + rport;
                        this->core->filePath = this->core->fileName;
                        LOG_INFO << "[RTK] 开始实时解算: base=" << bip << ":" << bport
                                << ", rover=" << rip << ":" << rport;

                        this->state = State::Running;
                        this->loading = true;
                        this->core->loading = true;
                        if (this->worker.joinable()) this->worker.join();
                        this->stop = false;
                        this->core->stop = false;
                        this->core->initializedRefECEF = false;
                        this->worker = std::thread(Realtime::RunRtk,
                                                   std::static_pointer_cast<RtkTask>(shared_from_this()),
                                                   this->core->rtEndpoints[0], this->core->rtEndpoints[1]);
                    },
                    [&] { renderBaseCoordInput(this); });
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        Helpers::endConfigBody();
    }
}
