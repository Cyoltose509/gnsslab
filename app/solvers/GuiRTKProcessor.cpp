#include <windows.h>
#include "GuiRTKProcessor.h"
#include "view/GuiHelpers.h"
#include "view/GuiProductSection.h"
#include "view/GuiParamPage.h"
#include "StringUtils.h"
#include "core/PathMemory.h"
#include "DualPair.h"
#include "imgui.h"
#include "RTK.h"
#include "Observer.h"
#include "view/QualityControl.h"
#include "Log.h"
#include "solvers/common/Driver.h"
#include "io/ObsSource.h"
#include "ui/FileDialog.h"
#include "solvers/spp/Realtime.h"
#include "solvers/rtk/Realtime.h"
#include "solvers/rtk/Pipeline.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <set>
#include <string>
#include <vector>

namespace Solvers::RTK {
    using Solvers::Epoch;
    using Solvers::Core;
    using Solvers::Mode;

    static void failRead(const std::shared_ptr<Task> &task, const std::string &msg) {
        task->core->setError(msg);
        task->readDone = true;
        task->core->readDone = true;
        task->core->loading = false;
        task->core->queueCv.notify_all();
    }

    static void renderBaseCoordInput(Task *task) {
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
                const int cnt = Str::scanDoubles(c, val, 3);
                if (cnt >= 1) task->baseXyz.x() = val[0];
                if (cnt >= 2) task->baseXyz.y() = val[1];
                if (cnt >= 3) task->baseXyz.z() = val[2];
            }
        }
    }

    static void persistBaseCoord(const Task *task) {
        const auto &cfg = AppConfig::PathMemory::instance();
        cfg.set("rtk_base_manual", task->useBaseCoord ? "1" : "0");
        std::stringstream ss;
        ss << task->baseXyz.x() << ',' << task->baseXyz.y() << ',' << task->baseXyz.z();
        cfg.set("rtk_base_xyz", ss.str());
    }

    // Task 的公共配置是自己的成员（非 core 的），解算/实时线程读 core，启动前同步过去
    static void syncCommonToCore(Task *task) {
        task->core->enabledSystems = task->enabledSystems;
        task->core->cutoffDeg = task->cutoffDeg;
        task->core->sampleInterval = task->sampleInterval;
    }

    void Task::readBody() {
        auto &task = *this;
        try {
            {
                std::lock_guard lk(task.core->qcInputMutex);
                task.core->qcInput.clear();
            }
            task.core->extraQcTabs.clear();
            task.core->qcTabLabel = "质量分析[流动站]";
            task.core->qcLazy = true;
            std::vector<QC::QCObsEpoch> baseQcInput;
            auto baseQc = std::make_shared<Solvers::Core>();
            baseQc->mode = Solvers::Mode::RTK;
            baseQc->filePath = task.baseObsPathBuf;
            baseQc->fileName = Str::baseName(task.baseObsPathBuf);
            baseQc->sampleInterval = task.sampleInterval;
            baseQc->qcLazy = true;

            const EpochDecimator dec(task.sampleInterval);
            std::string pipeErr;
            const int nPair = RTK::pairFiles(
                task.baseObsPathBuf, task.roverObsPathBuf, task.navPathBuf,
                task.sampleInterval, task.navTable,
                [&] { return task.stop.load(); },
                [&](const ObsData &base, const ObsData &rover,
                    const std::shared_ptr<EphemerisTable> &eph, EphemerisTable *) {
                    {
                        std::lock_guard lk(task.core->qcInputMutex);
                        task.core->qcInput.push_back(QC::makeQCObsEpoch(rover, rover.weekSecond.sow));
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
                std::lock_guard lk(task.core->qcInputMutex);
                if (task.core->qcInput.empty()) task.core->qcReady = true;
            }
            if (!baseQcInput.empty()) baseQc->qcInput = std::move(baseQcInput);
            baseQc->readDone = true;
            baseQc->done = true;
            {
                std::lock_guard lk(task.core->mutex);
                task.core->extraQcTabs.emplace_back("质量分析[基准站]", baseQc);
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
                Driver::commitRtk(core, rtk, item.rover);
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

    void Task::renderConfigPanel() {
        Helpers::beginConfigBody("RTK 处理配置###rtk_cfg_", *this);

        if (ImGui::BeginTabBar("##rtk_cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                Helpers::fileRow("基准站观测文件", &this->baseObsPathBuf, "rtk_base", FileDialog::fObs, 3,
                                 {}, nullptr, &this->navPathBuf);
                Helpers::fileRow("流动站观测文件", &this->roverObsPathBuf, "rtk_rover", FileDialog::fObs, 3,
                                 {}, nullptr, &this->navPathBuf);
                Helpers::fileRow("广播星历 BRDC", &this->navPathBuf, "rtk_nav", FileDialog::fRnx, 2, {}, "rtk_base");
                ImGui::Separator();
                Helpers::renderSimpleConfig(&this->cutoffDeg, &this->enabledSystems, &this->sampleInterval);

                ImGui::Separator();
                renderBaseCoordInput(this);

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    const auto &cfg = AppConfig::PathMemory::instance();
                    cfg.set("rtk_base", this->baseObsPathBuf);
                    cfg.set("rtk_rover", this->roverObsPathBuf);
                    cfg.set("rtk_nav", this->navPathBuf);
                    Helpers::saveCommonCfg("rtk", Solvers::Mode::RTK, core->opts,
                                           this->cutoffDeg, this->sampleInterval, this->enabledSystems);
                    persistBaseCoord(this);
                    syncCommonToCore(this);

                    if (const std::string inputErr = ObsSource::rtkInputError(
                            this->baseObsPathBuf, this->roverObsPathBuf, this->navPathBuf);
                        !inputErr.empty()) {
                        ImGui::TextColored(UI::kColFail, "%s", inputErr.c_str());
                    } else {
                        this->core->filePath = this->roverObsPathBuf;
                        this->core->fileName = Str::baseName(this->roverObsPathBuf) + " / "
                                               + Str::baseName(this->baseObsPathBuf);
                        LOG_INFO << "[RTK] 开始解算: 基准=" << this->baseObsPathBuf
                                << ", 流动站=" << this->roverObsPathBuf;

                        if (this->worker.joinable()) this->worker.join();
                        this->stop = false;
                        this->core->stop = false;
                        this->core->initializedRefECEF = false;

                        this->state = State::Running;
                        this->loading = true;
                        this->core->loading = true;
                        this->startFile();
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
                        Helpers::saveCommonCfg("rtk", Solvers::Mode::RTK, core->opts,
                                               this->cutoffDeg, this->sampleInterval, this->enabledSystems);
                        persistBaseCoord(this);
                        syncCommonToCore(this);

                        this->isRealtime = true;
                        this->core->isRealtime = true;
                        this->core->mode = Solvers::Mode::RTK;
                        this->core->fileName = bip + ":" + bport + " / " + rip + ":" + rport;
                        this->core->filePath = this->core->fileName;
                        LOG_INFO << "[RTK] 开始实时解算: base=" << bip << ":" << bport
                                << ", rover=" << rip << ":" << rport;

                        this->startRealtime();
                    },
                    [&] { renderBaseCoordInput(this); });
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("参数")) {
                ImGui::Spacing();
                Helpers::renderAdvancedParams(core->opts, Solvers::Mode::RTK);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        Helpers::endConfigBody();
    }
}
