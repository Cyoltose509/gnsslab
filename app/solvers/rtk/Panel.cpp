#include "Task.h"

#include <sstream>
#include <string>

#include "Log.h"
#include "Observer.h"
#include "StringUtils.h"
#include "core/PathMemory.h"
#include "imgui.h"
#include "io/ObsSource.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "solvers/view/GuiProductSection.h"
#include "ui/FileDialog.h"

namespace Solvers::RTK {
    namespace {
        void renderBaseCoordInput(Task *task) {
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

        void persistBaseCoord(const Task *task) {
            const auto &cfg = AppConfig::PathMemory::instance();
            cfg.set("rtk_base_manual", task->useBaseCoord ? "1" : "0");
            std::stringstream ss;
            ss << task->baseXyz.x() << ',' << task->baseXyz.y() << ',' << task->baseXyz.z();
            cfg.set("rtk_base_xyz", ss.str());
        }

        // Task 的公共配置是自己的成员（非 core 的），解算/实时线程读 core，启动前同步过去
        void syncCommonToCore(Task *task) {
            task->core->enabledSystems = task->enabledSystems;
            task->core->cutoffDeg = task->cutoffDeg;
            task->core->sampleInterval = task->sampleInterval;
        }
    }

    void Task::loadFromMemory() {
        const auto &pm = AppConfig::PathMemory::instance();
        baseObsPathBuf = pm.get("rtk_base");
        roverObsPathBuf = pm.get("rtk_rover");
        navPathBuf = pm.get("rtk_nav");
        Helpers::loadCommonCfg("rtk", Mode::RTK, &core->opts,
                               &cutoffDeg, &sampleInterval, &enabledSystems);
        core->rtEndpoints.resize(2);
        core->rtEndpoints[0].ip = pm.get("rtk_rt_base_ip", "8.148.22.229");
        core->rtEndpoints[0].port = pm.get("rtk_rt_base_port", "2002");
        core->rtEndpoints[1].ip = pm.get("rtk_rt_rover_ip", "8.148.22.229");
        core->rtEndpoints[1].port = pm.get("rtk_rt_rover_port", "3002");
        if (const std::string manualStr = pm.get("rtk_base_manual"); !manualStr.empty() && manualStr != "0")
            useBaseCoord = true;
        if (const std::string xyzStr = pm.get("rtk_base_xyz"); !xyzStr.empty()) {
            // "x,y,z"：三段都得是数，缺一段/有一段不是数就整份丢掉 —— 别把解析失败当 0 用
            const auto parts = Str::split(xyzStr, ',', false);
            double v[3]{0, 0, 0};
            bool ok = parts.size() == 3;
            for (std::size_t i = 0; ok && i < 3; ++i) {
                const char *cur = parts[i].c_str();
                ok = Str::nextDouble(cur, cur + parts[i].size(), v[i]);
            }
            if (ok) baseXyz = Eigen::Vector3d(v[0], v[1], v[2]);
        }
    }

    void Task::renderConfigPanel() {
        Helpers::beginConfigBody("RTK 处理配置###rtk_cfg_", *this);

        if (ImGui::BeginTabBar("##rtk_cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                Helpers::fileRow("基准站观测文件", &baseObsPathBuf, "rtk_base", FileDialog::fObs, 3,
                                 {}, nullptr, &navPathBuf);
                Helpers::fileRow("流动站观测文件", &roverObsPathBuf, "rtk_rover", FileDialog::fObs, 3,
                                 {}, nullptr, &navPathBuf);
                Helpers::fileRow("广播星历 BRDC", &navPathBuf, "rtk_nav", FileDialog::fRnx, 2, {}, "rtk_base");
                ImGui::Separator();
                Helpers::renderSimpleConfig(&cutoffDeg, &enabledSystems, &sampleInterval);

                ImGui::Separator();
                renderBaseCoordInput(this);

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    const auto &cfg = AppConfig::PathMemory::instance();
                    cfg.set("rtk_base", baseObsPathBuf);
                    cfg.set("rtk_rover", roverObsPathBuf);
                    cfg.set("rtk_nav", navPathBuf);
                    Helpers::saveCommonCfg("rtk", Mode::RTK, core->opts,
                                           cutoffDeg, sampleInterval, enabledSystems);
                    persistBaseCoord(this);
                    syncCommonToCore(this);

                    if (const std::string inputErr = ObsSource::rtkInputError(
                            baseObsPathBuf, roverObsPathBuf, navPathBuf);
                        !inputErr.empty()) {
                        ImGui::TextColored(UI::kColFail, "%s", inputErr.c_str());
                    } else {
                        core->filePath = roverObsPathBuf;
                        core->fileName = Str::baseName(roverObsPathBuf) + " / "
                                         + Str::baseName(baseObsPathBuf);
                        LOG_INFO << "[RTK] 开始解算: 基准=" << baseObsPathBuf
                                << ", 流动站=" << roverObsPathBuf;

                        if (worker.joinable()) worker.join();
                        stop = false;
                        core->stop = false;
                        core->initializedRefECEF = false;

                        state = State::Running;
                        loading = true;
                        core->loading = true;
                        startFile();
                    }
                }
                ImGui::SameLine();
                Helpers::renderCancelButton(this);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("实时")) {
                const auto &cfg = AppConfig::PathMemory::instance();
                Helpers::renderRealtimeConfig(
                    this, &core->rtEndpoints, {"基准站", "流动站"},
                    &cutoffDeg, &enabledSystems, &sampleInterval,
                    [&] {
                        const std::string &bip = core->rtEndpoints[0].ip;
                        const std::string &bport = core->rtEndpoints[0].port;
                        const std::string &rip = core->rtEndpoints[1].ip;
                        const std::string &rport = core->rtEndpoints[1].port;
                        cfg.set("rtk_rt_base_ip", bip);
                        cfg.set("rtk_rt_base_port", bport);
                        cfg.set("rtk_rt_rover_ip", rip);
                        cfg.set("rtk_rt_rover_port", rport);
                        Helpers::saveCommonCfg("rtk", Mode::RTK, core->opts,
                                               cutoffDeg, sampleInterval, enabledSystems);
                        persistBaseCoord(this);
                        syncCommonToCore(this);

                        isRealtime = true;
                        core->isRealtime = true;
                        core->mode = Mode::RTK;
                        core->fileName = bip + ":" + bport + " / " + rip + ":" + rport;
                        core->filePath = core->fileName;
                        LOG_INFO << "[RTK] 开始实时解算: base=" << bip << ":" << bport
                                << ", rover=" << rip << ":" << rport;

                        startRealtime();
                    },
                    [&] { renderBaseCoordInput(this); });
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("参数")) {
                ImGui::Spacing();
                Helpers::renderAdvancedParams(core->opts, Mode::RTK);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        Helpers::endConfigBody();
    }
}
