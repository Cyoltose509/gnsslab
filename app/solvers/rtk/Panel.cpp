#include "ui/GuiColors.h"
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
    }

    void Task::loadFromMemory() {
        const auto &pm = AppConfig::PathMemory::instance();
        baseObsPathBuf = pm.get("rtk_base");
        roverObsPathBuf = pm.get("rtk_rover");
        navPathBuf = pm.get("rtk_nav");
        Helpers::loadSolverParams(Mode::RTK, opts);
        rtEndpoints.resize(2);
        rtEndpoints[0].ip = pm.get("rtk_rt_base_ip", "47.114.134.129");
        rtEndpoints[0].port = pm.get("rtk_rt_base_port", "7190");
        rtEndpoints[1].ip = pm.get("rtk_rt_rover_ip", "8.148.22.229");
        rtEndpoints[1].port = pm.get("rtk_rt_rover_port", "7201");
        if (const std::string manualStr = pm.get("rtk_base_manual"); !manualStr.empty() && manualStr != "0")
            useBaseCoord = true;
        if (const std::string xyzStr = pm.get("rtk_base_xyz"); !xyzStr.empty()) {
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
        auto *self = this;
        const auto onStart = [self] {
            const auto &cfg = AppConfig::PathMemory::instance();
            cfg.set("rtk_base", self->baseObsPathBuf);
            cfg.set("rtk_rover", self->roverObsPathBuf);
            cfg.set("rtk_nav", self->navPathBuf);
            Helpers::saveSolverParams(Mode::RTK, self->opts);
            persistBaseCoord(self);

            if (const std::string inputErr = ObsSource::rtkInputError(
                    self->baseObsPathBuf, self->roverObsPathBuf, self->navPathBuf);
                !inputErr.empty()) {
                ImGui::TextColored(UI::kColFail, "%s", inputErr.c_str());
            } else {
                self->isRealtime = false;
                self->filePath = self->roverObsPathBuf;
                self->fileName = Str::baseName(self->roverObsPathBuf) + " / "
                                + Str::baseName(self->baseObsPathBuf);
                LOG_INFO << "[RTK] 开始解算: 基准=" << self->baseObsPathBuf
                        << ", 流动站=" << self->roverObsPathBuf;
                self->startFile();
            }
        };
        const auto fileTab = [self] {
            Helpers::fileRow("基准站观测文件", &self->baseObsPathBuf, "rtk_base", FileDialog::fObs, 3,
                             {}, nullptr, &self->navPathBuf);
            Helpers::fileRow("流动站观测文件", &self->roverObsPathBuf, "rtk_rover", FileDialog::fObs, 3,
                             {}, nullptr, &self->navPathBuf);
            Helpers::fileRow("广播星历 BRDC", &self->navPathBuf, "rtk_nav", FileDialog::fRnx, 2, {}, "rtk_base");
            ImGui::Separator();
            renderBaseCoordInput(self);
        };
        const auto realtimeTab = [self] {
            const auto &cfg = AppConfig::PathMemory::instance();
            self->renderRealtimeConfig({"基准站", "流动站"},
                [self, &cfg] {
                    const std::string &bip = self->rtEndpoints[0].ip;
                    const std::string &bport = self->rtEndpoints[0].port;
                    const std::string &rip = self->rtEndpoints[1].ip;
                    const std::string &rport = self->rtEndpoints[1].port;
                    cfg.set("rtk_rt_base_ip", bip);
                    cfg.set("rtk_rt_base_port", bport);
                    cfg.set("rtk_rt_rover_ip", rip);
                    cfg.set("rtk_rt_rover_port", rport);
                    Helpers::saveSolverParams(Mode::RTK, self->opts);
                    persistBaseCoord(self);

                    self->isRealtime = true;
                    self->mode = Mode::RTK;
                    self->fileName = bip + ":" + bport + " / " + rip + ":" + rport;
                    self->filePath = self->fileName;
                    LOG_INFO << "[RTK] 开始实时解算: base=" << bip << ":" << bport
                            << ", rover=" << rip << ":" << rport;

                    self->startRealtime();
                },
                [self] { renderBaseCoordInput(self); });
        };
        self->renderConfigTabs("RTK 处理配置###rtk_cfg_", fileTab, onStart, realtimeTab);
    }
}
