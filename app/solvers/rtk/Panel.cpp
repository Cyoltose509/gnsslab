#include "ui/GuiColors.h"
#include "FileUtils.h"
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
#include "solvers/view/GuiRefTruth.h"
#include "ui/FileDialog.h"
#include "i18n/I18n.h"
#include "ui/ImGuiLabel.h"

namespace Solvers::RTK {
    namespace {
        void renderBaseCoordInput(Task *task) {
            ImGui::Checkbox(I18n::trc("rtk.manual_base"), &task->useBaseCoord);
            if (!task->useBaseCoord) return;
            ImGui::PushItemWidth(200);
            ImGui::InputDouble("X##rtk_base_x", &task->baseXyz.x());
            ImGui::SameLine();
            ImGui::InputDouble("Y##rtk_base_y", &task->baseXyz.y());
            ImGui::SameLine();
            ImGui::InputDouble("Z##rtk_base_z", &task->baseXyz.z());
            ImGui::PopItemWidth();
            ImGui::SameLine();
            if (const auto p = Helpers::clipboardXYZButton("rtk_base_clip"); p.count > 0)
                for (int i = 0; i < p.count; i++) task->baseXyz[i] = p.xyz[i];
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
        view.obsPathBuf = pm.get("rtk_rover");
        navPathBuf = pm.get("rtk_nav");
        Helpers::loadSolverParams(Mode::RTK, opts);
        RefTruth::restore(this);
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
                ok = Str::next(cur, cur + parts[i].size(), v[i]);
            }
            if (ok) baseXyz = Eigen::Vector3d(v[0], v[1], v[2]);
        }
    }

    void Task::renderConfigPanel() {
        auto *self = this;
        const auto onStart = [self] {
            const auto &cfg = AppConfig::PathMemory::instance();
            cfg.set("rtk_base", self->baseObsPathBuf);
            cfg.set("rtk_rover", self->view.obsPathBuf);
            cfg.set("rtk_nav", self->navPathBuf);
            Helpers::saveSolverParams(Mode::RTK, self->opts);
            persistBaseCoord(self);
            RefTruth::persist(self);

            if (const std::string inputErr = ObsSource::RTKInputError(
                    self->baseObsPathBuf, self->view.obsPathBuf, self->navPathBuf);
                !inputErr.empty()) {
                ImGui::TextColored(UI::kColFail, "%s", inputErr.c_str());
            } else {
                self->isRealtime = false;
                self->applyObsPath(self->view.obsPathBuf);
                self->fileName = File::baseName(self->view.obsPathBuf) + " / "
                                + File::baseName(self->baseObsPathBuf);
                LOG_INFO << "[RTK] " << I18n::tr("rtk.log.start_file", self->baseObsPathBuf,
                                                                self->view.obsPathBuf);
                self->startFile();
            }
        };
        const auto fileTab = [self] {
            Helpers::fileRow(I18n::trc("rtk.base_obs_file"), &self->baseObsPathBuf, "rtk_base", FileDialog::fObs(),
                             {}, "rtk_base", &self->navPathBuf);
            Helpers::fileRow(I18n::trc("rtk.rover_obs_file"), &self->view.obsPathBuf, "rtk_rover", FileDialog::fObs(),
                             {}, "rtk_rover", &self->navPathBuf);
            Helpers::fileRow(I18n::trc("BroadcastEphemeris"), &self->navPathBuf, "rtk_nav", FileDialog::fRnx(), {}, "rtk_rover");
            ImGui::Separator();
            renderBaseCoordInput(self);
            ImGui::Separator();
            RefTruth::renderRefTruthSection(self);
        };
        const auto realtimeTab = [self] {
            const auto &cfg = AppConfig::PathMemory::instance();
            self->renderRealtimeConfig({I18n::tr("BaseStation"), I18n::tr("Rover")},
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
                    LOG_INFO << "[RTK] " << I18n::tr("rtk.log.start_rt", bip + ":" + bport,
                                                                  rip + ":" + rport);

                    self->startRealtime();
                },
                [self] { renderBaseCoordInput(self); });
        };
        self->renderConfigTabs(UI::idLabel("rtk.cfg_tab", "###rtk_cfg_"), fileTab, onStart, realtimeTab);
    }
}
