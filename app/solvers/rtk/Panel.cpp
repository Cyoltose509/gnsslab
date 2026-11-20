#include "ui/GuiColors.h"
#include "FileUtils.h"
#include "Task.h"

#include <iomanip>
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
            ss << std::setprecision(17) << task->baseXyz.x() << ','
               << task->baseXyz.y() << ',' << task->baseXyz.z();
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
        const auto onStart = [this] {
            const auto &cfg = AppConfig::PathMemory::instance();
            cfg.set("rtk_base", baseObsPathBuf);
            cfg.set("rtk_rover", view.obsPathBuf);
            cfg.set("rtk_nav", navPathBuf);
            Helpers::saveSolverParams(Mode::RTK, opts);
            persistBaseCoord(this);
            RefTruth::persist(this);

            if (const std::string inputErr = ObsSource::RTKInputError(
                    baseObsPathBuf, view.obsPathBuf, navPathBuf);
                !inputErr.empty()) {
                ImGui::TextColored(UI::kColFail, "%s", inputErr.c_str());
            } else {
                isRealtime = false;
                applyObsPath(view.obsPathBuf);
                fileName = File::baseName(view.obsPathBuf) + " / "
                                + File::baseName(baseObsPathBuf);
                LOG_INFO << "[RTK] " << I18n::tr("rtk.log.start_file", baseObsPathBuf,
                                                                view.obsPathBuf);
                startFile();
            }
        };
        const auto fileTab = [this] {
            Helpers::fileRow(I18n::trc("rtk.base_obs_file"), &baseObsPathBuf, "rtk_base", FileDialog::fObs(),
                             {}, "rtk_base", &navPathBuf);
            Helpers::fileRow(I18n::trc("rtk.rover_obs_file"), &view.obsPathBuf, "rtk_rover", FileDialog::fObs(),
                             {}, "rtk_rover", &navPathBuf);
            Helpers::fileRow(I18n::trc("BroadcastEphemeris"), &navPathBuf, "rtk_nav", FileDialog::fRnx(), {}, "rtk_rover");
            ImGui::Separator();
            renderBaseCoordInput(this);
            ImGui::Separator();
            RefTruth::renderRefTruthSection(this);
        };
        const auto realtimeTab = [this] {
            const auto &cfg = AppConfig::PathMemory::instance();
            renderRealtimeConfig({I18n::tr("BaseStation"), I18n::tr("Rover")},
                [this, &cfg] {
                    const std::string &bip = rtEndpoints[0].ip;
                    const std::string &bport = rtEndpoints[0].port;
                    const std::string &rip = rtEndpoints[1].ip;
                    const std::string &rport = rtEndpoints[1].port;
                    cfg.set("rtk_rt_base_ip", bip);
                    cfg.set("rtk_rt_base_port", bport);
                    cfg.set("rtk_rt_rover_ip", rip);
                    cfg.set("rtk_rt_rover_port", rport);
                    Helpers::saveSolverParams(Mode::RTK, opts);
                    persistBaseCoord(this);
                    RefTruth::persist(this);

                    isRealtime = true;
                    mode = Mode::RTK;
                    fileName = bip + ":" + bport + " / " + rip + ":" + rport;
                    filePath = fileName;
                    LOG_INFO << "[RTK] " << I18n::tr("rtk.log.start_rt", bip + ":" + bport,
                                                                  rip + ":" + rport);

                    startRealtime();
                },
                [this] {
                    renderBaseCoordInput(this);
                    ImGui::Separator();
                    RefTruth::renderRefTruthSection(this, true);
                });
        };
        renderConfigTabs(UI::idLabel("rtk.cfg_tab", "###rtk_cfg_"), fileTab, onStart, realtimeTab);
    }
}
