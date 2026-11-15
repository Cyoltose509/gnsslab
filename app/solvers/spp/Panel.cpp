#include "Task.h"

#include <string>

#include "Log.h"
#include "core/PathMemory.h"
#include "io/ProductPaths.h"
#include "imgui.h"
#include "io/ProductReader.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "solvers/view/GuiProductSection.h"
#include "solvers/view/GuiRefTruth.h"
#include "ui/FileDialog.h"
#include "i18n/I18n.h"
#include "ui/ImGuiLabel.h"

namespace Solvers::SPP {
    void Task::loadFromMemory() {
        const auto &pm = AppConfig::PathMemory::instance();
        view.obsPathBuf = pm.get("spp_obs");
        navPathBuf = pm.get("spp_brdc");
        rtEndpoints.resize(1);
        rtEndpoints[0].ip = pm.get("spp_rt_ip", pm.get("rt_ip", "47.114.134.129"));
        rtEndpoints[0].port = pm.get("spp_rt_port", pm.get("rt_port", "7190"));
        Helpers::loadSolverParams(Mode::SPP, opts);
        AppConfig::ProductPaths::of(modeKey(Mode::SPP)).load();
        RefTruth::restore(this);
    }

    void Task::renderConfigPanel() {
        auto *self = this;
        const auto onStart = [self] {
            self->renderFileStart("spp_obs", false,
                [self] {
                    AppConfig::PathMemory::instance().set("spp_brdc", self->navPathBuf);
                    RefTruth::persist(self);
                },
                [self] { LOG_INFO << "[SPP] " << I18n::tr("log.start", self->view.obsPathBuf); });
        };
        const auto fileTab = [self] {
            Helpers::fileRow(I18n::trc("ObservationFile"), &self->view.obsPathBuf, "spp_obs",
                             FileDialog::fObs(), {}, "spp_obs", &self->navPathBuf);
            ImGui::Separator();
            Helpers::fileRow(I18n::trc("BroadcastEphemeris"), &self->navPathBuf, "spp_brdc", FileDialog::fRnx(), {}, "spp_obs");
            ImGui::Separator();
            RefTruth::renderRefTruthSection(self);
        };
        const auto realtimeTab = [self] {
            const auto &pm = AppConfig::PathMemory::instance();
            self->renderRealtimeConfig({I18n::tr("Rover")},
                [self, &pm] {
                    const std::string &ip = self->rtEndpoints[0].ip;
                    const std::string &port = self->rtEndpoints[0].port;
                    self->isRealtime = true;
                    self->fileName = ip + ":" + port;
                    LOG_INFO << "[SPP] " << I18n::tr("log.start_rt", ip + ":" + port);
                    pm.set("spp_rt_ip", ip);
                    pm.set("spp_rt_port", port);
                    Helpers::saveSolverParams(Mode::SPP, self->opts);
                    self->startRealtime();
                });
        };
        self->renderConfigTabs(UI::idLabel("spp.cfg_tab", "###spp_cfg_"), fileTab, onStart, realtimeTab);
    }
}
