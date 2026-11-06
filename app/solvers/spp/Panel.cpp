#include "Task.h"

#include <string>

#include "Log.h"
#include "core/PathMemory.h"
#include "io/Products.h"
#include "imgui.h"
#include "io/ProductLoad.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "solvers/view/GuiProductSection.h"
#include "solvers/view/GuiRefTruth.h"
#include "ui/FileDialog.h"

namespace Solvers::SPP {
    void Task::loadFromMemory() {
        const auto &pm = AppConfig::PathMemory::instance();
        view.obsPathBuf = pm.get("spp_obs");
        navPathBuf = pm.get("spp_brdc");
        rtEndpoints.resize(1);
        rtEndpoints[0].ip = pm.get("spp_rt_ip", pm.get("rt_ip", "47.114.134.129"));
        rtEndpoints[0].port = pm.get("spp_rt_port", pm.get("rt_port", "7190"));
        Helpers::loadSolverParams(Mode::SPP, opts);
        AppConfig::Products::of(modeKey(Mode::SPP)).load();
        RefTruth::restore(this, "spp_");
    }

    void Task::renderConfigPanel() {
        auto *self = this;
        const auto onStart = [self] {
            self->renderFileStart("spp_obs", false,
                [self] {
                    AppConfig::PathMemory::instance().set("spp_brdc", self->navPathBuf);
                    RefTruth::persist(self, "spp_");
                },
                [self] { LOG_INFO << "[SPP] 开始解算: 观测=" << self->view.obsPathBuf; });
        };
        const auto fileTab = [self] {
            Helpers::fileRow("观测文件", &self->view.obsPathBuf, "spp_obs",
                             FileDialog::fObs, 3, {}, "spp_obs", &self->navPathBuf);
            ImGui::Separator();
            Helpers::fileRow("广播星历 BRDC", &self->navPathBuf, "spp_brdc", FileDialog::fRnx, 2, {}, "spp_obs");
            ImGui::Separator();
            RefTruth::renderRefTruthField(self);
        };
        const auto realtimeTab = [self] {
            const auto &pm = AppConfig::PathMemory::instance();
            self->renderRealtimeConfig({"流动站"},
                [self, &pm] {
                    const std::string &ip = self->rtEndpoints[0].ip;
                    const std::string &port = self->rtEndpoints[0].port;
                    self->isRealtime = true;
                    self->fileName = ip + ":" + port;
                    LOG_INFO << "[SPP] 开始实时解算: " << ip << ":" << port;
                    pm.set("spp_rt_ip", ip);
                    pm.set("spp_rt_port", port);
                    Helpers::saveSolverParams(Mode::SPP, self->opts);
                    self->startRealtime();
                });
        };
        self->renderConfigTabs("SPP 处理配置###spp_cfg_", fileTab, onStart, realtimeTab);
    }
}
