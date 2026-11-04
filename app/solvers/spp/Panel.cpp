#include "Task.h"

#include <string>

#include "Log.h"
#include "core/PathMemory.h"
#include "core/Products.h"
#include "download/ProductCatalog.h"
#include "download/ProductStore.h"
#include "imgui.h"
#include "io/ProductLoad.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "solvers/view/GuiProductSection.h"
#include "ui/FileDialog.h"

namespace Solvers::SPP {
    void Task::loadFromMemory() const {
        const auto &pm = AppConfig::PathMemory::instance();
        core->view.obsPathBuf = pm.get("spp_obs");
        core->brdcPathBuf = pm.get("spp_brdc");
        core->rtEndpoints.resize(1);
        core->rtEndpoints[0].ip = pm.get("spp_rt_ip", pm.get("rt_ip", "47.114.134.129"));
        core->rtEndpoints[0].port = pm.get("spp_rt_port", pm.get("rt_port", "7190"));
        Helpers::loadCommonCfg("spp", Mode::SPP, &core->opts,
                               &core->cutoffDeg, &core->sampleInterval, &core->enabledSystems);
        AppConfig::Products::of(modeKey(Mode::SPP)).load();
    }

    void Task::renderConfigPanel() {
        Helpers::beginConfigBody("处理配置###cfg_", *this);

        if (ImGui::BeginTabBar("##spp_cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                Helpers::fileRow("观测文件", &core->view.obsPathBuf, "spp_obs",
                                 FileDialog::fObs, 3, {}, "spp_obs", &core->brdcPathBuf);
                ImGui::Separator();
                Helpers::fileRow("广播星历 BRDC", &core->brdcPathBuf, "spp_brdc", FileDialog::fRnx, 2, {}, "spp_obs");
                ImGui::Separator();

                Helpers::renderSimpleConfig(&core->cutoffDeg, &core->enabledSystems, &core->sampleInterval);

                ImGui::Separator();

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    if (Helpers::beginFileSolve(this, core->view.obsPathBuf)) {
                        const std::string &path = core->view.obsPathBuf;
                        AppConfig::PathMemory::instance().set("spp_obs", path);
                        AppConfig::PathMemory::instance().set("spp_brdc", core->brdcPathBuf);
                        Helpers::saveCommonCfg("spp", Mode::SPP, core->opts,
                                               core->cutoffDeg, core->sampleInterval, core->enabledSystems);
                        LOG_INFO << "[SPP] 开始解算: 观测=" << path;
                        this->startFile();
                    }
                }
                ImGui::SameLine();
                Helpers::renderCancelButton(this);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("实时")) {
                const auto &pm = AppConfig::PathMemory::instance();
                Helpers::renderRealtimeConfig(
                    this, &core->rtEndpoints, {"流动站"},
                    &core->cutoffDeg, &core->enabledSystems, &core->sampleInterval,
                    [&] {
                        const std::string &ip = core->rtEndpoints[0].ip;
                        const std::string &port = core->rtEndpoints[0].port;
                        core->isRealtime = true;
                        isRealtime = true;
                        core->fileName = ip + ":" + port;
                        LOG_INFO << "[SPP] 开始实时解算: " << ip << ":" << port;
                        pm.set("spp_rt_ip", ip);
                        pm.set("spp_rt_port", port);
                        Helpers::saveCommonCfg("spp", Mode::SPP, core->opts,
                                               core->cutoffDeg, core->sampleInterval, core->enabledSystems);
                        this->startRealtime();
                    });
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("参数")) {
                ImGui::Spacing();
                Helpers::renderAdvancedParams(core->opts, Mode::SPP);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        Helpers::endConfigBody();
    }
}
