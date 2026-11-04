#include "Task.h"

#include <string>

#include "Log.h"
#include "core/PathMemory.h"
#include "core/Products.h"
#include "imgui.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "solvers/view/GuiProductSection.h"
#include "ui/FileDialog.h"

namespace Solvers::PPP {
    void Task::loadFromMemory() {
        const auto &pm = AppConfig::PathMemory::instance();
        core->view.obsPathBuf = pm.get("ppp_obs");
        kinematic = pm.get("ppp_kinematic") == "1";
        Helpers::loadCommonCfg("ppp", Mode::PPP, &core->opts,
                               &core->cutoffDeg, &core->sampleInterval, &core->enabledSystems);
        AppConfig::Products::of(modeKey(Mode::PPP)).load();
    }

    void Task::renderConfigPanel() {
        auto &prod = AppConfig::Products::of(modeKey(Mode::PPP));
        Helpers::beginConfigBody("PPP 处理配置###ppp_cfg_", *this);

        if (ImGui::BeginTabBar("##ppp_cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                ImGui::Spacing();

                Helpers::fileRow("观测文件", &core->view.obsPathBuf, "ppp_obs", FileDialog::fObs, 3,
                                 {}, "ppp_obs", &prod.brdc);
                Helpers::renderProductSection(prod, "ppp_obs");
                ImGui::Separator();
                Helpers::renderSimpleConfig(&core->cutoffDeg, &core->enabledSystems, &core->sampleInterval);
                ImGui::Separator();
                ImGui::Checkbox("动态解算 (Kinematic)", &kinematic);
                ImGui::Separator();

                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    if (Helpers::beginFileSolve(this, core->view.obsPathBuf)) {
                        const std::string &path = core->view.obsPathBuf;

                        const auto &cfg = AppConfig::PathMemory::instance();
                        cfg.set("ppp_obs", path);
                        prod.save();
                        cfg.set("ppp_kinematic", kinematic ? "1" : "0");
                        Helpers::saveCommonCfg("ppp", Mode::PPP, core->opts,
                                               core->cutoffDeg, core->sampleInterval, core->enabledSystems);
                        LOG_INFO << "[PPP] 开始解算: 观测=" << path
                                << ", 格式=" << (core->isRINEX ? "RINEX" : "NovAtel")
                                << ", 动态=" << (kinematic ? "是" : "否");
                        startFile();
                    }
                }
                ImGui::SameLine();
                Helpers::renderCancelButton(this);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("参数")) {
                ImGui::Spacing();
                Helpers::renderAdvancedParams(core->opts, Mode::PPP);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        Helpers::endConfigBody();
    }
}
