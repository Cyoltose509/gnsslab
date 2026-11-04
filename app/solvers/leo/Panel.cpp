#include "Task.h"

#include <algorithm>
#include <string>
#include <vector>

#include "Log.h"
#include "StringUtils.h"
#include "core/PathMemory.h"
#include "core/Products.h"
#include "imgui.h"
#include "solvers/view/GuiHelpers.h"
#include "solvers/view/GuiParamPage.h"
#include "solvers/view/GuiProductSection.h"
#include "ui/FileDialog.h"
#include "ui/Gui.h"

namespace Solvers::LEO {
    void Task::loadFromMemory() {
        const auto &cfg = AppConfig::PathMemory::instance();
        leoObsPathBuf = cfg.get("leo_obs");
        refSp3PathBuf = cfg.get("leo_ref_sp3");
        auxFiles.clear();
        for (auto &r: cfg.getRecent("leo_aux")) {
            if (std::find(auxFiles.begin(), auxFiles.end(), r) == auxFiles.end())
                auxFiles.push_back(r);
        }
        if (const std::string prnStr = cfg.get("leo_ref_prn"); !prnStr.empty()) {
            try { refPrn = std::stoi(prnStr); } catch (...) {}
        }
        Helpers::loadCommonCfg("leo", Mode::LEO, &core->opts,
                               &core->cutoffDeg, &core->sampleInterval, &core->enabledSystems);
        AppConfig::Products::of(modeKey(Mode::LEO)).load();
        refSat = SatID('L', refPrn);
    }

    void Task::renderConfigPanel() {
        auto &prod = AppConfig::Products::of(modeKey(Mode::LEO));
        Helpers::beginConfigBody("LEO 处理配置###leo_cfg_", *this);

        if (ImGui::BeginTabBar("##leo_cfg_tabs")) {
            if (ImGui::BeginTabItem("文件")) {
                ImGui::Spacing();

                Helpers::fileRow("观测文件", &leoObsPathBuf, "leo_obs", FileDialog::fObs, 3,
                                 {}, "leo_obs", &prod.brdc);
                Helpers::renderProductSection(prod, "leo_obs");
                Helpers::fileRow("参考轨道 SP3 (可选)", &refSp3PathBuf, "leo_ref_sp3", FileDialog::fSp3, 2, {}, "leo_obs");
                ImGui::Text("辅助文件:");
                if (!auxFiles.empty()) {
                    for (std::size_t i = 0; i < auxFiles.size(); i++) {
                        ImGui::Bullet();
                        std::string shortName = Str::baseName(auxFiles[i]);
                        ImGui::Text("%s", shortName.c_str());
                        ImGui::SameLine();
                        if (ImGui::SmallButton(("删除##aux" + std::to_string(i)).c_str()))
                            auxFiles.erase(auxFiles.begin() + i);
                    }
                }
                if (ImGui::Button("添加...")) {
                    if (std::vector<std::wstring> wpaths; ShowOpenFilesDialog(wpaths, FileDialog::fHdr, 2)) {
                        for (auto &wp: wpaths) {
                            std::string p = Str::wideToACP(wp);
                            if (std::find(auxFiles.begin(), auxFiles.end(), p) == auxFiles.end())
                                auxFiles.push_back(p);
                            AppConfig::PathMemory::instance().addRecent("leo_aux", p);
                        }
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("从记录添加")) ImGui::OpenPopup("##leo_aux_hist_pop");
                if (ImGui::BeginPopup("##leo_aux_hist_pop")) {
                    const auto recent = AppConfig::PathMemory::instance().getRecent("leo_aux");
                    if (recent.empty()) ImGui::TextDisabled("(暂无记录)");
                    for (auto &r: recent) {
                        if (ImGui::Selectable(r.c_str())) {
                            if (std::find(auxFiles.begin(), auxFiles.end(), r) == auxFiles.end())
                                auxFiles.push_back(r);
                            AppConfig::PathMemory::instance().addRecent("leo_aux", r);
                        }
                    }
                    ImGui::EndPopup();
                }
                ImGui::Separator();
                Helpers::renderSimpleConfig(&core->cutoffDeg, &core->enabledSystems, &core->sampleInterval);
                ImGui::Separator();
                if (ImGui::Button("开始解算", ImVec2(200, 40))) {
                    if (Helpers::beginFileSolve(this, leoObsPathBuf)) {
                        const std::string &path = leoObsPathBuf;
                        const auto &cfg = AppConfig::PathMemory::instance();
                        cfg.set("leo_obs", path);
                        prod.save();
                        cfg.set("leo_ref_sp3", refSp3PathBuf);
                        cfg.set("leo_ref_prn", std::to_string(refPrn));
                        Helpers::saveCommonCfg("leo", Mode::LEO, core->opts,
                                               core->cutoffDeg, core->sampleInterval, core->enabledSystems);

                        {
                            const std::string refInfo = refSp3PathBuf.empty() ? std::string("无") : refSp3PathBuf;
                            LOG_INFO << "[LEO] 开始定轨: 观测=" << path
                                     << ", 格式=" << (core->isRINEX ? "RINEX" : "NovAtel")
                                     << ", 参考轨道=" << refInfo;
                        }

                        startFile();
                    }
                }
                ImGui::SameLine();
                Helpers::renderCancelButton(this);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("参数")) {
                ImGui::Spacing();
                Helpers::renderAdvancedParams(core->opts, Mode::LEO);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        Helpers::endConfigBody();
    }
}
